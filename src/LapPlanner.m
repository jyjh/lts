classdef LapPlanner
    % LAPPLANNER Build a racing line and a skill-governed velocity envelope.
    %
    % A pure, stateless planning layer used once at simulation start. It
    % replaces the legacy centerline-only GGV sweep with a two-stage plan:
    %
    %   1. buildRacingLine   - a smooth minimum-curvature line that uses the
    %                          full track width (how a real driver commits).
    %   2. buildVelocityProfile - a forward/backward speed sweep along that
    %                          line, governed by a single driverSkill margin
    %                          so the car runs a consistent fraction below
    %                          the grip limit (the key to correlating data).
    %
    % Both the racing line and the velocity profile are indexed identically to
    % the track centerline (trackData.points), so the live driver can reference
    % them by the same observation.idx it already receives from the simulator.
    %
    % Capability estimation (tire mu + aero downforce + powertrain + brake
    % bias) lives here as a static method so the planner and the live driver
    % agree on grip at every speed.

    methods (Static)
        function line = buildRacingLine(trackData, opts)
            % BUILDRACINGLINE Minimum-curvature line within the track limits.
            %
            %   line = LapPlanner.buildRacingLine(trackData)
            %   line = LapPlanner.buildRacingLine(trackData, opts)
            %
            % Returns a struct with the racing-line geometry, indexed to match
            % trackData.points:
            %   points    - Nx2 racing-line waypoints [m]
            %   arcLen    - Nx1 cumulative arc length [m]
            %   curvature - Nx1 signed curvature [1/m]
            %   heading   - Nx1 unwrapped heading [rad]
            %   offsetW   - Nx1 lateral offset from centerline [m] (+ = left)
            %
            % The line is found by projected Gauss-Seidel on a discrete
            % bending-energy objective (second differences of the offset
            % profile) plus a term that pulls each waypoint toward the inside
            % of any local corner. Unlike per-waypoint curvature minimization
            % — which gets stuck at the all-zero line because cutting a corner
            % needs neighbors to move together — this QP-like objective is
            % unimodal in each variable, so coordinate sweeps converge
            % reliably and the bounds keep the line inside the track.

            points = trackData.points;
            n = size(points, 1);
            closed = LapPlanner.fieldOr(trackData, 'closedLoop', false);
            heading = trackData.heading(:);
            centerKappa = trackData.curvature(:);
            halfWidth = LapPlanner.fieldOr(trackData, 'trackHalfWidth', 1.5);

            if nargin < 2 || isempty(opts)
                opts = struct();
            end
            lineUsage = LapPlanner.fieldOr(opts, 'lineUsage', 0.78);
            maxPasses = LapPlanner.fieldOr(opts, 'maxPasses', 60);
            smoothSpan = LapPlanner.fieldOr(opts, 'smoothSpan', 9);

            % Maximum lateral offset. The line must stay clear of the track
            % edge by `lineEdgeMargin` so the closed-loop driver has room to
            % follow it without overshooting off-track. lineUsage is a further
            % fraction of what remains.
            lineEdgeMargin = LapPlanner.fieldOr(opts, 'lineEdgeMargin', 0.50);
            wMax = (halfWidth - lineEdgeMargin) * max(0.5, min(0.99, lineUsage));
            wMax = max(0, wMax);

            % Left-pointing normal at each centerline waypoint. The simulator's
            % lateralError convention is + = vehicle left of centerline, so a
            % +offset moves the racing line left, matching that sign.
            nx = -sin(heading);
            ny = cos(heading);

            % The racing line is found in two phases:
            %
            %  (1) Seed. Start from a path that already uses the inside of each
            %      corner: at every waypoint push the offset toward the inside
            %      normal of the local curvature, scaled by how sharp the corner
            %      is. This gives the optimizer something to smooth rather than
            %      the centerline (which is a local minimum it cannot leave by
            %      smoothing alone).
            %
            %  (2) Smooth (elastic band). Repeatedly move each interior waypoint
            %      toward the midpoint of its neighbors — the move that most
            %      reduces local curvature — while clamping to the track
            %      corridor (|w| <= wMax) and pinning straights to centerline.
            %      This is a coordinate-descent minimizer of summed curvature^2
            %      that converges to a smooth, corner-cutting line. Because the
            %      seed already sits at the inside, smoothing rounds the corner
            %      off into a classic late-apex line instead of collapsing to
            %      the centerline.

            % --- Robust corner detection. Raw discrete curvature on coarse or
            % noisy centerlines is unreliable (it can read ~0 at a real apex),
            % so detect corners by the integrated heading change over a window
            % and smooth the curvature for the inside-offset seed.
            kSmooth = LapPlanner.smoothVector(abs(centerKappa), max(3, round(n/40)), closed);
            turnSign = sign(centerKappa);
            % Inside-offset target: ramps up with smoothed curvature, squared
            % so only genuinely sharp corners reach the full inside offset and
            % approach/exit straights stay near the centerline.
            kPeak = max(kSmooth);
            if kPeak <= eps
                wSeed = zeros(n, 1);
            else
                ramp = min(kSmooth / kPeak, 1).^2;
                wSeed = -turnSign .* ramp .* wMax;
            end

            w = wSeed;
            straightTol = max(1e-4, 1e-3 * kPeak);
            for pass = 1:maxPasses %#ok<NASGU>
                maxChange = 0;
                for i = 1:n
                    if ~closed && (i == 1 || i == n)
                        continue;   % open-track endpoints pinned
                    end
                    % On genuine straights, hold the centerline.
                    if kSmooth(i) <= straightTol
                        maxChange = max(maxChange, abs(w(i)));
                        w(i) = 0;
                        continue;
                    end
                    ip = LapPlanner.neighbor(i, n, closed, -1);
                    in = LapPlanner.neighbor(i, n, closed, +1);
                    pp = points(ip, :) + w(ip) * [nx(ip), ny(ip)];
                    pn = points(in, :) + w(in) * [nx(in), ny(in)];
                    base = points(i, :);
                    % Move point i onto the line through its neighbors (the
                    % collinear configuration has zero local curvature). Project
                    % the current point onto the pp->pn segment, then back out
                    % the offset along the centerline normal at i.
                    wNew = LapPlanner.collinearOffset( ...
                        pp, pn, base, [nx(i), ny(i)]);
                    wNew = max(-wMax, min(wMax, wNew));
                    maxChange = max(maxChange, abs(wNew - w(i)));
                    w(i) = wNew;
                end
                if maxChange < 1e-4 * halfWidth
                    break;
                end
            end

            % Light smoothing of the offset profile to remove any kinks.
            w = LapPlanner.smoothOffset(w, smoothSpan, closed);

            % Re-pin open-track endpoints to the centerline after smoothing
            % (the moving average drifts them inward). The car starts on the
            % centerline at s=0, so the line must meet it there or the driver
            % would immediately chase a large cross-track error.
            if ~closed
                w(1) = 0;
                w(n) = 0;
            end

            % Feasibility pass: where the racing-line curvature exceeds what the
            % steering can physically command, blend those offsets back toward
            % the centerline until the line is followable. The tightest turn the
            % car can make is R_min = wheelbase / tan(maxSteer), so the max
            % followable curvature is tan(maxSteer)/wheelbase. An infeasibly
            % tight line (an optimizer kink on a sharp corner) would otherwise
            % command a steer the wheels cannot reach and the car would run off.
            maxSteerAng = LapPlanner.fieldOr(opts, 'maxSteeringAngle', 0.6);
            wheelbase = LapPlanner.fieldOr(opts, 'wheelbase', 1.558);
            kappaSteerMax = tan(maxSteerAng) / max(wheelbase, eps);
            w = LapPlanner.feasibilityClamp(points, nx, ny, w, closed, kappaSteerMax);

            % Build the racing-line polyline from centerline + offsets.
            linePoints = points + w .* [nx, ny];

            arcLen = components.Track.cumulativeArcLength(linePoints, closed);
            curvature = components.Track.computeCurvature(linePoints, closed);
            lineHeading = components.Track.computeHeading(linePoints, closed);

            line.points = linePoints;
            line.arcLen = arcLen(:);
            line.curvature = curvature(:);
            line.heading = lineHeading(:);
            line.offsetW = w(:);
            line.trackHalfWidth = halfWidth;
            line.edgeMargin = lineEdgeMargin;
            line.maxOffsetW = wMax;
            line.closedLoop = closed;
        end

        function profile = buildVelocityProfile(line, vehicleManager, initialState, opts)
            % BUILDVELOCITYPROFILE Forward/backward speed sweep along a line.
            %
            %   profile = LapPlanner.buildVelocityProfile(line, vm, initialState)
            %
            % Produces a skill-governed velocity envelope and the resulting
            % longitudinal accel reference, indexed to match the racing line:
            %   vTarget - Nx1 speed plan (what the car should be doing) [m/s]
            %   vLimit  - Nx1 corner-speed limit (before braking/accel) [m/s]
            %   axRef   - Nx1 longitudinal accel reference [m/s^2]
            %
            % A single driverSkill fraction is applied to cornering, braking,
            % and drive capability so the car holds a consistent grip margin.
            % Iterating the corner-speed solve lets speed-dependent aero
            % downforce feed back into the grip envelope.

            if nargin < 4 || isempty(opts)
                opts = struct();
            end
            skill = LapPlanner.fieldOr(opts, 'driverSkill', 0.93);
            skill = max(0.5, min(0.999, skill));
            nIters = LapPlanner.fieldOr(opts, 'cornerSpeedIters', 3);

            n = numel(line.curvature);
            arcLen = line.arcLen;
            vMax = vehicleManager.maxSpeed;

            % Curvature used for corner-speed targets. Use the MAX |kappa| over
            % a small window (a few metres) rather than the point value or a
            % mean: a mean would UNDER-estimate a sharp corner (averaging it
            % with gentler neighbors) and command an unsafe entry speed, while
            % the max is conservative and immune to single-point noise because
            % it considers the neighborhood. This keeps corner speeds safe.
            closed = LapPlanner.fieldOr(line, 'closedLoop', false);
            kForSpeed = LapPlanner.maxAbsVector(line.curvature, 5, closed);

            vLimit = vMax * ones(n, 1);

            % Maximum curvature the steering can actually command: tan(maxSteer)/L.
            % A line point whose curvature exceeds this cannot be followed at any
            % speed (the wheels cannot turn far enough), so cap the speed-limiting
            % curvature at this value to keep targets feasible. The car will run
            % wide there rather than be commanded an impossible steer.
            kappaSteerMax = tan(0.6) / max(vehicleManager.wheelbase, eps);
            kForSpeed = min(kForSpeed, kappaSteerMax);

            % Corner-speed limit: v = sqrt(skill * maxLatAccel / |kappa|).
            % Iterate so speed-dependent downforce enters maxLatAccel.
            for iter = 1:nIters %#ok<NASGU>
                for i = 1:n
                    k = kForSpeed(i);
                    if k > 1e-6
                        caps = LapPlanner.estimateCapability( ...
                            vehicleManager, initialState, vLimit(i));
                        vLimit(i) = min(vMax, ...
                            sqrt(max(skill * caps.maxLatAccel, 0.1) / k));
                    else
                        vLimit(i) = vMax;
                    end
                end
            end

            % Brake/drive capability at the corner-limited envelope speed.
            maxBrakeAccel = zeros(n, 1);
            maxDriveAccel = zeros(n, 1);
            F_drive_full = zeros(n, 1);
            F_resistance = zeros(n, 1);
            brakeForceAccel = zeros(n, 1);
            for i = 1:n
                caps = LapPlanner.estimateCapability( ...
                    vehicleManager, initialState, vLimit(i));
                maxBrakeAccel(i) = skill * caps.maxBrakeAccel;
                maxDriveAccel(i) = skill * caps.maxDriveAccel;
                F_drive_full(i) = caps.F_drive_full;
                F_resistance(i) = caps.F_resistance;
                brakeForceAccel(i) = caps.brakeForceAccel;
            end

            vTarget = vLimit;

            % Backward (braking) sweep: latest feasible braking point upstream.
            % A braking safety factor < 1 plans for less decel than the car can
            % theoretically deliver, so the speed plan brakes EARLIER and gentler
            % than the envelope — giving the closed-loop driver margin to absorb
            % transient grip loss on tight, noisy corners without arriving too
            % fast and spinning.
            brakeSafety = LapPlanner.fieldOr(opts, 'brakeSafety', 0.70);
            brakeSafety = max(0.3, min(1.0, brakeSafety));
            for i = n-1:-1:1
                ds = max(arcLen(i+1) - arcLen(i), 0.001);
                aBrake = maxBrakeAccel(i+1) * brakeSafety;
                reachable = sqrt(vTarget(i+1)^2 + 2 * aBrake * ds);
                vTarget(i) = min(vTarget(i), reachable);
            end

            % Forward (acceleration) sweep: traction/power-limited corner exit.
            vPlan = vTarget;
            vPlan(1) = min(max(initialState.speed, 0), vTarget(1));
            for i = 1:n-1
                ds = max(arcLen(i+1) - arcLen(i), 0.001);
                axCap = max(maxDriveAccel(i), 0);
                reachable = sqrt(vPlan(i)^2 + 2 * axCap * ds);
                vPlan(i+1) = min(vTarget(i+1), reachable);
            end

            axRef = zeros(n, 1);
            for i = 1:n-1
                ds = max(arcLen(i+1) - arcLen(i), 0.001);
                axRef(i) = (vPlan(i+1)^2 - vPlan(i)^2) / (2 * ds);
            end
            axRef(n) = axRef(max(n-1, 1));

            profile.vTarget = vPlan(:);
            profile.vLimit = vLimit(:);
            profile.axRef = axRef(:);
            profile.F_drive_full = F_drive_full(:);
            profile.F_resistance = F_resistance(:);
            profile.brakeForceAccel = brakeForceAccel(:);
        end

        function caps = estimateCapability(vehicleManager, templateState, speed)
            % ESTIMATECAPABILITY Vehicle longitudinal/lateral capability at a speed.
            %
            %   caps = LapPlanner.estimateCapability(vm, templateState, speed)
            %
            % Returns the grip/drive/brake envelope at the given speed, using the
            % tire model's peak mu (with load sensitivity), aero downforce/drag,
            % the powertrain drive map, and the brake bias. This is the single
            % source of truth for grip used by both the planner and the live
            % driver, so the planned speed envelope and the runtime traction
            % circle always agree.
            %
            % Fields:
            %   maxLatAccel     - lateral grip accel [m/s^2]
            %   maxBrakeAccel   - total decel capability [m/s^2]
            %   F_drive_full    - full-throttle wheel force, traction-capped [N]
            %   F_resistance    - drag + rolling resistance at this speed [N]
            %   maxDriveAccel   - net forward accel capability [m/s^2]
            %   brakeForceAccel - hydraulic-brake decel per unit brake [m/s^2]
            vm = vehicleManager;
            tempState = templateState;
            tempState.vehicleManager = vm;
            tempState.speed = max(speed, 0);
            tempState.vx = tempState.speed;
            tempState.vy = 0;

            aeroForces = vm.aero.computeForces(tempState);
            F_drag = max(0, aeroForces.F_drag);
            totalNormalLoad = vm.totalMass * vm.g + ...
                aeroForces.Fz_front + aeroForces.Fz_rear;

            % Grip comes entirely from the tire model (Pacejka peak mu with load
            % sensitivity). Dry FSAE rubber, no surface-mu cap.
            peakMu = max(vm.tire.getPeakFriction(totalNormalLoad / 4), 0);
            tireAccel = peakMu * totalNormalLoad / vm.totalMass;

            % Brake force: hydraulic capacity capped by per-axle grip at the
            % fixed front/rear bias, so an over-aggressive bias cannot ask for
            % more than the locked axle can supply.
            W = vm.totalMass * vm.g;
            frontNormalLoad = max(W * vm.staticFrontWeight + aeroForces.Fz_front, 0);
            rearNormalLoad = max(W * (1 - vm.staticFrontWeight) + aeroForces.Fz_rear, 0);
            frontMu = max(vm.tire.getPeakFriction(frontNormalLoad / 2), 0);
            rearMu = max(vm.tire.getPeakFriction(rearNormalLoad / 2), 0);
            brakeBiasFront = max(0, min(1, vm.brakeBiasFront));
            brakeBiasRear = 1 - brakeBiasFront;
            brakeGripLimit = inf;
            if brakeBiasFront > eps
                brakeGripLimit = min(brakeGripLimit, ...
                    frontMu * frontNormalLoad / brakeBiasFront);
            end
            if brakeBiasRear > eps
                brakeGripLimit = min(brakeGripLimit, ...
                    rearMu * rearNormalLoad / brakeBiasRear);
            end
            maxBrakeForce = min(max(0, vm.brakeForceCoefficient) * totalNormalLoad, ...
                brakeGripLimit);

            rollingResistance = 0.015 * totalNormalLoad;
            brakeAccel = (maxBrakeForce + F_drag + rollingResistance) / vm.totalMass;

            % Forward drive: powertrain map capped by driven (rear) axle grip.
            F_drive_full = 0;
            if ~isempty(vm.powertrain)
                F_drive_full = max(0, vm.powertrain.computeMaxDriveForce(tempState.speed));
            end
            F_traction_rear = rearMu * rearNormalLoad;
            F_drive_full = min(F_drive_full, F_traction_rear);

            F_resistance = F_drag + rollingResistance;

            caps.maxLatAccel = max(0.1, tireAccel);
            caps.maxBrakeAccel = max(0.1, min(tireAccel, brakeAccel));
            caps.F_drive_full = F_drive_full;
            caps.F_resistance = F_resistance;
            caps.maxDriveAccel = max(0, (F_drive_full - F_resistance) / vm.totalMass);
            caps.brakeForceAccel = max(0.1, maxBrakeForce / vm.totalMass);
        end

        function y = smoothSignal(x, span, closed)
            % SMOOTHSIGNAL Public moving-average smoothing of a 1-D signal with
            % closed-loop wrapping. Exposed so the driver can smooth the line
            % curvature it consumes (image-derived centerlines are jagged).
            if nargin < 3
                closed = false;
            end
            y = LapPlanner.smoothVector(x, span, closed);
        end
    end

    methods (Static, Access = private)
        function j = neighbor(i, n, closed, dir)
            % NEIGHBOR Index of the waypoint one step (dir = -1 or +1) from i,
            % wrapping on closed loops and clamping at open-track ends.
            j = i + dir;
            if closed
                if j < 1; j = j + n; end
                if j > n; j = j - n; end
            else
                j = max(1, min(n, j));
            end
        end

        function woff = collinearOffset(pp, pn, base, normal)
            % COLLINEAROFFSET Offset of base along `normal` that places it on
            % the segment pp->pn (the zero-local-curvature configuration).
            %
            % The foot of the perpendicular from `base` to the line pp->pn is
            % the point that makes (pp, base+offset, pn) collinear. We then
            % express (foot - base) in the centerline normal direction.
            seg = pn - pp;
            segLen2 = dot(seg, seg);
            if segLen2 <= eps
                woff = 0;
                return;
            end
            foot = pp + seg * dot(base - pp, seg) / segLen2;
            disp = foot - base;             % world-space displacement needed
            % Offset along `normal`. Solve disp = woff*normal via the larger
            % component for numerical stability.
            if abs(normal(1)) >= abs(normal(2))
                woff = disp(1) / normal(1);
            else
                woff = disp(2) / normal(2);
            end
        end

        function w = feasibilityClamp(points, nx, ny, w, closed, kappaMax)
            % FEASIBILITYCLAMP Scale down offsets where the resulting line
            % curvature exceeds kappaMax, so the line is always followable by
            % the steering. Offenders are blended toward the centerline (w->0)
            % by a factor each pass until the local curvature is feasible.
            n = numel(w);
            for pass = 1:25 %#ok<NASGU>
                linePts = points + w .* [nx, ny];
                kappa = abs(components.Track.computeCurvature(linePts, closed));
                worst = max(kappa);
                if worst <= kappaMax
                    return;
                end
                % Shrink offsets at every point whose neighborhood curvature is
                % too high, by 15% per pass. This preserves smoothness while
                % relaxing the tight spots.
                bad = find(kappa > kappaMax);
                for b = bad(:)'
                    lo = max(1, b - 1); hi = min(n, b + 1);
                    if closed
                        lo = LapPlanner.neighborIdx(b, n, -1);
                        hi = LapPlanner.neighborIdx(b, n, +1);
                    end
                    w(lo:hi) = w(lo:hi) * 0.85;
                end
            end
        end

        function j = neighborIdx(i, n, dir)
            % NEIGHBORIDX Static neighbor index (wrap clamped to [1,n] range).
            j = i + dir;
            if j < 1; j = 1; end
            if j > n; j = n; end
        end

        function y = maxAbsVector(x, span, closed)
            % MAXABSVECTOR Sliding max of |x| over a window of +/-span (so a
            % conservative, neighborhood-aware curvature for speed targets).
            % Closed-loop aware (wraps); open-track endpoints clamp.
            n = numel(x);
            x = x(:);
            if n < 2 || span <= 0
                y = abs(x);
                return;
            end
            y = zeros(n, 1);
            for i = 1:n
                lo = i - span; hi = i + span;
                if closed
                    idx = mod((lo:hi) - 1, n) + 1;
                else
                    idx = max(1, lo):min(n, hi);
                end
                y(i) = max(abs(x(idx)));
            end
        end

        function y = smoothVector(x, span, closed)
            % SMOOTHVECTOR Moving-average smoothing of a 1-D signal with
            % closed-loop wrapping (used to robustly detect corner regions).
            n = numel(x);
            if n < 3 || span <= 1
                y = x(:);
                return;
            end
            span = max(1, round(span));
            x = x(:);
            if closed
                pad = min(n - 1, ceil(span / 2));
                xe = [x(end-pad+1:end); x; x(1:pad)];
                y = movmean(xe, span);
                y = y(pad+1:pad+n);
            else
                y = movmean(x, span);
            end
        end

        function w = smoothOffset(w, span, closed)
            % SMOOTHOFFSET Moving-average smoothing of the offset profile.
            n = numel(w);
            if n < 3 || span <= 1
                return;
            end
            span = max(1, round(span));
            if closed
                pad = min(n - 1, ceil(span / 2));
                we = [w(end-pad+1:end); w; w(1:pad)];
                ws = movmean(we, span);
                w = ws(pad+1:pad+n);
            else
                w = movmean(w, span);
            end
        end

        function val = fieldOr(s, name, default)
            % FIELDOR Return s.name if present and nonempty, else default.
            if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
                val = s.(name);
            else
                val = default;
            end
        end
    end
end
