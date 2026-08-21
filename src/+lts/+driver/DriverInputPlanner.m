classdef DriverInputPlanner
    % DRIVERINPUTPLANNER Builds open-loop controls from track reference data.
    %
    % The planner treats the track centerline as a reference for estimating
    % throttle, brake, and steering commands. It does not perform path
    % tracking and does not constrain vehicle motion to the centerline.
    %
    % Physics role: this is a GGV-style speed planner, not a vehicle model.
    % It asks aero/tire/powertrain/brake models for approximate capability,
    % propagates speed limits forward/backward along distance, then converts
    % required longitudinal acceleration into pedal commands.

    properties
        vehicleManager
        driverModel
        maxSteeringAngle = 0.6
        maxDriveAccel = 5.0
        speedFeedbackDeadband = 0.20
        speedFeedbackThrottleBand = 1.0
        speedFeedbackBrakeBand = 1.0
        speedFeedbackCorrectionGain = 0.35
        racingLineEnabled = true
        % Fraction of the per-waypoint drivable corridor (minus the CG
        % safety margin) that the minimum-curvature optimizer may use.
        % 1.0 = use the full corridor; smaller values optimize within an
        % inset band (conservative line). HierarchicalOptimizer sweeps this.
        racingLineOffsetFraction = 0.65
        racingLineCurvatureSmoothDistance = 6.0
        racingLineOffsetSmoothDistance = 8.0
        vehicleTrackWidth = 0
        edgeSlowdownMargin = 0.75
        cachedRollingResistanceCoeff = 0.015
        cachedCorneringUsage = 0.98
        cachedBrakingUsage = 0.98
        cachedDriveUsage = 0.98
        cachedTrailBrakeReserve = 0.30
        cachedTractionCircleReserve = 0
        cachedCorneringGripMargin = 0.95
        % When the feedforward plan calls for coast (both pedals zero) and the
        % speed error stays within this tolerance [m/s], the closed-loop layer
        % holds coast instead of pedaling to correct the small drift. This lets
        % visible lift-off coasting plateaus survive at corner entries/exits.
        % Only applies when the plan is actually slowing (axRef below
        % coastAxRefThreshold), not when it is holding speed against drag.
        coastSpeedTolerance = 0.75
        coastAxRefThreshold = 0.05  % |axRef| above which a coast plan counts as slowing
    end

    methods
        function obj = DriverInputPlanner(vehicleManager, driverModelOrMaxSteer)
            obj.vehicleManager = vehicleManager;
            if nargin >= 2
                if isnumeric(driverModelOrMaxSteer)
                    obj.maxSteeringAngle = driverModelOrMaxSteer;
                    obj.driverModel = [];
                else
                    obj.driverModel = driverModelOrMaxSteer;
                    if isprop(driverModelOrMaxSteer, 'maxSteeringAngle')
                        obj.maxSteeringAngle = driverModelOrMaxSteer.maxSteeringAngle;
                    end
                    if isprop(driverModelOrMaxSteer, 'throttleBand')
                        obj.speedFeedbackDeadband = max( ...
                            obj.speedFeedbackDeadband, ...
                            driverModelOrMaxSteer.throttleBand);
                    end
                    if isprop(driverModelOrMaxSteer, 'brakeBlendSpeed')
                        blendSpeed = driverModelOrMaxSteer.brakeBlendSpeed;
                        if isfinite(blendSpeed) && blendSpeed > 0
                            obj.speedFeedbackBrakeBand = blendSpeed;
                        end
                    end
                end
            else
                obj.driverModel = [];
            end
            obj = obj.cacheRunConstants();
        end

        function obj = cacheRunConstants(obj)
            vm = obj.vehicleManager;
            if isa(vm, 'lts.vehicle.VehicleManager')
                obj.vehicleTrackWidth = vm.trackWidth;
                if isa(vm.tire, 'lts.components.Tire.PacejkaTire')
                    obj.cachedRollingResistanceCoeff = vm.tire.rollingResistanceCoeff;
                elseif ~isempty(vm.tire) && isprop(vm.tire, 'rollingResistanceCoeff')
                    obj.cachedRollingResistanceCoeff = vm.tire.rollingResistanceCoeff;
                end
            elseif isstruct(vm)
                if isfield(vm, 'trackWidth')
                    obj.vehicleTrackWidth = vm.trackWidth;
                end
                if isfield(vm, 'tire')
                    obj.cachedRollingResistanceCoeff = obj.readRollingResistanceCoeff(vm.tire);
                end
            elseif isobject(vm)
                if isprop(vm, 'trackWidth')
                    obj.vehicleTrackWidth = vm.trackWidth;
                end
                if isprop(vm, 'tire')
                    obj.cachedRollingResistanceCoeff = obj.readRollingResistanceCoeff(vm.tire);
                end
            end

            dm = obj.driverModel;
            if ~isempty(dm)
                if isa(dm, 'lts.driver.DriverModel')
                    obj.edgeSlowdownMargin = dm.edgeSlowdownMargin;
                    obj.cachedCorneringUsage = dm.corneringUsage;
                    obj.cachedBrakingUsage = dm.brakingUsage;
                    obj.cachedDriveUsage = dm.driveUsage;
                    obj.cachedTrailBrakeReserve = dm.trailBrakeReserve;
                    obj.cachedTractionCircleReserve = dm.tractionCircleReserve;
                    obj.cachedCorneringGripMargin = dm.corneringGripMargin;
                    obj.racingLineOffsetFraction = dm.racingLineOffsetFraction;
                    obj.racingLineCurvatureSmoothDistance = ...
                        dm.racingLineCurvatureSmoothDistance;
                    obj.racingLineOffsetSmoothDistance = ...
                        dm.racingLineOffsetSmoothDistance;
                else
                    obj.edgeSlowdownMargin = obj.readObjectValue(dm, 'edgeSlowdownMargin', obj.edgeSlowdownMargin);
                    obj.cachedCorneringUsage = obj.readObjectValue(dm, 'corneringUsage', obj.cachedCorneringUsage);
                    obj.cachedBrakingUsage = obj.readObjectValue(dm, 'brakingUsage', obj.cachedBrakingUsage);
                    obj.cachedDriveUsage = obj.readObjectValue(dm, 'driveUsage', obj.cachedDriveUsage);
                    obj.cachedTrailBrakeReserve = obj.readObjectValue(dm, 'trailBrakeReserve', obj.cachedTrailBrakeReserve);
                    obj.cachedTractionCircleReserve = obj.readObjectValue(dm, 'tractionCircleReserve', obj.cachedTractionCircleReserve);
                    obj.cachedCorneringGripMargin = obj.readObjectValue(dm, 'corneringGripMargin', obj.cachedCorneringGripMargin);
                end
            end

            if isempty(obj.cachedRollingResistanceCoeff) || ~isfinite(obj.cachedRollingResistanceCoeff)
                obj.cachedRollingResistanceCoeff = 0.015;
            end
            obj.cachedRollingResistanceCoeff = max(obj.cachedRollingResistanceCoeff, 0);
            obj.edgeSlowdownMargin = max(0, obj.edgeSlowdownMargin);
            obj.cachedCorneringUsage = lts.util.saturate(obj.cachedCorneringUsage);
            obj.cachedBrakingUsage = lts.util.saturate(obj.cachedBrakingUsage);
            obj.cachedDriveUsage = lts.util.saturate(obj.cachedDriveUsage);
            obj.cachedTrailBrakeReserve = lts.util.saturate(obj.cachedTrailBrakeReserve);
            obj.cachedTractionCircleReserve = lts.util.saturate(obj.cachedTractionCircleReserve);
            obj.cachedCorneringGripMargin = lts.util.clamp(obj.cachedCorneringGripMargin, 1e-3, 1);
        end

        function profile = buildOpenLoopProfile(obj, initialState, trackData)
            % BUILDOPENLOOPPROFILE Construct a distance-indexed control plan.
            %
            % The pass order mirrors classic lap-time envelope logic:
            %   lateral limit from curvature -> backward braking sweep ->
            %   forward acceleration sweep -> pedal and steer references.
            % The actual lts.simulation.Simulator later closes the loop with tire forces.
            vm = obj.vehicleManager;
            n = trackData.nPts;
            line = obj.buildRacingLine(trackData);
            curvature = line.planningCurvature(:);
            lineDs = obj.computeLineDs(line.lineS);
            vTarget = vm.maxSpeed * ones(n, 1);

            % Iterating lets speed-dependent aero influence the GGV envelope
            % without turning this into a full trajectory optimization.
            for iter = 1:3
                for i = 1:n
                    if abs(curvature(i)) > 1e-6
                        limits = obj.estimateGGVLimits( ...
                            vTarget(i), initialState, curvature(i));
                        vTarget(i) = min(vm.maxSpeed, ...
                            sqrt(max(limits.maxLatAccel, 0.1) / abs(curvature(i))));
                    else
                        vTarget(i) = vm.maxSpeed;
                    end
                end
            end

            % Backward (braking) sweep: query brake capability at the current
            % planned speed as the envelope is propagated upstream. This keeps
            % decel requests bounded by the car's local capability rather than
            % by a stale pre-sweep speed guess.
            maxBrakeAccel = zeros(n, 1);
            driveAccelCap = zeros(n, 1);
            F_drive_full = zeros(n, 1);
            F_resistance = zeros(n, 1);
            brakeForceAccel = zeros(n, 1);
            driveScale = zeros(n, 1);      % traction-circle throttle cap per point
            brakeScale = zeros(n, 1);      % traction-circle brake cap per point
            for i = n-1:-1:1
                limits = obj.estimateGGVLimits( ...
                    vTarget(i + 1), initialState, curvature(i + 1));
                maxBrakeAccel(i + 1) = limits.maxBrakeAccel;
                ds = max(lineDs(i), 0.001);
                reachableSpeed = sqrt(vTarget(i+1)^2 + 2 * maxBrakeAccel(i+1) * ds);
                vTarget(i) = min(vTarget(i), reachableSpeed);
            end
            maxBrakeAccel(1) = obj.estimateGGVLimits( ...
                vTarget(1), initialState, curvature(1)).maxBrakeAccel;

            % Forward (acceleration) sweep. Drive capability is strongly
            % speed-dependent (traction-limited at low speed, power-limited at
            % high speed), so it is queried at the actual sweep speed
            % speedPlan(i) — not the corner-limited vTarget(i) — so corner-exit
            % acceleration is modeled correctly. The capability returned at
            % each point is reused by the pedal map below for consistency.
            speedPlan = vTarget;
            speedPlan(1) = min(max(initialState.speed, 0), vTarget(1));
            for i = 1:n
                % Pass curvature so drive/brake force is traction-circle-capped:
                % at a corner apex little longitudinal capacity remains, so the
                % plan commands coast/partial instead of saturating throttle.
                limits = obj.estimateGGVLimits(speedPlan(i), initialState, curvature(i));
                driveAccelCap(i) = limits.maxDriveAccel;
                F_drive_full(i) = limits.F_drive_full;
                F_resistance(i) = limits.F_resistance;
                brakeForceAccel(i) = limits.brakeForceAccel;
                driveScale(i) = limits.driveScale;
                brakeScale(i) = limits.brakeScale;
                if i < n
                    ds = max(lineDs(i), 0.001);
                    axCap = max(driveAccelCap(i), 0);
                    reachableSpeed = sqrt(speedPlan(i)^2 + 2 * axCap * ds);
                    speedPlan(i+1) = min(vTarget(i+1), reachableSpeed);
                end
            end

            axRef = zeros(n, 1);
            for i = 1:n-1
                ds = max(lineDs(i), 0.001);
                axRaw = (speedPlan(i+1)^2 - speedPlan(i)^2) / (2 * ds);
                axRef(i) = max(-maxBrakeAccel(i), ...
                    min(driveAccelCap(i), axRaw));
            end
            axRef(n) = axRef(max(n-1, 1));

            maxSteer = obj.maxSteeringAngle;
            steerRef = atan(vm.wheelbase * line.lineCurvature(:));
            steerRef = lts.util.clamp(steerRef, -maxSteer, maxSteer);

            % Physics-based pedal map: each planned accel maps to partial
            % throttle, coast, or gradual brake based on the actual force
            % balance, instead of saturating to {0, WOT, full-brake}.
            brakeRef = zeros(n, 1);
            throttleRef = zeros(n, 1);
            for i = 1:n
                [throttleRef(i), brakeRef(i)] = lts.driver.DriverInputPlanner.computePedals( ...
                    axRef(i), F_drive_full(i), F_resistance(i), ...
                    vm.totalMass, brakeForceAccel(i), vm.powertrain);
                % Traction-circle cap: at a corner apex the lateral grip demand
                % leaves little longitudinal capacity, so cap the throttle
                % toward driveScale (0 -> pure coast) and the brake toward
                % brakeScale (trail-brake taper). This makes the planned pedals
                % apex-aware: lift into the apex, coast through it, drive out.
                drivePedalCap = lts.driver.DriverInputPlanner.pedalForTorqueFraction( ...
                    vm.powertrain, driveScale(i));
                throttleRef(i) = min(throttleRef(i), drivePedalCap);
                brakeRef(i) = min(brakeRef(i), brakeScale(i));
            end

            profile = struct( ...
                's', trackData.arcLen, ...
                'vTarget', speedPlan, ...
                'vLimit', vTarget, ...
                'axRef', axRef, ...
                'throttle', throttleRef, ...
                'brake', brakeRef, ...
                'steer', steerRef, ...
                'targetLateralError', line.targetLateralError, ...
                'lineHeading', line.lineHeading, ...
                'lineCurvature', line.lineCurvature, ...
                'lineS', line.lineS);
        end

        function input = sample(obj, profile, idx, actualSpeed)
            idx = max(1, min(idx, numel(profile.throttle)));
            input = struct( ...
                'throttle', profile.throttle(idx), ...
                'brake', profile.brake(idx), ...
                'steer', profile.steer(idx), ...
                'targetSpeed', profile.vTarget(idx), ...
                'axRef', profile.axRef(idx), ...
                'targetLateralError', obj.sampleProfileField(profile, 'targetLateralError', idx, 0), ...
                'lineHeading', obj.sampleProfileField(profile, 'lineHeading', idx, NaN), ...
                'lineCurvature', obj.sampleProfileField(profile, 'lineCurvature', idx, NaN), ...
                'lineS', obj.sampleProfileField(profile, 'lineS', idx, NaN), ...
                'speedError', NaN);

            if nargin >= 4 && isfinite(actualSpeed)
                input.speedError = actualSpeed - input.targetSpeed;
                input = obj.applySpeedFeedback(input, actualSpeed);
            end
        end

        function input = sampleAtProgress(obj, profile, s, actualSpeed)
            sProfile = profile.s(:);
            s = max(sProfile(1), min(sProfile(end), s));
            [idx0, idx1, frac] = obj.profileInterpolationBracket(sProfile, s);

            input = struct( ...
                'throttle', obj.interpProfileVector(profile.throttle, idx0, idx1, frac, 0), ...
                'brake', obj.interpProfileVector(profile.brake, idx0, idx1, frac, 0), ...
                'steer', obj.interpProfileVector(profile.steer, idx0, idx1, frac, 0), ...
                'targetSpeed', obj.interpProfileVector(profile.vTarget, idx0, idx1, frac, NaN), ...
                'axRef', obj.interpProfileVector(profile.axRef, idx0, idx1, frac, NaN), ...
                'targetLateralError', obj.interpProfileField(profile, 'targetLateralError', idx0, idx1, frac, 0), ...
                'lineHeading', obj.interpProfileField(profile, 'lineHeading', idx0, idx1, frac, NaN), ...
                'lineCurvature', obj.interpProfileField(profile, 'lineCurvature', idx0, idx1, frac, NaN), ...
                'lineS', obj.interpProfileField(profile, 'lineS', idx0, idx1, frac, NaN), ...
                'speedError', NaN);

            input.throttle = lts.util.saturate(input.throttle);
            input.brake = lts.util.saturate(input.brake);

            if nargin >= 4 && isfinite(actualSpeed)
                input.speedError = actualSpeed - input.targetSpeed;
                input = obj.applySpeedFeedback(input, actualSpeed);
            end
        end
    end

    methods (Access = private)
        function input = applySpeedFeedback(obj, input, actualSpeed)
            if ~isfield(input, 'targetSpeed') || ~isfinite(input.targetSpeed)
                return;
            end
            if ~isfield(input, 'axRef') || ~isfinite(input.axRef)
                input.axRef = 0;
            end

            speedError = actualSpeed - input.targetSpeed;
            deadband = max(0, obj.speedFeedbackDeadband);
            plannedCoast = input.throttle <= 0 && input.brake <= 0;
            % Coast-aware hold: when the feedforward plan calls for coast AND
            % the planned axRef indicates the car is meant to be slowing (drag
            % doing the work), tolerate small speed drift without pedaling so
            % lift-off coast plateaus survive. When axRef is near zero (a
            % maintain-speed coast, e.g. holding speed against drag on a
            % straight), the feedback still trims throttle to hold target speed.
            % Apex coasts come from the planner's traction-circle cap, which
            % drives the planned pedals to zero independently of this hold.
            planningToSlow = input.axRef < -obj.coastAxRefThreshold;
            if plannedCoast && planningToSlow && actualSpeed >= 0.5 && ...
                    abs(speedError) <= obj.coastSpeedTolerance
                input.throttle = 0;
                input.brake = 0;
            elseif actualSpeed < 0.5 && speedError <= 0
                input.brake = 0;
                input.throttle = 1;
            elseif speedError < -deadband
                input.brake = 0;
                throttleCorrection = (-speedError - deadband) / ...
                    max(obj.speedFeedbackThrottleBand, eps);
                throttleCorrection = min(1, ...
                    obj.speedFeedbackCorrectionGain * throttleCorrection);
                input.throttle = max(input.throttle, throttleCorrection);
            elseif speedError > deadband
                input.throttle = 0;
                brakeCorrection = (speedError - deadband) / ...
                    max(obj.speedFeedbackBrakeBand, eps);
                brakeCorrection = min(1, ...
                    obj.speedFeedbackCorrectionGain * brakeCorrection);
                input.brake = max(input.brake, brakeCorrection);
            else
                if input.axRef >= 0
                    input.brake = 0;
                end
                if input.axRef <= 0
                    input.throttle = 0;
                end
            end

            input.throttle = lts.util.saturate(input.throttle);
            input.brake = lts.util.saturate(input.brake);
        end

        function line = buildRacingLine(obj, trackData)
            % BUILDRACINGLINE Minimum-curvature racing line within the corridor.
            %
            % The line is parametrized by a lateral offset n_i at each
            % centerline waypoint (positive = left, negative = right). The
            % offsets are chosen to minimize the total squared curvature of
            % the resulting polyline, subject to per-waypoint box constraints
            % n_i in [-rightLimit, +leftLimit]. This is the classic
            % minimum-curvature racing line: it cuts corners, runs out to the
            % full track width wherever that lowers curvature, and yields a
            % smooth path with few direction changes -- as opposed to a
            % centerline-relative apex heuristic.
            %
            % Contract of the returned struct (all Nx1, N = trackData.nPts):
            %   targetLateralError - normal offset [m], +left/-right
            %   lineS               - arc length ALONG the racing line
            %   lineHeading         - racing-line tangent heading [rad]
            %   lineCurvature       - racing-line curvature [1/m], smoothed
            %   planningCurvature   - curvature used by the GGV speed sweep
            n = trackData.nPts;
            centerS = trackData.arcLen(:);
            centerPoints = trackData.points;
            centerHeading = trackData.heading(:);
            centerCurvature = trackData.curvature(:);

            line.targetLateralError = zeros(n, 1);
            line.lineS = centerS;
            line.lineHeading = centerHeading;
            line.lineCurvature = centerCurvature;
            line.planningCurvature = obj.smoothByDistance( ...
                centerCurvature, centerS, obj.racingLineCurvatureSmoothDistance);

            if ~obj.racingLineEnabled || n < 3 || obj.isSteadyCircle(trackData)
                return;
            end
            if ~isfield(trackData, 'trackHalfWidth') || ...
                    ~isfinite(trackData.trackHalfWidth) || trackData.trackHalfWidth <= 0
                return;
            end

            % Per-waypoint, per-side offset limits. A positive targetOffset
            % moves the line left of the centerline (bounded by the left
            % half-width), a negative one moves it right. racingLineOffsetFraction
            % scales how much of the available corridor the optimizer may use.
            [leftLimit, rightLimit] = obj.computeRacingLineOffsetLimits(trackData);
            if isempty(leftLimit) || (max(leftLimit) <= 0 && max(rightLimit) <= 0)
                return;
            end

            closedLoop = isfield(trackData, 'closedLoop') && ...
                ~isempty(trackData.closedLoop) && logical(trackData.closedLoop);

            % Solve the minimum-curvature problem over the offset vector.
            targetOffset = obj.solveMinCurvatureOffsets( ...
                centerPoints, centerHeading, leftLimit, rightLimit, closedLoop);

            % Light post-polish: smooth the offsets so the discrete Menger
            % curvature fed downstream is not noisy at the corridor walls.
            targetOffset = obj.smoothByDistance( ...
                targetOffset, centerS, obj.racingLineOffsetSmoothDistance);
            % Asymmetric clamp: positive offsets bounded by the local left
            % limit, negative by the local right limit.
            targetOffset = min(targetOffset, leftLimit);
            targetOffset = max(targetOffset, -rightLimit);

            heading = unwrap(centerHeading);
            normalX = -sin(heading);
            normalY = cos(heading);
            linePoints = centerPoints + [targetOffset .* normalX, targetOffset .* normalY];
            lineDs = hypot(diff(linePoints(:,1)), diff(linePoints(:,2)));
            lineS = [0; cumsum(lineDs)];
            if numel(lineS) ~= n || lineS(end) <= eps
                return;
            end

            % computeHeading/computeCurvature are called with closed=false so
            % cleanPoints does not drop the closure point and the result stays
            % length-N (matching the centerline/trackData indexing). The line
            % is a closed loop in geometry but indexed as an open polyline here.
            lineHeading = lts.components.Track.computeHeading(linePoints, false);
            lineCurvature = lts.components.Track.computeCurvature(linePoints, false);
            lineCurvature = obj.smoothByDistance( ...
                lineCurvature, centerS, obj.racingLineCurvatureSmoothDistance);
            if numel(lineHeading) ~= n || numel(lineCurvature) ~= n
                return;
            end
            line.targetLateralError = targetOffset;
            line.lineS = lineS;
            line.lineHeading = lineHeading(:);
            line.lineCurvature = lineCurvature(:);
            line.planningCurvature = line.lineCurvature;
        end

        function [leftLimit, rightLimit] = computeRacingLineOffsetLimits(obj, trackData)
            % COMPUTERACINGLINEOFFSETLIMITS Per-waypoint left/right racing-line
            % offset limits [m]. Returns two Nx1 column vectors. Positive
            % offsets (left of centerline) are bounded by leftLimit; negative
            % (right) by rightLimit. When the track carries a per-waypoint
            % corridor (trackLeftHalfWidth/trackRightHalfWidth) the limits
            % follow it; otherwise both reduce to the symmetric scalar
            % half-width and the result matches the legacy single-limit path.
            % racingLineOffsetFraction scales how much of the corridor is
            % available to the optimizer (1.0 = full corridor minus CG margin).
            n = size(trackData.points, 1);
            if isfield(trackData, 'trackLeftHalfWidth') && ...
                    isfield(trackData, 'trackRightHalfWidth') && ...
                    ~isempty(trackData.trackLeftHalfWidth)
                leftHalf = trackData.trackLeftHalfWidth(:);
                rightHalf = trackData.trackRightHalfWidth(:);
            else
                leftHalf = repmat(trackData.trackHalfWidth, n, 1);
                rightHalf = leftHalf;
            end
            cgMargin = max(0.5 * obj.vehicleTrackWidth + 0.25, ...
                obj.edgeSlowdownMargin);
            leftLimit = max(0, obj.racingLineOffsetFraction * (leftHalf - cgMargin));
            rightLimit = max(0, obj.racingLineOffsetFraction * (rightHalf - cgMargin));
        end

        function offset = solveMinCurvatureOffsets(obj, points, heading, ...
                leftLimit, rightLimit, closed)
            % SOLVEMINCURVATUREOFFSETS Minimum-curvature lateral offsets.
            %
            % Minimizes J(n) = sum_i kappa_i(n)^2 where kappa_i is the Menger
            % curvature of the offset polyline, subject to per-waypoint box
            % constraints n_i in [-rightLimit_i, +leftLimit_i]. Each kappa_i
            % depends only on the three neighboring offsets (n_{i-1}, n_i,
            % n_{i+1}), so each offset influences at most three curvature
            % samples and the full gradient is computed in O(N).
            %
            % Strategy: projected gradient descent with an adaptive step.
            % Warm-started from the convex smoothness surrogate (which is
            % always finite, smooth, and corridor-respecting). Box projection
            % keeps every iterate feasible, so the line always respects track
            % limits regardless of convergence. Extended deviations from the
            % centerline are encouraged wherever they reduce total curvature.
            n = size(points, 1);
            offset = zeros(n, 1);
            if n < 3
                return;
            end

            lower = -rightLimit(:);
            upper = leftLimit(:);

            h = unwrap(heading(:));
            normalX = -sin(h);
            normalY = cos(h);

            kappaOf = @(o) obj.offsetPolylineCurvature( ...
                points, normalX, normalY, o, closed);
            costOf = @(o) sum(kappaOf(o).^2);

            % Warm start from the convex smoothness surrogate: it is cheap,
            % strictly convex, and always feasible. The Gauss-Newton curvature
            % refinement below then drives the offsets out to the corridor
            % walls wherever that reduces total curvature.
            offset = obj.solveConvexSmoothness(lower, upper, closed);
            bestOffset = offset;
            bestCost = costOf(offset);

            % Levenberg-Marquardt (damped Gauss-Newton) on J(n) = sum kappa^2.
            % Each step solves (J'*J + lambda*I) dn = -J'*kappa, then a
            % backtracking line search accepts the first scaled step that
            % reduces the cost (after box projection). LM damping keeps the
            % step well-conditioned on tight corners; box projection keeps
            % every iterate inside the corridor, so the line always respects
            % track limits regardless of convergence.
            lambda = 1e-2;
            tol = 1e-8;
            maxIter = 50;
            for iter = 1:maxIter
                [J, kappa] = obj.curvatureJacobian( ...
                    points, normalX, normalY, offset, closed);
                g = J' * kappa;                   % gradient of (1/2) sum kappa^2
                if ~all(isfinite(g)) || norm(g) < tol
                    break;
                end

                A = (J' * J) + lambda * speye(n);
                dn = A \ (-g);                    % Gauss-Newton direction
                if ~all(isfinite(dn))
                    lambda = lambda * 10;
                    if lambda > 1e10
                        break;
                    end
                    continue;
                end

                % Backtracking line search along dn.
                curCost = costOf(offset);
                accepted = false;
                alpha = 1.0;
                for ls = 1:20
                    trial = offset + alpha * dn;
                    trial = max(lower, min(upper, trial));
                    trialCost = costOf(trial);
                    if isfinite(trialCost) && trialCost < curCost - 1e-12
                        stepNorm = norm(trial - offset);
                        offset = trial;
                        if trialCost < bestCost
                            bestCost = trialCost;
                            bestOffset = trial;
                        end
                        accepted = true;
                        break;
                    end
                    alpha = alpha * 0.5;
                end

                if accepted
                    lambda = max(lambda * 0.3, 1e-8);
                    if exist('stepNorm', 'var') && stepNorm < tol * (1 + norm(offset))
                        break;
                    end
                else
                    lambda = lambda * 4;
                    if lambda > 1e10
                        break;
                    end
                end
            end

            offset = bestOffset;
            % Guarantee feasibility (defends against any non-finite slip).
            offset(~isfinite(offset)) = 0;
            offset = max(lower, min(upper, offset));
        end

        function [J, kappa] = curvatureJacobian(obj, points, normalX, normalY, offset, closed)
            % Sparse finite-difference curvature Jacobian.
            n = numel(offset);
            J = spalloc(n, n, 3 * n);
            kappa = obj.offsetPolylineCurvature( ...
                points, normalX, normalY, offset, closed);
            if n < 3
                return;
            end

            ds = hypot(diff(points(:, 1)), diff(points(:, 2)));
            ds = ds(isfinite(ds) & ds > 0);
            epsStep = max(median(ds), 1e-2) * 1e-3;

            for j = 1:n
                op = offset; op(j) = op(j) + epsStep;
                om = offset; om(j) = om(j) - epsStep;
                kp = obj.offsetPolylineCurvature(points, normalX, normalY, op, closed);
                km = obj.offsetPolylineCurvature(points, normalX, normalY, om, closed);
                dk = (kp - km) / (2 * epsStep);
                rows = obj.neighborRows(j, n, closed);
                rows = rows(isfinite(dk(rows)));
                J(rows, j) = dk(rows); %#ok<SPRIX>
            end
        end

        function kappa = offsetPolylineCurvature(~, points, normalX, normalY, offset, closed)
            % OFFSETPOLYLINECURVATURE Menger curvature of the offset polyline.
            %   kappa_i = 2 * signed_area / (a*b*c)
            % using the three neighboring offset points. Matches the sign
            % convention of lts.components.Track.computeCurvature (positive =
            % left turn). Vectorized; periodic wrapping for closed loops.
            P = points + [offset .* normalX, offset .* normalY];
            n = size(P, 1);
            kappa = zeros(n, 1);
            if n < 3
                return;
            end
            if closed
                im = mod((1:n) - 2, n) + 1;
                ip = mod((1:n), n) + 1;
            else
                im = max((1:n) - 1, 1);
                ip = min((1:n) + 1, n);
            end
            aVec = P - P(im, :);
            bVec = P(ip, :) - P;
            cVec = P(ip, :) - P(im, :);
            a = hypot(aVec(:, 1), aVec(:, 2));
            b = hypot(bVec(:, 1), bVec(:, 2));
            c = hypot(cVec(:, 1), cVec(:, 2));
            denom = a .* b .* c;
            area2 = aVec(:, 1) .* bVec(:, 2) - aVec(:, 2) .* bVec(:, 1);
            kappa = 2 .* area2 ./ denom;
            kappa(~isfinite(kappa)) = 0;
            if ~closed
                kappa(1) = kappa(2);
                kappa(end) = kappa(end - 1);
            end
        end

        function rows = neighborRows(~, j, n, closed)
            % NEIGHBORROWS Row indices whose curvature depends on offset j.
            % Curvature at row i depends on offsets (i-1, i, i+1), so offset j
            % influences rows (j-1, j, j+1), with periodic wrapping for closed
            % loops and clamping at open-track endpoints.
            if closed
                rows = [mod(j - 2, n) + 1, j, mod(j, n) + 1];
            else
                rows = [j - 1, j, j + 1];
                rows = rows(rows >= 1 & rows <= n);
            end
        end

        function offset = solveConvexSmoothness(~, lower, upper, closed)
            % SOLVECONVEXSMOOTHNESS Strictly-convex minimum-second-difference
            % surrogate. Minimizes sum (n_{i-1} - 2 n_i + n_{i+1})^2 subject
            % to the box constraints, via projected gradient descent. Used as
            % the warm start and safety fallback: it is always finite, smooth,
            % and corridor-respecting. For a near-straight centerline this is
            % already close to optimal; for corners it is a sensible seed that
            % the curvature refinement improves.
            n = numel(lower);
            offset = zeros(n, 1);
            if n < 3
                return;
            end

            % Second-difference operator D (path-graph Laplacian), periodic
            % for closed loops. A = D'*D is PSD; we minimize 0.5 n' A n.
            e = ones(n, 1);
            D = spdiags(e * [1 -2 1], -1:1, n, n);
            if closed
                D(1, n) = 1;
                D(n, 1) = 1;
            end
            A = D' * D;
            % Pin the constant-offset null space to zero so the minimizer
            % stays near the centerline (otherwise the surrogate is indifferent
            % to a constant shift). Tiny ridge keeps the solve well-conditioned.
            A = A + spdiags(1e-3 * e, 0, n, n);

            % ||A||_2 <= 16 for the second-difference operator, so alpha=1/16
            % guarantees descent for the unconstrained problem; projection
            % only improves feasibility.
            alpha = 1 / 16;
            offset = zeros(n, 1);
            for iter = 1:200
                grad = A * offset;
                newOffset = offset - alpha * grad;
                newOffset = max(lower, min(upper, newOffset));
                if max(abs(newOffset - offset)) < 1e-8
                    offset = newOffset;
                    break;
                end
                offset = newOffset;
            end
            offset(~isfinite(offset)) = 0;
            offset = max(lower, min(upper, offset));
        end

        function values = smoothByDistance(~, values, s, smoothDistance)
            values = values(:);
            if numel(values) < 3 || smoothDistance <= 0 || ~isfinite(smoothDistance)
                return;
            end

            ds = diff(s(:));
            ds = ds(isfinite(ds) & ds > eps);
            if isempty(ds)
                return;
            end
            window = max(1, round(smoothDistance / median(ds)));
            if window <= 1
                return;
            end
            values = movmean(values, window, 'Endpoints', 'shrink');
        end

        function tf = isSteadyCircle(~, trackData)
            tf = false;
            if ~isfield(trackData, 'closedLoop') || ~trackData.closedLoop || ...
                    ~isfield(trackData, 'curvature')
                return;
            end
            curvature = trackData.curvature(:);
            active = abs(curvature) > 1e-6;
            if ~all(active)
                return;
            end
            firstSign = sign(curvature(find(active, 1, 'first')));
            tf = firstSign ~= 0 && all(sign(curvature(active)) == firstSign);
        end

        function lineDs = computeLineDs(~, lineS)
            lineS = lineS(:);
            if numel(lineS) < 2
                lineDs = 0;
                return;
            end
            lineDs = diff(lineS);
            lineDs(~isfinite(lineDs) | lineDs <= 0) = 0.001;
        end

        function value = sampleProfileField(~, profile, fieldName, idx, defaultValue)
            value = defaultValue;
            if isfield(profile, fieldName)
                values = profile.(fieldName);
                if numel(values) >= idx
                    value = values(idx);
                end
            end
        end

        function value = interpProfileField(obj, profile, fieldName, idx0, idx1, frac, defaultValue)
            value = defaultValue;
            if isfield(profile, fieldName)
                values = profile.(fieldName);
                value = obj.interpProfileVector(values, idx0, idx1, frac, defaultValue);
            end
        end

        function [idx0, idx1, frac] = profileInterpolationBracket(~, sProfile, s)
            n = numel(sProfile);
            if n <= 1
                idx0 = 1;
                idx1 = 1;
                frac = 0;
                return;
            end

            idx1 = find(sProfile >= s, 1, 'first');
            if isempty(idx1)
                idx1 = n;
            end
            if idx1 <= 1
                idx0 = 1;
                idx1 = 1;
                frac = 0;
                return;
            end

            idx0 = idx1 - 1;
            ds = sProfile(idx1) - sProfile(idx0);
            if ds <= eps || ~isfinite(ds)
                frac = 0;
            else
                frac = (s - sProfile(idx0)) / ds;
                frac = lts.util.saturate(frac);
            end
        end

        function value = interpProfileVector(~, values, idx0, idx1, frac, defaultValue)
            value = defaultValue;
            if isempty(values)
                return;
            end
            values = values(:);
            if numel(values) < max(idx0, idx1)
                return;
            end
            if idx0 == idx1
                value = values(idx0);
            else
                value = values(idx0) + frac * (values(idx1) - values(idx0));
            end
        end

        function crr = getRollingResistanceCoeff(obj)
            crr = obj.cachedRollingResistanceCoeff;
        end

        function crr = readRollingResistanceCoeff(~, tire)
            crr = 0.015;
            if isstruct(tire) && isfield(tire, 'rollingResistanceCoeff')
                crr = tire.rollingResistanceCoeff;
            elseif isobject(tire) && isprop(tire, 'rollingResistanceCoeff')
                crr = tire.rollingResistanceCoeff;
            end
        end

        function value = readObjectValue(~, source, fieldName, defaultValue)
            value = defaultValue;
            if isstruct(source) && isfield(source, fieldName)
                value = source.(fieldName);
            elseif isobject(source) && isprop(source, fieldName)
                value = source.(fieldName);
            end
        end

        function limits = estimateGGVLimits(obj, speed, templateState, curvature)
            % ESTIMATEGGVLIMITS Vehicle longitudinal/lateral capability at a speed.
            %   When curvature [1/m] is supplied, drive and brake force are
            %   further capped by the traction circle so a corner apex (high
            %   lateral grip demand) leaves little longitudinal capacity ->
            %   the plan commands coast/partial there, not full throttle.
            if nargin < 4 || isempty(curvature) || ~isfinite(curvature)
                curvature = 0;
            end
            vm = obj.vehicleManager;
            tempState = templateState;
            tempState.vehicleManager = vm;
            tempState.speed = max(speed, 0);
            tempState.vx = tempState.speed;
            tempState.vy = 0;

            % Use the same component models as the dynamic simulation for the
            % static capability estimate. This keeps driver targets consistent
            % with aero downforce, powertrain force, tire load sensitivity, and
            % rolling resistance used in lts.simulation.Simulator.step().
            aeroForces = vm.aero.computeForces(tempState);
            F_drag = max(0, aeroForces.F_drag);
            totalNormalLoad = vm.totalMass * vm.g + aeroForces.Fz_front + aeroForces.Fz_rear;
            peakMu = vm.tire.getPeakFriction(totalNormalLoad / 4);
            % Grip comes entirely from the tire model (no surface mu cap);
            % the vehicles run on dry rubber with no friction variability.
            tireAccel = max(peakMu, 0) * totalNormalLoad / vm.totalMass;

            brakeForce = max(0, vm.brakeForceCoefficient) * totalNormalLoad;
            rollingResistance = obj.getRollingResistanceCoeff() * totalNormalLoad;
            brakeAccel = (brakeForce + F_drag + rollingResistance) / vm.totalMass;

            corneringUsage = obj.cachedCorneringUsage;
            brakingUsage = obj.cachedBrakingUsage;
            driveUsage = obj.cachedDriveUsage;
            trailBrakeReserve = obj.cachedTrailBrakeReserve;
            tractionReserve = obj.cachedTractionCircleReserve;
            corneringGripMargin = obj.cachedCorneringGripMargin;

            % --- Traction-circle longitudinal cap from lateral demand ---
            % At a corner the tires spend grip on ay = v^2 * |kappa|; only the
            % remainder is available longitudinally. driveScale collapses to
            % tractionReserve (0 -> coast) at peak lateral demand; brakeScale
            % trails to trailBrakeReserve (gentler, for trail-braking).
            ay = tempState.speed^2 * abs(curvature);
            ayMax = max(corneringUsage * tireAccel, 0.1);
            margin = corneringGripMargin;
            latUse = min(ay / ayMax / margin, 1);
            ellipse = sqrt(max(0, 1 - latUse^2));
            driveScale = lts.util.saturate(tractionReserve) + (1 - lts.util.saturate(tractionReserve)) * ellipse;
            brakeScale = lts.util.saturate(trailBrakeReserve) + (1 - lts.util.saturate(trailBrakeReserve)) * ellipse;

            % --- Forward drive capability (speed- and load-dependent) ---
            % Full-throttle wheel force from the powertrain map, capped by the
            % driven (rear) axle's traction limit. Net of drag + rolling
            % resistance so it is the achievable forward accel, not gross.
            F_drive_full = 0;
            if ~isempty(vm.powertrain)
                F_drive_full = max(0, vm.powertrain.computeMaxDriveForce(tempState.speed));
            end
            W = vm.totalMass * vm.g;
            rearNormalLoad = max(W * (1 - vm.staticFrontWeight) + aeroForces.Fz_rear, 0);
            rearMu = max(vm.tire.getPeakFriction(rearNormalLoad / 2), 0);
            F_traction_rear = driveUsage * rearMu * rearNormalLoad;
            F_drive_full = min(F_drive_full, F_traction_rear);

            F_resistance = F_drag + rollingResistance;
            availableDriveAccel = max(0, ...
                (F_drive_full - F_resistance) / vm.totalMass);
            % Lift-off deceleration comes from drag and rolling resistance.
            coastDecel = F_resistance / vm.totalMass;
            brakeForceAccel = max(0.1, brakingUsage * brakeForce / vm.totalMass);

            limits.maxLatAccel = max(0.1, corneringUsage * tireAccel);
            limits.maxBrakeAccel = max(0.1, brakingUsage * min(tireAccel, brakeAccel));
            limits.F_drive_full = F_drive_full;
            limits.F_resistance = F_resistance;
            limits.maxDriveAccel = availableDriveAccel;
            limits.coastDecel = coastDecel;
            limits.brakeForceAccel = brakeForceAccel;
            limits.driveScale = driveScale;
            limits.brakeScale = brakeScale;
        end
    end

    methods (Static)
        function [throttle, brake] = computePedals(axRef, F_drive_full, ...
                F_resistance, mass, brakeForceAccel, powertrain, driveForceScale)
            % Map requested acceleration to mutually exclusive pedal commands.
            mass = max(mass, eps);
            brakeForceAccel = max(brakeForceAccel, eps);
            if nargin < 6
                powertrain = [];
            end
            if nargin < 7 || isempty(driveForceScale) || ...
                    ~isscalar(driveForceScale) || ~isfinite(driveForceScale)
                driveForceScale = 1;
            end
            driveForceScale = lts.util.saturate(driveForceScale);

            F_req = axRef * mass + F_resistance;
            coastFraction = 0.03;

            throttle = 0;
            brake = 0;
            if F_req <= 0
                excessDecel = max(0, -axRef) - F_resistance / mass;
                if excessDecel >= coastFraction * brakeForceAccel
                    brake = lts.util.saturate(excessDecel / brakeForceAccel);
                end
            elseif F_drive_full <= 0
                if driveForceScale >= coastFraction
                    throttle = lts.driver.DriverInputPlanner.pedalForTorqueFraction( ...
                        powertrain, driveForceScale);
                end
            else
                torqueFraction = lts.util.saturate( ...
                    F_req / F_drive_full) * driveForceScale;
                if torqueFraction >= coastFraction
                    throttle = lts.driver.DriverInputPlanner.pedalForTorqueFraction( ...
                        powertrain, torqueFraction);
                end
            end
        end

        function pedal = pedalForTorqueFraction(powertrain, fraction)
            % PEDALFORTORQUEFRACTION Invert a nonlinear powertrain pedal map.
            % Components that do not expose the optional inverse API retain
            % the legacy linear behavior.
            fraction = lts.util.saturate(fraction);
            pedal = fraction;
            if isempty(powertrain) || ~isobject(powertrain) || ...
                    ~ismethod(powertrain, 'pedalForTorqueFraction')
                return;
            end
            try
                candidate = powertrain.pedalForTorqueFraction(fraction);
                if isnumeric(candidate) && isscalar(candidate) && isfinite(candidate)
                    pedal = lts.util.saturate(candidate);
                end
            catch err
                % A third-party/legacy implementation must not make the
                % planner unusable; fall back to a linear command. Warn once
                % so the misconfiguration is not entirely invisible.
                warning('lts_driver_DriverInputPlanner:PedalMapFailed', ...
                    'powertrain.pedalForTorqueFraction failed (%s); using linear pedal fallback.', ...
                    err.identifier);
                pedal = fraction;
            end
        end
    end
end
