classdef DriverModel < handle
    % DRIVERMODEL A preview-follow racing driver for FSAE-style cars.
    %
    % Drives the car the way a real driver does: it commits to a racing line,
    % holds a consistent sub-limit grip margin, looks ahead through corners,
    % trail-brakes in and squeezes the throttle out, and—critically—reacts to
    % understeer/oversteer. The goal is not theoretical-optimal lap time but
    % repeatable, correlatable behavior so setup changes produce realistic
    % deltas in line, speed, and pedal use.
    %
    % Architecture (all relative to a pre-built racing line):
    %
    %   prepareForSimulation:
    %     LapPlanner.buildRacingLine   -> minimum-curvature line (full width)
    %     LapPlanner.buildVelocityProfile -> skill-governed speed envelope
    %
    %   computeInput (every step):
    %     Steering:  feedforward curvature + look-ahead heading + gentle
    %                cross-track + slip-reactive counter/breathe + edge avoid
    %     Longi:     speed-error feedback on axRef, single traction-circle
    %                taper (lift->coast->squeeze), mapped to pedals via PedalMap
    %                plus drive-slip traction control.
    %
    % Public contract (unchanged from the legacy driver, so Simulator.m and
    % run_simulation.m need no edits):
    %   DriverModel(vehicleManager)
    %   obj = prepareForSimulation(obj, initialState, trackData, dt)
    %   input = computeInput(obj, state, observation)
    % where input is a struct with throttle, brake, steer.

    properties
        % ---- Reference to VehicleManager for component access ----
        vehicleManager

        % ---- Driver skill (the single margin knob) ----
        % Fraction of the grip/drive/brake envelope the driver uses. Lower is
        % safer and more repeatable; higher is quicker but less forgiving. The
        % car runs a consistent fraction below the limit so setup deltas show
        % up as monotonic, correlatable changes.
        driverSkill = 0.90

        % ---- Racing line ----
        lineUsage = 0.78          % Fraction of half-width the line may use

        % ---- Preview steering ----
        % The feedforward curvature term (atan(L*kappa)) is the primary steer
        % command and is physically correct for the line. The look-ahead and
        % cross-track terms are small corrections on top of it, so their gains
        % are kept modest to avoid over-reacting to tight corners (which would
        % saturate steering and spin the car).
        lookaheadTime = 0.4       % Seconds of preview -> lookahead distance
        minLookahead  = 2.0       % Minimum lookahead distance [m]
        kHeading      = 0.35      % Look-ahead heading-error steer gain
        kCross        = 0.5       % Cross-track (to racing line) steer gain
        softening     = 3.0       % Low-speed Stanley softening term [m/s]
        steeringUsage = 1.0       % Fraction of line curvature -> feedforward steer
        previewSteerLimit = 0.15  % Cap on preview/cross-track correction [rad]
        maxSteeringAngle = 0.6    % Road-wheel angle limit [rad]

        % ---- Slip stabilization (the human reaction) ----
        enableSlipStabilizer = true
        kOversteer   = 1.0        % Counter-steer gain when rear slides
        slipTarget   = 0.05       % Target |body slip| kept by counter-steer [rad]
        understeerThrottleCut = 0.5  % Throttle fraction kept when understeering
        understeerYawErr = 0.05   % Yaw-rate deficit [rad/s] that triggers breathe

        % ---- Longitudinal control ----
        launchSpeedThreshold = 3.0  % Below this speed, pin throttle to launch [m/s]
        kSpeed       = 0.5        % Speed-error -> accel feedback gain
        % Traction-circle taper. ay = v^2*|kappa| uses up lateral grip; the
        % remaining longitudinal fraction is the ellipse below. The throttle
        % collapses to tractionReserve (0 -> coast) at peak lateral demand, and
        % the brake trails to trailBrakeReserve (gentle trail-braking).
        tractionReserve   = 0.0   % Min throttle fraction at peak lateral grip
        trailBrakeReserve = 0.30  % Min brake fraction at peak lateral grip
        corneringLiftStart = 0.55 % Lateral-grip fraction below which throttle is unhindered

        % ---- Drive-slip traction control (retained, proven) ----
        enableDriveSlipLimit = true
        driveSlipTarget = 0.12    % Rear slip ratio where throttle limiting starts
        driveSlipCutoff = 0.35    % Rear slip ratio where throttle is fully cut

        % ---- Track-edge safety ----
        edgeMargin    = 0.75      % Start reacting this far from the edge [m]
        edgeSteerGain = 0.30      % Steer-back gain near the edge [rad]
        edgeSlowGain  = 0.6       % Throttle reduction near the edge [0-1]
        edgeBrakeAdd  = 0.2       % Brake added near/outside the edge [0-1]

        % ---- Light actuator smoothing ----
        % Single first-order filter on each pedal for smoothness. The simulator
        % also enforces a steering rate limit, so no steering slew is needed.
        pedalFilterTime = 0.10    % Pedal first-order time constant [s]

        % ---- Cached plan (built in prepareForSimulation) ----
        racingLine  = []          % LapPlanner.buildRacingLine output
        velProfile  = []          % LapPlanner.buildVelocityProfile output

        % ---- Driver actuator state ----
        inputDt = 0.001
        lastThrottle = 0
        lastBrake = 0
        filteredThrottle = 0
        filteredBrake = 0
        inputStateInitialized = false
        stuckTimer = 0   % consecutive near-stationary steps (for spin recovery)
    end

    properties (Access = private)
        % Lookup index of the car's last projected reference point, used to pick
        % the matching racing-line sample without re-searching the polyline.
        lastIndex = 1
    end

    methods
        function obj = DriverModel(vehicleManager)
            % DRIVERMODEL Construct with a VehicleManager reference.
            obj.vehicleManager = vehicleManager;
        end

        function obj = prepareForSimulation(obj, initialState, trackData, dt)
            % PREPAREFORSIMULATION Build the racing line and velocity profile.
            if nargin >= 4 && isfinite(dt) && dt > 0
                obj.inputDt = dt;
            end
            obj.lastThrottle = max(0, min(1, initialState.throttle));
            obj.lastBrake = max(0, min(1, initialState.brake));
            obj.filteredThrottle = obj.lastThrottle;
            obj.filteredBrake = obj.lastBrake;
            obj.inputStateInitialized = true;
            obj.lastIndex = 1;

            lineOpts = struct('lineUsage', obj.lineUsage, ...
                'maxSteeringAngle', obj.maxSteeringAngle, ...
                'wheelbase', obj.vehicleManager.wheelbase);
            obj.racingLine = LapPlanner.buildRacingLine(trackData, lineOpts);

            % Precompute the feedforward steer profile along the racing line.
            % The line curvature is smoothed first: image-derived centerlines
            % are jagged (curvature sign-flips every metre), and feeding that
            % raw into the steer command oscillates the steering and spins the
            % car. A moving-average over a few metres captures real corners
            % (which span many points) while suppressing that noise.
            L = obj.vehicleManager.wheelbase;
            closed = isfield(trackData, 'closedLoop') && logical(trackData.closedLoop);
            kappaSmooth = LapPlanner.smoothSignal(obj.racingLine.curvature, 7, closed);
            obj.racingLine.curvatureSmoothed = kappaSmooth;
            steerFF = atan(L .* kappaSmooth) * obj.steeringUsage;
            obj.racingLine.steerFF = max(-obj.maxSteeringAngle, ...
                min(obj.maxSteeringAngle, steerFF));

            velOpts = struct('driverSkill', obj.driverSkill);
            obj.velProfile = LapPlanner.buildVelocityProfile( ...
                obj.racingLine, obj.vehicleManager, initialState, velOpts);
        end

        function input = computeInput(obj, state, observation)
            % COMPUTEINPUT Return throttle, brake, and steer for this state.
            if nargin < 3 || isempty(observation)
                observation = obj.defaultObservationFromState(state);
            end

            % If no plan was built (e.g. used outside prepareForSimulation),
            % fall back to a safe centerline cruise so the car stays controllable.
            if isempty(obj.racingLine) || isempty(obj.velProfile)
                input = struct('throttle', 0.5, 'brake', 0, 'steer', 0, ...
                    'targetSpeed', NaN, 'axRef', 0);
                return;
            end

            idx = obj.lineIndex(observation);

            steer = obj.computeSteering(idx, state, observation);

            [throttle, brake, axCmd] = obj.computeLongitudinal(idx, state, observation);

            % Slip-reactive longitudinal breathe for understeer (steering is
            % already counter-steered in computeSteering for oversteer).
            [throttle, brake] = obj.applyUndersteerBreathe( ...
                throttle, brake, idx, state);

            % Drive-slip traction control (rear slip ratio).
            throttle = obj.applyDriveSlipLimit(throttle, state);

            % Launch: from a standstill (or near it) while still below target
            % speed, a driver pins the throttle and releases the brake —
            % otherwise edge/understeer brakes can hold the car at a stop and
            % the pedal filter never releases. The steer is also faded toward
            % the planned line steer (not the reactive correction) so the
            % wheels can build forward momentum instead of being pinned at full
            % lock, which would just spin the car in place. Disabled once moving.
            vTarget = obj.velProfile.vTarget(idx);
            if ~isfinite(vTarget)
                vTarget = state.speed;   % NaN/inf guard: don't fight the plan
            end
            % Launch: whenever the car is nearly stopped, pin throttle and
            % release brake so it always makes forward progress (a real driver
            % never stalls on track). This also breaks any low-speed limit
            % cycle. The "below target" check is dropped: from a standstill the
            % driver always wants to get moving.
            if state.speed < obj.launchSpeedThreshold
                throttle = 1;
                brake = 0;
                % At a standstill, cap the steer well below full lock so the
                % wheels can build forward momentum — a stopped car at full lock
                % just pivots. Blend back to the planned line steer as speed
                % rises toward the launch threshold.
                launchFrac = max(0, min(1, state.speed / obj.launchSpeedThreshold));
                plannedSteer = 0;
                if isfield(obj.racingLine, 'steerFF') && ~isempty(obj.racingLine.steerFF)
                    plannedSteer = obj.racingLine.steerFF(idx);
                end
                launchSteerCap = 0.3 * obj.maxSteeringAngle;
                plannedSteer = max(-launchSteerCap, min(launchSteerCap, plannedSteer));
                steer = plannedSteer * (1 - launchFrac) + steer * launchFrac;

                % Spin recovery: if the car has been near-stationary for a
                % sustained period it has lost control (a spin). Straighten the
                % wheels toward the car's actual heading and creep forward with
                % bounded throttle until forward speed returns — this guarantees
                % the simulation always makes progress instead of looping
                % forever at a stall, and mirrors a driver re-gathering the car.
                obj.stuckTimer = obj.stuckTimer + 1;
                if obj.stuckTimer > round(1.5 / obj.inputDt)
                    steer = 0;
                    throttle = 0.5;
                    brake = 0;
                end
            else
                obj.stuckTimer = 0;
            end

            input = struct( ...
                'throttle', throttle, ...
                'brake', brake, ...
                'steer', steer, ...
                'targetSpeed', vTarget, ...
                'axRef', axCmd);

            input = obj.applyPedalFilter(input);
        end
    end

    methods (Access = private)
        % ============================================================
        % Steering: preview-follow + slip reaction
        % ============================================================
        function steer = computeSteering(obj, idx, state, observation)
            line = obj.racingLine;

            % Lookahead distance (speed-scaled, bounded).
            lookahead = state.speed * obj.lookaheadTime + obj.minLookahead;

            % 1) Feedforward: the PRECOMPUTED steer for this station along the
            % racing line (atan(L*kappa_line)), sampled by progress. This is the
            % primary command — non-reactive and free of preview lag, so it
            % follows the planned line through sharp corners reliably.
            if isfield(line, 'steerFF') && ~isempty(line.steerFF)
                steerFF = line.steerFF(idx);
            else
                steerFF = atan(obj.vehicleManager.wheelbase * line.curvature(idx)) ...
                    * obj.steeringUsage;
            end

            % 2) Heading correction: a SMALL term comparing the car's heading to
            % the line tangent ahead. Bounded so it can never dominate the
            % feedforward (which is what actually follows the line).
            targetPoint = obj.linePointAhead(line, idx, lookahead);
            originPoint = line.points(idx, :);
            headingToTarget = atan2( ...
                targetPoint(2) - originPoint(2), targetPoint(1) - originPoint(1));
            headingErr = obj.wrapAngle(headingToTarget - state.yaw);
            steerPreview = max(-obj.previewSteerLimit, min(obj.previewSteerLimit, ...
                obj.kHeading * headingErr));

            % 3) Cross-track to the racing line (Stanley). The simulator's
            % lateralError is + = car left of centerline; the racing line offset
            % is + = line left of centerline; so the error to the line is
            % e_line = lateralError - offsetW. A car left of its line (e>0)
            % needs to steer right: -atan(k*e / v+soft). Bounded likewise.
            offsetW = line.offsetW(idx);
            eLine = obj.crossTrackError(observation) - offsetW;
            steerCross = -atan2(obj.kCross * eLine, ...
                max(state.speed, 0) + obj.softening);
            steerCross = max(-obj.previewSteerLimit, min(obj.previewSteerLimit, steerCross));

            steer = steerFF + steerPreview + steerCross;

            % 4) Slip reaction (human): counter-steer into oversteer.
            if obj.enableSlipStabilizer
                steer = steer + obj.oversteerCorrection(state);
            end

            % 5) Edge avoidance: steer back from the track limit.
            steer = steer + obj.edgeSteerCorrection(observation);

            steer = max(-obj.maxSteeringAngle, min(obj.maxSteeringAngle, steer));
        end

        function corr = oversteerCorrection(obj, state)
            % OVERSTEERCORRECTION Counter-steer into a developing slide.
            % Body slip angle grows when the rear breaks loose. A driver
            % counter-steers (steers toward the slide). Positive body slip =
            % tail out to the right -> add negative (left) steer to chase it.
            corr = 0;
            slip = state.bodySlipAngle;
            if ~isfinite(slip)
                return;
            end
            excess = abs(slip) - obj.slipTarget;
            if excess <= 0
                return;
            end
            % Sign: counter-steer opposite to the slip's sign so the front
            % wheels point toward the direction the car is sliding.
            corr = -sign(slip) * obj.kOversteer * excess;
        end

        function correction = edgeSteerCorrection(obj, observation)
            % EDGESTEERCORRECTION Steer away from the track edge.
            correction = 0;
            halfWidth = obj.fieldOr(observation, 'trackHalfWidth', []);
            latErr = obj.fieldOr(observation, 'lateralError', []);
            if isempty(halfWidth) || ~isfinite(halfWidth) || halfWidth <= 0
                return;
            end
            if isempty(latErr) || ~isfinite(latErr) || abs(latErr) <= eps
                return;
            end
            margin = halfWidth - abs(latErr);
            if margin >= obj.edgeMargin
                return;
            end
            edgeUse = (obj.edgeMargin - margin) / max(obj.edgeMargin, eps);
            edgeUse = max(0, min(1, edgeUse));
            % Positive lateral error (left of center) -> steer right (negative).
            correction = -sign(latErr) * obj.edgeSteerGain * edgeUse;
        end

        function [targetPoint, idxAhead] = linePointAhead(obj, line, idx, lookahead) %#ok<INUSD>
            % LINEPOINTAHEAD Point on the racing line ~lookahead metres ahead.
            arcLen = line.arcLen;
            n = numel(arcLen);
            sTarget = arcLen(idx) + max(lookahead, 0);
            % Allow wrapping past the end on closed loops (returns the last
            % valid point on open tracks, which is fine).
            idxAhead = idx;
            bestDist = inf;
            for k = idx:min(idx + 60, n)
                d = abs(arcLen(k) - sTarget);
                if d < bestDist
                    bestDist = d;
                    idxAhead = k;
                end
            end
            targetPoint = line.points(idxAhead, :);
        end

        % ============================================================
        % Longitudinal: speed feedback + single traction circle + PedalMap
        % ============================================================
        function [throttle, brake, axCmd] = computeLongitudinal(obj, idx, state, observation) %#ok<INUSD>
            % COMPUTELONGITUDINAL Map the speed plan onto pedal commands.
            profile = obj.velProfile;
            vTarget = profile.vTarget(idx);
            axRef = profile.axRef(idx);

            % Speed-error feedback: add proportional accel to close the gap.
            speedErr = vTarget - state.speed;
            axCmd = axRef + obj.kSpeed * speedErr;

            % Single traction circle from lateral demand along the line. This
            % caps the *output* pedals (not the force input) so the PedalMap
            % ratio is not distorted. ay uses the line curvature, not the
            % centerline, so it reflects where the car actually is.
            %
            % The taper is shaped so the driver holds throttle through moderate
            % cornering (a real driver does — power-through mid-corner) and only
            % lifts toward the apex as lateral grip demand approaches the limit.
            % A pure ellipse sqrt(1-latUse^2) is too aggressive (it forces coast
            % everywhere there is any curvature), so the effective lateral use is
            % shifted: nothing happens below corneringLiftStart, then tapers.
            kappaLine = obj.lineCurvature(idx);
            ay = state.speed^2 * abs(kappaLine);
            ayMax = obj.lateralGripLimit(state);
            latUse = min(ay / max(ayMax, 0.1), 1);
            liftStart = obj.corneringLiftStart;   % lateral-use fraction where lift begins
            shapedUse = max(0, (latUse - liftStart) / max(1 - liftStart, eps));
            shapedUse = min(shapedUse, 1);
            ellipse = sqrt(max(0, 1 - shapedUse^2));
            driveScale = obj.tractionReserve + (1 - obj.tractionReserve) * ellipse;
            brakeScale = obj.trailBrakeReserve + (1 - obj.trailBrakeReserve) * ellipse;

            % Map the commanded accel to pedals via the proven physics map.
            F_drive_full = profile.F_drive_full(idx);
            F_resistance = profile.F_resistance(idx);
            brakeForceAccel = profile.brakeForceAccel(idx);
            [throttle, brake] = PedalMap.compute( ...
                axCmd, F_drive_full, F_resistance, ...
                obj.vehicleManager.totalMass, brakeForceAccel);

            % Apply the traction-circle caps: lift to coast at the apex, taper
            % brake for trail-braking.
            throttle = min(throttle, driveScale);
            brake = min(brake, brakeScale);

            % Track-edge slowdown: reduce throttle and add a little brake near
            % the limit so excursions are rare and graceful.
            [throttle, brake] = obj.applyEdgeSlowdown(throttle, brake, observation);

            throttle = max(0, min(1, throttle));
            brake = max(0, min(1, brake));
            [throttle, brake] = obj.enforceExclusivity(throttle, brake);
        end

        function ayMax = lateralGripLimit(obj, state)
            % LATERALGRIPLIMIT Approx max lateral accel at the current speed.
            % Uses the planner's capability estimator (tire mu + downforce).
            caps = LapPlanner.estimateCapability( ...
                obj.vehicleManager, state, state.speed);
            ayMax = caps.maxLatAccel;
        end

        function [throttle, brake] = applyUndersteerBreathe(obj, throttle, brake, idx, state)
            % APPLYUNDERSTEERBREATHE Ease the throttle when understeering.
            % Understeer = yaw rate lagging the kinematic expectation for the
            % commanded path. A driver breathes the throttle (shifts load
            % forward) to recover front grip rather than adding more steer.
            if ~obj.enableSlipStabilizer
                return;
            end
            kappaLine = obj.lineCurvature(idx);
            yawExpected = state.speed * kappaLine;
            yawDeficit = yawExpected - state.yawRate;   % + = understeer (L turn)
            % Only counts for the turn the car is trying to make.
            if sign(yawExpected) ~= sign(yawDeficit) || abs(yawExpected) < 1e-3
                return;
            end
            excess = abs(yawDeficit) - obj.understeerYawErr;
            if excess <= 0
                return;
            end
            cut = min(excess / 0.3, 1);   % full breathe over 0.3 rad/s deficit
            keep = obj.understeerThrottleCut + (1 - obj.understeerThrottleCut) * (1 - cut);
            throttle = throttle * keep;
        end

        function [throttle, brake] = applyEdgeSlowdown(obj, throttle, brake, observation)
            % APPLYEDGESLOWDOWN Lift (reduce throttle) near the track edge.
            %
            % A real driver who drifts toward the edge eases the throttle to
            % bleed speed gently, rather than dabbing the brake — continuous
            % brake-dabbing near the edge creates a limit cycle that bleeds
            % speed forever without settling. Brake is only added if the car is
            % genuinely past the edge (an actual excursion needing recovery).
            halfWidth = obj.fieldOr(observation, 'trackHalfWidth', []);
            latErr = obj.fieldOr(observation, 'lateralError', []);
            if isempty(halfWidth) || ~isfinite(halfWidth) || halfWidth <= 0
                return;
            end
            if isempty(latErr) || ~isfinite(latErr)
                return;
            end
            margin = halfWidth - abs(latErr);
            if margin >= obj.edgeMargin
                return;
            end
            edgeUse = (obj.edgeMargin - margin) / max(obj.edgeMargin, eps);
            edgeUse = max(0, min(1, edgeUse));
            throttle = throttle * (1 - obj.edgeSlowGain * edgeUse);
            % Only brake if actually past the edge (recovery), not just close.
            if margin < 0
                brake = max(brake, obj.edgeBrakeAdd * edgeUse);
            end
        end

        function throttle = applyDriveSlipLimit(obj, throttle, state)
            % APPLYDRIVESLIPLIMIT Traction control: cut throttle on rear slip.
            if ~obj.enableDriveSlipLimit || throttle <= 0
                return;
            end
            maxRearSlip = obj.maxRearDriveSlip(state);
            if ~isfinite(maxRearSlip) || maxRearSlip <= obj.driveSlipTarget
                return;
            end
            slipUse = (maxRearSlip - obj.driveSlipTarget) / ...
                max(obj.driveSlipCutoff - obj.driveSlipTarget, eps);
            slipUse = max(0, min(1, slipUse));
            throttle = throttle * (1 - slipUse);
        end

        function maxSlip = maxRearDriveSlip(obj, state)
            % MAXREARDRIVESLIP Max slip ratio over the driven (rear) axle.
            maxSlip = NaN;
            vm = state.vehicleManager;
            if isempty(vm) && ~isempty(obj.vehicleManager)
                vm = obj.vehicleManager;
            end
            if isempty(vm) || ~obj.hasField(vm, 'tire')
                return;
            end
            tire = vm.tire;
            slips = [obj.cornerSlip(tire, 'RL'), obj.cornerSlip(tire, 'RR')];
            slips = slips(isfinite(slips));
            if isempty(slips)
                return;
            end
            maxSlip = max(slips);
        end

        function sr = cornerSlip(obj, tire, cornerName)
            sr = NaN;
            if ~obj.hasField(tire, cornerName)
                return;
            end
            cornerState = tire.(cornerName);
            if ~obj.hasField(cornerState, 'slipRatio')
                return;
            end
            sr = cornerState.slipRatio;
            if ~isfinite(sr)
                sr = NaN;
            end
        end

        function tf = hasField(~, s, name)
            % HASFIELD True if s has a non-empty field/property `name`. Works
            % for both MATLAB structs and handle/value objects (isprop fails
            % on plain structs, isfield fails on objects).
            if isstruct(s)
                tf = isfield(s, name) && ~isempty(s.(name));
            elseif isobject(s)
                tf = isprop(s, name) && ~isempty(s.(name));
            else
                tf = false;
            end
        end

        function input = applyPedalFilter(obj, input)
            % APPLYPEDALFILTER First-order smoothing of the pedal outputs.
            if obj.pedalFilterTime <= 0 || ~isfinite(obj.pedalFilterTime)
                obj.lastThrottle = input.throttle;
                obj.lastBrake = input.brake;
                return;
            end
            if ~obj.inputStateInitialized
                obj.filteredThrottle = input.throttle;
                obj.filteredBrake = input.brake;
                obj.inputStateInitialized = true;
            end
            alpha = obj.inputDt / max(obj.pedalFilterTime + obj.inputDt, eps);
            obj.filteredThrottle = obj.filteredThrottle + ...
                alpha * (input.throttle - obj.filteredThrottle);
            obj.filteredBrake = obj.filteredBrake + ...
                alpha * (input.brake - obj.filteredBrake);
            obj.lastThrottle = obj.filteredThrottle;
            obj.lastBrake = obj.filteredBrake;
            input.throttle = max(0, min(1, obj.filteredThrottle));
            input.brake = max(0, min(1, obj.filteredBrake));
            [input.throttle, input.brake] = obj.enforceExclusivity( ...
                input.throttle, input.brake);
        end

        % ============================================================
        % Helpers
        % ============================================================
        function idx = lineIndex(obj, observation)
            % LINEINDEX Racing-line sample matching the projected ref index.
            idx = obj.fieldOr(observation, 'idx', NaN);
            n = numel(obj.racingLine.curvature);
            if isempty(idx) || ~isfinite(idx)
                idx = obj.lastIndex;
            end
            idx = max(1, min(round(idx), n));
            obj.lastIndex = idx;
        end

        function k = lineCurvature(obj, idx)
            % LINECURVATURE Smoothed racing-line curvature at idx (preferred
            % over the raw curvature, which is noisy on image-derived tracks).
            if isfield(obj.racingLine, 'curvatureSmoothed') && ...
                    ~isempty(obj.racingLine.curvatureSmoothed)
                k = obj.racingLine.curvatureSmoothed(idx);
            else
                k = obj.racingLine.curvature(idx);
            end
        end

        function e = crossTrackError(obj, observation)
            e = obj.fieldOr(observation, 'lateralError', 0);
            if isempty(e) || ~isfinite(e)
                e = 0;
            end
        end

        function observation = defaultObservationFromState(obj, state)
            refHeading = state.refHeading;
            if ~isfinite(refHeading); refHeading = state.heading; end
            refCurvature = state.refCurvature;
            if ~isfinite(refCurvature); refCurvature = state.curvature; end
            observation = struct( ...
                'idx', obj.lastIndex, ...
                's', state.s, ...
                'x', state.x, ...
                'y', state.y, ...
                'heading', refHeading, ...
                'curvature', refCurvature, ...
                'mu', state.mu, ...
                'lateralError', state.lateralError, ...
                'trackWidth', NaN, ...
                'trackHalfWidth', NaN, ...
                'trackLimitMargin', NaN, ...
                'onTrack', state.onTrack);
        end

        function [t, b] = enforceExclusivity(~, t, b)
            t = max(0, min(1, t));
            b = max(0, min(1, b));
            if b > 0
                t = 0;
            elseif t > 0
                b = 0;
            end
        end

        function ang = wrapAngle(~, ang)
            % WRAPANGLE Wrap an angle to (-pi, pi].
            ang = mod(ang + pi, 2 * pi) - pi;
            % Map the -pi exclusive boundary to +pi for symmetry.
            ang(ang <= -pi) = pi;
        end

        function val = fieldOr(obj, s, name, default)
            % FIELDOR Return s.name if present and nonempty, else default.
            % Robust to both structs and objects.
            if obj.hasField(s, name)
                val = s.(name);
            else
                val = default;
            end
        end
    end
end
