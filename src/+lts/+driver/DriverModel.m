classdef DriverModel < handle
    % Feedforward GGV planning with Stanley path correction.

    properties
        % Reference to lts.vehicle.VehicleManager for component access
        vehicleManager

        % Tuneable driver parameters
        brakingLookahead = 1.0    % Multiplier on calculated braking distance
        lookaheadTime    = 2.0    % Minimum seconds ahead to inspect
        minLookaheadDist = 15     % Minimum lookahead distance [m]
        corneringUsage   = 0.8   % Fraction of lateral grip used for speed targets
        brakingUsage     = 0.75   % Fraction of braking capability used in planning
        driveUsage       = 0.85   % Fraction of tractive/drive grip used in planning
        brakeBlendSpeed  = 1.0    % Speed error [m/s] that ramps braking
        throttleBand     = 0.15   % Speed band [m/s] around target before switching
        curvatureTol     = 1e-6   % Curvature below this is treated as straight
        steeringUsage    = 1.0    % Fraction of path curvature converted to steer
        maxSteeringAngle = 0.6    % Steering angle limit [rad]
        minLongitudinalCommandScale = 0.15 % Longitudinal command left at peak steer
        % Planned-path combined-grip limits.
        tractionCircleReserve = 0      % Min throttle fraction at peak lateral grip [0-1]
        trailBrakeReserve = 0.30       % Min brake fraction kept at peak lateral grip (trail-braking) [0-1]
        corneringGripMargin = 0.95     % Lateral-grip fraction at which throttle is fully reserved
        apexPhase        = 0.5    % Corner apex location as fraction from entry to exit
        racingLineOffsetFraction = 0.65 % Track-half-width fraction used by reduced planner
        racingLineCurvatureSmoothDistance = 6.0 % Racing-line curvature smoothing [m]
        racingLineOffsetSmoothDistance = 8.0 % Racing-line offset smoothing [m]
        stanleyGain      = 1.5    % Cross-track correction gain
        stanleySoftening = 1.5    % Low-speed softening term [m/s]
        headingGain      = 1.0    % Heading-error steering correction gain
        edgeSlowdownMargin = 0.75 % Start slowing this far from track edge [m]
        edgeBrakeCommand = 0.45   % Brake added near/outside track edge
        edgeSteeringMargin = 0.75 % Start steering away this far from track edge [m]
        edgeSteeringGain = 0.40   % Track-edge steering correction gain [rad]
        correctionSlowdownThreshold = 0.65 % Steering correction use before slowing
        correctionBrakeCommand = 0.15      % Brake added for large path corrections
        throttleRampTime = 0.10   % Time from 0 to 100% throttle [s]
        brakeRampTime    = 0.10   % Time from 0 to 100% brake [s]
        steeringRampTime = 0.10   % Time from center to full steering command [s]
        pedalSwitchHoldTime = 0.10    % Opposite pedal request must persist [s]
        immediateBrakeSwitchThreshold = 0.50 % Brake command that bypasses switch dwell
        pedalReductionHoldTime = 0.15 % Reduction must persist before pedal follows [s]
        pedalReleaseFilterTime = 0.20 % Smooth brief same-pedal reductions [s]
        pedalTargetDeadband = 0.04    % Ignore small same-pedal target changes
        launchSpeedThreshold = 0.5 % Below this, do not correction-brake under target speed [m/s]
        cornerOutsideBiasFraction = 0.35 % Fraction of half-width targeted outside in corners
        cornerOutsideBiasMax = 0.7 % Max outside target offset from centerline [m]
        enableDriveSlipLimit = true % Reduce throttle when driven rear slip is excessive
        driveSlipTarget = 0.12      % Rear slip ratio where throttle limiting starts
        driveSlipCutoff = 0.35      % Rear slip ratio where throttle is fully cut

        % Cached track geometry
        trackArcLen      = []
        trackCurvature   = []

        % Planned lap controls owned by the driver/controller layer
        inputPlanner     = []
        inputProfile     = []

        % Driver actuator state
        inputDt          = 0.001
        lastThrottle     = 0
        lastBrake        = 0
        lastSteer        = 0
        filteredThrottleTarget = 0
        filteredBrakeTarget = 0
        pendingThrottleReductionTarget = NaN
        pendingThrottleReductionTime = 0
        pendingBrakeReductionTarget = NaN
        pendingBrakeReductionTime = 0
        pendingPedalSwitchTarget = ""
        pendingPedalSwitchTime = 0
        inputStateInitialized = false
    end

    properties (Access = private)
        trackClosedLoop = false
        trackBaseLength = NaN
        trackTotalLaps = 1
        trackLapBreakS = []
        steadyCircleTrack = false
    end

    methods
        function obj = DriverModel(vehicleManager)
            % DRIVERMODEL Construct with a lts.vehicle.VehicleManager reference
            obj.vehicleManager = vehicleManager;
            obj = obj.cacheTrackGeometry();
        end

        function obj = prepareForSimulation(obj, initialState, trackData, dt)
            % PREPAREFORSIMULATION Build the driver's feedforward lap plan.
            if nargin >= 4 && isfinite(dt) && dt > 0
                obj.inputDt = dt;
            end
            [obj.lastThrottle, obj.lastBrake] = ...
                obj.resolvePedalTargets(initialState.throttle, initialState.brake);
            obj.lastSteer = obj.clampSteer(initialState.steer);
            obj.filteredThrottleTarget = obj.lastThrottle;
            obj.filteredBrakeTarget = obj.lastBrake;
            obj.resetPedalReductionMemory();
            obj.clearPendingPedalSwitch();
            obj.inputStateInitialized = true;
            obj.trackArcLen = trackData.arcLen(:);
            obj.trackCurvature = trackData.curvature(:);
            obj = obj.cacheTrackMetadataFromTrackData(trackData);
            obj.inputPlanner = lts.driver.DriverInputPlanner(obj.vehicleManager, obj);
            obj.inputProfile = obj.inputPlanner.buildOpenLoopProfile( ...
                initialState, trackData);
        end

        function input = computeInput(obj, state, observation)
            % COMPUTEINPUT Return throttle, brake, and steer for this state.
            % The simulator supplies observation telemetry; all control
            % policy and path correction lives in the driver layer.
            if nargin < 3 || isempty(observation)
                observation = obj.defaultObservationFromState(state);
            else
                observation = obj.completeObservation(observation, state);
            end

            if isempty(obj.inputProfile) || isempty(obj.inputPlanner)
                [throttle, brake, steer] = obj.computeInputs(state);
                input = struct( ...
                    'throttle', throttle, ...
                    'brake', brake, ...
                    'steer', steer, ...
                    'targetSpeed', NaN, ...
                    'axRef', NaN);
                input = obj.applyInputSlew(input, state);
                input = obj.applyDriveSlipLimit(input);
                return;
            end

            input = obj.inputPlanner.sampleAtProgress( ...
                obj.inputProfile, observation.s, state.speed);
            input = obj.correctPlannedInput(input, state, observation);
            input = obj.applyInputSlew(input, state);
            input = obj.applyDriveSlipLimit(input);
        end

        function [throttle, brake, steer] = computeInputs(obj, state)
            % Fallback controller used without a feedforward lap plan.

            speed = max(state.speed, 0);
            s = state.s;
            [arcLen, curvature] = obj.getTrackGeometry();
            nPts = numel(curvature);

            idx = find(arcLen <= s, 1, 'last');
            if isempty(idx)
                idx = 1;
            end
            idx = max(1, min(idx, nPts));

            caps = obj.estimateAvailableAcceleration(state);
            maxLateralAccel = caps.maxLatAccel;
            maxBrakeAccel = caps.maxBrakeAccel;
            lookAheadDist = obj.computeLookaheadDistance(speed, maxBrakeAccel);
            idxEnd = find(arcLen <= s + lookAheadDist, 1, 'last');
            if isempty(idxEnd)
                idxEnd = idx;
            end
            if idx < nPts
                idxEnd = max(idx + 1, min(idxEnd, nPts));
            else
                idxEnd = nPts;
            end

            profileIdx = idx:idxEnd;
            profileS = arcLen(profileIdx);
            profileS(1) = s;
            profileSpeed = obj.computeBackwardSpeedProfile( ...
                profileS, curvature(profileIdx), maxLateralAccel, maxBrakeAccel);

            targetSpeed = profileSpeed(1);

            [steer, steeringUsageFrac] = obj.computeSteeringCommand(idx, s);
            longitudinalCommandScale = obj.computeLongitudinalCommandScale(steeringUsageFrac);

            axRef = 0;
            if numel(profileSpeed) >= 2 && numel(profileS) >= 2
                ds0 = max(profileS(2) - profileS(1), 0.001);
                axRef = (profileSpeed(2)^2 - profileSpeed(1)^2) / (2 * ds0);
            end

            speedError = speed - targetSpeed;
            axRef = axRef - 0.5 * speedError;

            [throttle, brake] = lts.driver.DriverInputPlanner.computePedals( ...
                axRef, caps.F_drive_full, caps.F_resistance, ...
                obj.vehicleManager.totalMass, caps.brakeForceAccel, ...
                obj.vehicleManager.powertrain, longitudinalCommandScale);

            brake = brake * longitudinalCommandScale;
        end

        function input = correctPlannedInput(obj, plannedInput, state, ref)
            % Add path and track-edge corrections to feedforward input.
            input = plannedInput;
            input.throttle = lts.util.fieldOrDefault(input, 'throttle', 0);
            input.brake = lts.util.fieldOrDefault(input, 'brake', 0);
            input.steer = lts.util.fieldOrDefault(input, 'steer', 0);

            lineHeading = ref.heading;
            if isfield(input, 'lineHeading') && isfinite(input.lineHeading)
                lineHeading = input.lineHeading;
            end
            headingError = obj.wrapAngle(lineHeading - state.yaw);

            if isfield(input, 'targetLateralError') && ...
                    isfinite(input.targetLateralError)
                targetLateralError = input.targetLateralError;
            else
                targetLateralError = obj.computeTargetLateralError(ref);
            end
            lateralTrackingError = ref.lateralError - targetLateralError;
            crossTrackCorrection = atan2( ...
                -obj.stanleyGain * lateralTrackingError, ...
                max(state.speed, 0) + obj.stanleySoftening);
            plannedSteer = input.steer;
            steeringCorrection = obj.headingGain * headingError + crossTrackCorrection;
            steeringCorrection = steeringCorrection + ...
                obj.computeEdgeSteeringCorrection(ref);
            steer = plannedSteer + steeringCorrection;
            input.steer = lts.util.clamp(steer, -obj.maxSteeringAngle, obj.maxSteeringAngle);
            input.targetLateralError = targetLateralError;

            correctionUse = abs(steeringCorrection) / max(obj.maxSteeringAngle, eps);
            correctionUse = lts.util.saturate(correctionUse);
            if correctionUse > obj.correctionSlowdownThreshold
                slowdownUse = (correctionUse - obj.correctionSlowdownThreshold) / ...
                    max(1 - obj.correctionSlowdownThreshold, eps);
                slowdownUse = lts.util.saturate(slowdownUse);
                input.throttle = input.throttle * (1 - 0.40 * slowdownUse);
                input.brake = max(input.brake, obj.correctionBrakeCommand * slowdownUse);
            end

            if isfield(ref, 'trackHalfWidth') && isfinite(ref.trackHalfWidth)
                halfWidth = lts.driver.DriverModel.refHalfWidthForSide(ref, ref.lateralError);
                margin = halfWidth - abs(ref.lateralError);
                if margin < obj.edgeSlowdownMargin
                    edgeUse = (obj.edgeSlowdownMargin - margin) / ...
                        max(obj.edgeSlowdownMargin, eps);
                    edgeUse = lts.util.saturate(edgeUse);
                    input.throttle = input.throttle * (1 - 0.85 * edgeUse);
                    input.brake = max(input.brake, obj.edgeBrakeCommand * edgeUse);
                end
            end

            input.throttle = lts.util.saturate(input.throttle);
            input.brake = lts.util.saturate(input.brake);
            if state.speed < obj.launchSpeedThreshold && ...
                    obj.isAtOrBelowTargetSpeed(state.speed, input)
                input.brake = 0;
                input.throttle = 1;
            end
            input = obj.enforcePedalExclusivity(input);
        end
    end

    methods (Access = private)
        function input = applyDriveSlipLimit(obj, input)
            % Reduce throttle between the target and cutoff rear slip.
            if ~obj.enableDriveSlipLimit || ~isfield(input, 'throttle') || ...
                    input.throttle <= 0
                return;
            end

            maxRearSlip = obj.getMaxRearDriveSlip();
            if ~isfinite(maxRearSlip)
                return;
            end

            slipTarget = max(0, obj.driveSlipTarget);
            slipCutoff = max(slipTarget + eps, obj.driveSlipCutoff);
            if maxRearSlip <= slipTarget
                return;
            end

            slipUse = (maxRearSlip - slipTarget) / max(slipCutoff - slipTarget, eps);
            slipUse = lts.util.saturate(slipUse);
            input.throttle = input.throttle * (1 - slipUse);
        end

        function maxRearSlip = getMaxRearDriveSlip(obj)
            maxRearSlip = NaN;
            tire = [];
            if ~isempty(obj.vehicleManager)
                tire = obj.vehicleManager.tire;
            end
            if isempty(tire)
                return;
            end

            if isa(tire, 'lts.components.Tire.PacejkaTire')
                slipRL = tire.RL.slipRatio;
                slipRR = tire.RR.slipRatio;
            else
                slipRL = obj.getCornerSlipRatio(tire, 'RL');
                slipRR = obj.getCornerSlipRatio(tire, 'RR');
            end
            rearSlip = [slipRL, slipRR];
            rearSlip = rearSlip(isfinite(rearSlip));
            if isempty(rearSlip)
                return;
            end
            maxRearSlip = max(rearSlip);
        end

        function slipRatio = getCornerSlipRatio(obj, tire, cornerName)
            slipRatio = NaN;
            cornerState = obj.getFieldValue(tire, cornerName, []);
            if isempty(cornerState)
                return;
            end
            slipRatio = obj.getFieldValue(cornerState, 'slipRatio', NaN);
        end

        function input = applyInputSlew(obj, input, state)
            if ~obj.inputStateInitialized
                [obj.lastThrottle, obj.lastBrake] = ...
                    obj.resolvePedalTargets(state.throttle, state.brake);
                obj.lastSteer = obj.clampSteer(state.steer);
                obj.filteredThrottleTarget = obj.lastThrottle;
                obj.filteredBrakeTarget = obj.lastBrake;
                obj.resetPedalReductionMemory();
                obj.clearPendingPedalSwitch();
                obj.inputStateInitialized = true;
            end

            [targetThrottle, targetBrake] = obj.resolvePedalTargets( ...
                input.throttle, input.brake);
            [targetThrottle, targetBrake] = obj.filterPedalTargets( ...
                targetThrottle, targetBrake);
            targetSteer = obj.clampSteer(input.steer);

            if targetBrake > 0 && obj.lastThrottle > 0
                input.throttle = obj.slewCommand( ...
                    obj.lastThrottle, 0, obj.throttleRampTime);
                input.brake = 0;
            elseif targetThrottle > 0 && obj.lastBrake > 0
                input.throttle = 0;
                input.brake = obj.slewCommand( ...
                    obj.lastBrake, 0, obj.brakeRampTime);
            elseif targetThrottle > 0
                input.throttle = obj.slewCommand( ...
                    obj.lastThrottle, targetThrottle, obj.throttleRampTime);
                input.brake = 0;
            elseif targetBrake > 0
                input.throttle = 0;
                input.brake = obj.slewCommand( ...
                    obj.lastBrake, targetBrake, obj.brakeRampTime);
            elseif obj.lastBrake > 0
                input.throttle = 0;
                input.brake = obj.slewCommand( ...
                    obj.lastBrake, 0, obj.brakeRampTime);
            else
                input.throttle = obj.slewCommand( ...
                    obj.lastThrottle, 0, obj.throttleRampTime);
                input.brake = 0;
            end

            obj.lastThrottle = input.throttle;
            obj.lastBrake = input.brake;

            input.steer = obj.slewSteeringCommand(obj.lastSteer, targetSteer);
            obj.lastSteer = input.steer;
        end

        function [targetThrottle, targetBrake] = filterPedalTargets(obj, targetThrottle, targetBrake)
            [targetThrottle, targetBrake] = obj.applyPedalSwitchDwell( ...
                targetThrottle, targetBrake);

            if targetBrake > 0
                obj.filteredThrottleTarget = 0;
                obj.filteredBrakeTarget = obj.filterSamePedalReduction( ...
                    "brake", obj.filteredBrakeTarget, targetBrake);
            elseif targetThrottle > 0
                obj.filteredBrakeTarget = 0;
                obj.filteredThrottleTarget = obj.filterSamePedalReduction( ...
                    "throttle", obj.filteredThrottleTarget, targetThrottle);
            else
                obj.filteredThrottleTarget = obj.filterSamePedalReduction( ...
                    "throttle", obj.filteredThrottleTarget, 0);
                obj.filteredBrakeTarget = obj.filterSamePedalReduction( ...
                    "brake", obj.filteredBrakeTarget, 0);
            end

            [targetThrottle, targetBrake] = obj.resolvePedalTargets( ...
                obj.filteredThrottleTarget, obj.filteredBrakeTarget);
        end

        function [targetThrottle, targetBrake] = applyPedalSwitchDwell(obj, targetThrottle, targetBrake)
            activeThrottle = max(obj.lastThrottle, obj.filteredThrottleTarget);
            activeBrake = max(obj.lastBrake, obj.filteredBrakeTarget);

            if targetBrake > 0 && activeThrottle > obj.pedalTargetDeadband
                if targetBrake >= obj.immediateBrakeSwitchThreshold
                    obj.clearPendingPedalSwitch();
                elseif ~obj.pedalSwitchHasPersisted("brake")
                    targetThrottle = activeThrottle;
                    targetBrake = 0;
                end
            elseif targetThrottle > 0 && activeBrake > obj.pedalTargetDeadband
                if ~obj.pedalSwitchHasPersisted("throttle")
                    targetThrottle = 0;
                    targetBrake = activeBrake;
                end
            else
                obj.clearPendingPedalSwitch();
            end
        end

        function persisted = pedalSwitchHasPersisted(obj, targetPedal)
            if obj.pedalSwitchHoldTime <= 0 || ~isfinite(obj.pedalSwitchHoldTime)
                persisted = true;
                return;
            end

            if obj.pendingPedalSwitchTarget ~= targetPedal
                obj.pendingPedalSwitchTarget = targetPedal;
                obj.pendingPedalSwitchTime = 0;
                persisted = false;
                return;
            end

            obj.pendingPedalSwitchTime = obj.pendingPedalSwitchTime + obj.inputDt;
            persisted = obj.pendingPedalSwitchTime >= obj.pedalSwitchHoldTime;
        end

        function clearPendingPedalSwitch(obj)
            obj.pendingPedalSwitchTarget = "";
            obj.pendingPedalSwitchTime = 0;
        end

        function value = filterSamePedalReduction(obj, pedalName, previousValue, targetValue)
            targetValue = lts.util.saturate(targetValue);
            previousValue = lts.util.saturate(previousValue);

            if targetValue >= previousValue
                obj.clearPendingPedalReduction(pedalName);
                value = targetValue;
                return;
            end

            if previousValue - targetValue <= obj.pedalTargetDeadband
                obj.clearPendingPedalReduction(pedalName);
                value = previousValue;
                return;
            end

            if ~obj.reductionTargetHasPersisted(pedalName, targetValue)
                value = previousValue;
                return;
            end

            if obj.pedalReleaseFilterTime <= 0 || ~isfinite(obj.pedalReleaseFilterTime)
                value = targetValue;
                return;
            end

            alpha = obj.inputDt / max(obj.pedalReleaseFilterTime + obj.inputDt, eps);
            value = previousValue + alpha * (targetValue - previousValue);
            if value < obj.pedalTargetDeadband && targetValue <= obj.pedalTargetDeadband
                value = 0;
            end
        end

        function persisted = reductionTargetHasPersisted(obj, pedalName, targetValue)
            if obj.pedalReductionHoldTime <= 0 || ~isfinite(obj.pedalReductionHoldTime)
                persisted = true;
                return;
            end

            [targetField, timeField] = obj.getPendingPedalReductionFields(pedalName);
            pendingTarget = obj.(targetField);
            if ~isfinite(pendingTarget) || ...
                    abs(pendingTarget - targetValue) > obj.pedalTargetDeadband
                obj.(targetField) = targetValue;
                obj.(timeField) = 0;
                persisted = false;
                return;
            end

            obj.(timeField) = obj.(timeField) + obj.inputDt;
            persisted = obj.(timeField) >= obj.pedalReductionHoldTime;
        end

        function clearPendingPedalReduction(obj, pedalName)
            [targetField, timeField] = obj.getPendingPedalReductionFields(pedalName);
            obj.(targetField) = NaN;
            obj.(timeField) = 0;
        end

        function resetPedalReductionMemory(obj)
            obj.clearPendingPedalReduction("throttle");
            obj.clearPendingPedalReduction("brake");
        end

        function [targetField, timeField] = getPendingPedalReductionFields(~, pedalName)
            if strcmp(string(pedalName), "brake")
                targetField = 'pendingBrakeReductionTarget';
                timeField = 'pendingBrakeReductionTime';
            else
                targetField = 'pendingThrottleReductionTarget';
                timeField = 'pendingThrottleReductionTime';
            end
        end

        function input = enforcePedalExclusivity(obj, input)
            [input.throttle, input.brake] = obj.resolvePedalTargets( ...
                input.throttle, input.brake);
        end

        function [throttle, brake] = resolvePedalTargets(~, throttle, brake)
            throttle = lts.util.saturate(throttle);
            brake = lts.util.saturate(brake);

            if brake > 0
                throttle = 0;
            end
        end

        function atOrBelow = isAtOrBelowTargetSpeed(~, speed, input)
            atOrBelow = true;
            if isfield(input, 'targetSpeed') && isfinite(input.targetSpeed)
                atOrBelow = speed <= input.targetSpeed;
            end
        end

        function steer = clampSteer(obj, steer)
            steer = lts.util.clamp(steer, -obj.maxSteeringAngle, obj.maxSteeringAngle);
        end

        function steer = slewSteeringCommand(obj, previousSteer, targetSteer)
            if obj.steeringRampTime <= 0 || ~isfinite(obj.steeringRampTime)
                steer = targetSteer;
                return;
            end

            maxDelta = obj.maxSteeringAngle * obj.inputDt / ...
                max(obj.steeringRampTime, eps);
            delta = targetSteer - previousSteer;
            delta = lts.util.clamp(delta, -maxDelta, maxDelta);
            steer = obj.clampSteer(previousSteer + delta);
        end

        function value = slewCommand(obj, previousValue, targetValue, rampTime)
            if rampTime <= 0 || ~isfinite(rampTime)
                value = targetValue;
                return;
            end

            maxDelta = obj.inputDt / rampTime;
            delta = targetValue - previousValue;
            delta = lts.util.clamp(delta, -maxDelta, maxDelta);
            value = previousValue + delta;
            value = lts.util.saturate(value);
        end

        function targetLateralError = computeTargetLateralError(obj, ref)
            targetLateralError = 0;
            if ~isfield(ref, 'curvature') || abs(ref.curvature) <= obj.curvatureTol
                return;
            end
            if obj.isSteadyCircleControl()
                return;
            end
            if ~isfield(ref, 'trackHalfWidth') || ~isfinite(ref.trackHalfWidth) || ...
                    ref.trackHalfWidth <= 0
                return;
            end

            biasSideLateralError = -sign(ref.curvature);
            biasHalfWidth = lts.driver.DriverModel.refHalfWidthForSide(ref, biasSideLateralError);
            usableOffset = max(biasHalfWidth - ...
                max(obj.edgeSlowdownMargin, obj.edgeSteeringMargin), 0);
            outsideBias = obj.cornerOutsideBiasFraction * biasHalfWidth;
            outsideBias = min([outsideBias, obj.cornerOutsideBiasMax, usableOffset]);
            targetLateralError = -sign(ref.curvature) * outsideBias;

            [arcLen, ~] = obj.getTrackGeometry();
            if isfield(ref, 'idx') && isfinite(ref.idx)
                idx = max(1, min(round(ref.idx), numel(arcLen)));
            elseif isfield(ref, 's') && isfinite(ref.s)
                idx = find(arcLen <= ref.s, 1, 'last');
                if isempty(idx)
                    idx = 1;
                end
            else
                return;
            end

            [segmentStart, segmentEnd, turnSign, found] = obj.findCornerSegment(idx);
            if ~found || turnSign == 0
                return;
            end

            segmentStartS = arcLen(segmentStart);
            segmentEndS = arcLen(segmentEnd);
            segmentLength = max(segmentEndS - segmentStartS, eps);
            if isfield(ref, 's') && isfinite(ref.s)
                cornerS = ref.s;
            else
                cornerS = arcLen(idx);
            end
            phase = (cornerS - segmentStartS) / segmentLength;
            phase = lts.util.saturate(phase);
            apexPhaseClamped = obj.getClampedApexPhase();

            if phase <= apexPhaseClamped
                blend = phase / max(apexPhaseClamped, eps);
                targetLateralError = -turnSign * outsideBias * cos(pi * blend);
            else
                blend = (phase - apexPhaseClamped) / ...
                    max(1 - apexPhaseClamped, eps);
                targetLateralError = turnSign * outsideBias * cos(pi * blend);
            end
        end

        function correction = computeEdgeSteeringCorrection(obj, ref)
            correction = 0;
            if ~isfield(ref, 'trackHalfWidth') || ~isfinite(ref.trackHalfWidth) || ...
                    ref.trackHalfWidth <= 0
                return;
            end
            if ~isfield(ref, 'lateralError') || ~isfinite(ref.lateralError) || ...
                    abs(ref.lateralError) <= eps
                return;
            end

            halfWidth = lts.driver.DriverModel.refHalfWidthForSide(ref, ref.lateralError);
            margin = halfWidth - abs(ref.lateralError);
            if margin >= obj.edgeSteeringMargin
                return;
            end

            edgeUse = (obj.edgeSteeringMargin - margin) / ...
                max(obj.edgeSteeringMargin, eps);
            edgeUse = lts.util.saturate(edgeUse);

            correction = -sign(ref.lateralError) * obj.edgeSteeringGain * edgeUse;
        end

        function observation = defaultObservationFromState(~, state)
            refHeading = state.refHeading;
            if ~isfinite(refHeading)
                refHeading = state.heading;
            end
            refCurvature = state.refCurvature;
            if ~isfinite(refCurvature)
                refCurvature = state.curvature;
            end

            observation = struct( ...
                'idx', 1, ...
                's', state.s, ...
                'x', state.x, ...
                'y', state.y, ...
                'heading', refHeading, ...
                'curvature', refCurvature, ...
                'mu', state.mu, ...
                'lateralError', state.lateralError, ...
                'trackWidth', NaN, ...
                'trackHalfWidth', NaN, ...
                'leftHalfWidth', NaN, ...
                'rightHalfWidth', NaN, ...
                'trackLimitMargin', NaN, ...
                'onTrack', state.onTrack);
        end

        function observation = completeObservation(obj, observation, state)
            defaults = obj.defaultObservationFromState(state);
            fields = fieldnames(defaults);
            for i = 1:numel(fields)
                field = fields{i};
                if ~isfield(observation, field) || isempty(observation.(field))
                    observation.(field) = defaults.(field);
                end
            end
        end

        function angle = wrapAngle(~, angle)
            angle = atan2(sin(angle), cos(angle));
        end

        function value = getVehicleManagerValue(obj, fieldName, defaultValue)
            value = obj.getFieldValue(obj.vehicleManager, fieldName, defaultValue);
        end

        function value = getFieldValue(~, source, fieldName, defaultValue)
            value = defaultValue;
            if isempty(source)
                return;
            end
            if (isstruct(source) && isfield(source, fieldName)) || ...
                    (isobject(source) && isprop(source, fieldName))
                value = source.(fieldName);
            end
        end

        function obj = cacheTrackGeometry(obj)
            track = obj.getVehicleManagerValue('track', []);
            if isempty(track)
                return;
            end
            trackPts = track.getTrackPoints();

            dx = diff(trackPts(:,1));
            dy = diff(trackPts(:,2));
            obj.trackArcLen = [0; cumsum(sqrt(dx.^2 + dy.^2))];
            obj.trackCurvature = track.getCurvature();
            obj.trackCurvature = obj.trackCurvature(:);
            obj = obj.cacheTrackMetadataFromTrack(track, obj.trackArcLen(end));
        end

        function obj = cacheTrackMetadataFromTrack(obj, track, baseTrackLength)
            obj.trackClosedLoop = false;
            obj.trackBaseLength = baseTrackLength;
            obj.trackTotalLaps = 1;
            obj.trackLapBreakS = [0; baseTrackLength];

            if isempty(track) || ~isfinite(baseTrackLength) || baseTrackLength <= 0
                obj.steadyCircleTrack = false;
                return;
            end

            if ismethod(track, 'isClosedLoop')
                obj.trackClosedLoop = logical(track.isClosedLoop());
            elseif isprop(track, 'closedLoop')
                obj.trackClosedLoop = logical(track.closedLoop);
            elseif isprop(track, 'Closed')
                % WaypointTrack exposes Closed rather than closedLoop/isClosedLoop
                obj.trackClosedLoop = logical(track.Closed);
            end

            warmupLaps = 0;
            recordedLaps = 1;
            if ismethod(track, 'getWarmupLaps')
                warmupLaps = track.getWarmupLaps();
            elseif isprop(track, 'warmupLaps')
                warmupLaps = track.warmupLaps;
            end
            if ismethod(track, 'getRecordedLaps')
                recordedLaps = track.getRecordedLaps();
            elseif isprop(track, 'recordedLaps')
                recordedLaps = track.recordedLaps;
            end

            obj.trackTotalLaps = max(1, round(max(0, warmupLaps) + ...
                max(1, recordedLaps)));
            if ~obj.trackClosedLoop
                obj.trackTotalLaps = 1;
            end
            obj.trackLapBreakS = (0:obj.trackTotalLaps)' * obj.trackBaseLength;
            obj.steadyCircleTrack = obj.computeSteadyCircleTrack( ...
                obj.trackCurvature, obj.trackClosedLoop);
        end

        function obj = cacheTrackMetadataFromTrackData(obj, trackData)
            obj.trackClosedLoop = false;
            obj.trackBaseLength = NaN;
            obj.trackTotalLaps = 1;
            obj.trackLapBreakS = [];

            if isfield(trackData, 'closedLoop')
                obj.trackClosedLoop = logical(trackData.closedLoop);
            end
            if isfield(trackData, 'baseTrackLength')
                obj.trackBaseLength = trackData.baseTrackLength;
            elseif isfield(trackData, 'length')
                obj.trackBaseLength = trackData.length;
            end
            if isfield(trackData, 'totalLaps')
                obj.trackTotalLaps = max(1, round(trackData.totalLaps));
            end
            if isfield(trackData, 'lapBreakS') && ~isempty(trackData.lapBreakS)
                obj.trackLapBreakS = trackData.lapBreakS(:);
            elseif isfinite(obj.trackBaseLength) && obj.trackBaseLength > 0
                obj.trackLapBreakS = (0:obj.trackTotalLaps)' * obj.trackBaseLength;
            end

            obj.steadyCircleTrack = obj.computeSteadyCircleTrack( ...
                obj.trackCurvature, obj.trackClosedLoop);
        end

        function steady = computeSteadyCircleTrack(obj, curvature, closedLoop)
            steady = false;
            if ~closedLoop || isempty(curvature)
                return;
            end

            curvature = curvature(:);
            active = abs(curvature) > obj.curvatureTol;
            if ~all(active)
                return;
            end
            turnSign = sign(curvature(find(active, 1, 'first')));
            steady = turnSign ~= 0 && all(sign(curvature(active)) == turnSign);
        end

        function tf = isSteadyCircleControl(obj)
            tf = obj.trackClosedLoop && obj.steadyCircleTrack;
        end

        function [arcLen, curvature] = getTrackGeometry(obj)
            if ~isempty(obj.trackArcLen) && ~isempty(obj.trackCurvature)
                arcLen = obj.trackArcLen;
                curvature = obj.trackCurvature;
                return;
            end

            track = obj.vehicleManager.track;
            trackPts = track.getTrackPoints();
            dx = diff(trackPts(:,1));
            dy = diff(trackPts(:,2));
            arcLen = [0; cumsum(sqrt(dx.^2 + dy.^2))];
            curvature = track.getCurvature();
            curvature = curvature(:);
        end

        function caps = estimateAvailableAcceleration(obj, state)
            % Quasi-static grip and powertrain capability for planning.
            vm = obj.vehicleManager;
            aeroForces = vm.aero.computeForces(state);
            F_drag = max(0, aeroForces.F_drag);
            F_downforce = aeroForces.Fz_front + aeroForces.Fz_rear;
            W = vm.totalMass * vm.g;
            totalNormalLoad = W + F_downforce;

            peakMu = vm.tire.getPeakFriction(totalNormalLoad / 4);
            maxTireAccel = max(peakMu, 0) * totalNormalLoad / vm.totalMass;
            maxLateralAccel = max(0.1, maxTireAccel * obj.corneringUsage);

            frontNormalLoad = max(W * vm.staticFrontWeight + aeroForces.Fz_front, 0);
            rearNormalLoad = max(W * (1 - vm.staticFrontWeight) + aeroForces.Fz_rear, 0);
            frontMu = max(vm.tire.getPeakFriction(frontNormalLoad / 2), 0);
            rearMu = max(vm.tire.getPeakFriction(rearNormalLoad / 2), 0);
            brakeBiasFront = lts.util.saturate(vm.brakeBiasFront);
            brakeBiasRear = 1 - brakeBiasFront;
            brakeGripLimit = inf;
            if brakeBiasFront > eps
                brakeGripLimit = min(brakeGripLimit, frontMu * frontNormalLoad / brakeBiasFront);
            end
            if brakeBiasRear > eps
                brakeGripLimit = min(brakeGripLimit, rearMu * rearNormalLoad / brakeBiasRear);
            end

            maxBrakeForce = min(vm.brakeForceCoefficient * totalNormalLoad, brakeGripLimit);
            rollingResistance = obj.getRollingResistanceCoeff() * totalNormalLoad;
            brakeLimitedAccel = ...
                (maxBrakeForce + F_drag + rollingResistance) / vm.totalMass;

            maxBrakeAccel = min(maxTireAccel, brakeLimitedAccel) * obj.brakingUsage;
            maxBrakeAccel = max(maxBrakeAccel, 0.1);

            F_drive_full = 0;
            if ~isempty(vm.powertrain)
                F_drive_full = max(0, vm.powertrain.computeMaxDriveForce( ...
                    max(state.speed, 0)));
            end
            tractionUsage = lts.util.saturate(obj.driveUsage);
            F_traction_rear = tractionUsage * rearMu * rearNormalLoad;
            F_drive_full = min(F_drive_full, F_traction_rear);

            F_resistance = F_drag + rollingResistance;
            maxDriveAccel = max(0, (F_drive_full - F_resistance) / vm.totalMass);
            coastDecel = F_resistance / vm.totalMass;
            brakeForceAccel = max(0.1, obj.brakingUsage * maxBrakeForce / vm.totalMass);

            caps.maxLatAccel = maxLateralAccel;
            caps.maxBrakeAccel = maxBrakeAccel;
            caps.F_drive_full = F_drive_full;
            caps.F_resistance = F_resistance;
            caps.maxDriveAccel = maxDriveAccel;
            caps.coastDecel = coastDecel;
            caps.brakeForceAccel = brakeForceAccel;
        end

        function lookAheadDist = computeLookaheadDistance(obj, speed, maxBrakeAccel)
            brakeDistance = speed^2 / (2 * maxBrakeAccel);
            lookAheadDist = max([ ...
                obj.minLookaheadDist, ...
                speed * obj.lookaheadTime, ...
                brakeDistance * obj.brakingLookahead + obj.minLookaheadDist]);
        end

        function crr = getRollingResistanceCoeff(obj)
            crr = 0.015;
            tire = [];
            if ~isempty(obj.vehicleManager)
                tire = obj.vehicleManager.tire;
            end
            if isempty(tire)
                return;
            end
            if isa(tire, 'lts.components.Tire.PacejkaTire')
                crr = tire.rollingResistanceCoeff;
            elseif isstruct(tire) && isfield(tire, 'rollingResistanceCoeff')
                crr = tire.rollingResistanceCoeff;
            elseif isobject(tire) && isprop(tire, 'rollingResistanceCoeff')
                crr = tire.rollingResistanceCoeff;
            end
            if isempty(crr) || ~isfinite(crr)
                crr = 0.015;
            end
            crr = max(crr, 0);
        end

        function profileSpeed = computeBackwardSpeedProfile(obj, profileS, curvature, maxLateralAccel, maxBrakeAccel)
            vm = obj.vehicleManager;
            profileSpeed = obj.computeCornerSpeedLimit(curvature, maxLateralAccel);
            profileSpeed = min(profileSpeed, vm.maxSpeed);

            for i = numel(profileSpeed)-1:-1:1
                ds = max(profileS(i+1) - profileS(i), 0.001);
                brakeTerm = profileSpeed(i+1)^2 + 2 * maxBrakeAccel * ds;
                reachableSpeed = sqrt(max(brakeTerm, 0));
                profileSpeed(i) = min(profileSpeed(i), reachableSpeed);
            end
        end

        function speedLimit = computeCornerSpeedLimit(obj, curvature, maxLateralAccel)
            vm = obj.vehicleManager;
            absKappa = abs(curvature(:));
            speedLimit = vm.maxSpeed * ones(size(absKappa));

            cornerIdx = absKappa > obj.curvatureTol;
            lateralAccel = max(maxLateralAccel, 0.1);
            speedLimit(cornerIdx) = sqrt(lateralAccel ./ absKappa(cornerIdx));
            speedLimit = min(speedLimit, vm.maxSpeed);
        end

        function [steer, steeringUsageFrac] = computeSteeringCommand(obj, idx, s)
            [arcLen, curvature] = obj.getTrackGeometry();
            steer = 0;
            steeringUsageFrac = 0;

            [segmentStart, segmentEnd, turnSign, found] = obj.findCornerSegment(idx);
            if ~found
                return;
            end

            segmentStartS = arcLen(segmentStart);
            segmentEndS = arcLen(segmentEnd);
            segmentLength = max(segmentEndS - segmentStartS, eps);
            phase = (s - segmentStartS) / segmentLength;
            phase = lts.util.saturate(phase);

            if obj.isSteadyCircleControl()
                steeringUsageFrac = 1;
            else
                apexPhaseClamped = obj.getClampedApexPhase();
                if phase <= apexPhaseClamped
                    steeringUsageFrac = sin((pi / 2) * phase / apexPhaseClamped);
                else
                    steeringUsageFrac = sin((pi / 2) * (1 - phase) / (1 - apexPhaseClamped));
                end
            end
            peakKappa = max(abs(curvature(segmentStart:segmentEnd)));
            peakSteer = atan(obj.vehicleManager.wheelbase * peakKappa) * obj.steeringUsage;
            peakSteer = min(obj.maxSteeringAngle, peakSteer);

            steer = turnSign * peakSteer * steeringUsageFrac;
        end

        function scale = computeLongitudinalCommandScale(obj, steeringUsageFrac)
            lateralUse = lts.util.saturate(abs(steeringUsageFrac));
            ellipseScale = sqrt(max(0, 1 - lateralUse^2));
            reserve = lts.util.saturate(obj.minLongitudinalCommandScale);
            scale = reserve + (1 - reserve) * ellipseScale;
        end

        function [segmentStart, segmentEnd, turnSign, found] = findCornerSegment(obj, idx)
            [arcLen, curvature] = obj.getTrackGeometry();
            absKappa = abs(curvature);
            nPts = numel(curvature);
            segmentStart = 1;
            segmentEnd = 1;
            turnSign = 0;
            found = false;

            if idx > nPts || all(absKappa <= obj.curvatureTol)
                return;
            end
            idx = max(1, min(idx, nPts));
            [lapStartIdx, lapEndIdx] = obj.getLapIndexBounds(idx, arcLen);

            if absKappa(idx) > obj.curvatureTol
                segmentStart = idx;
                turnSign = sign(curvature(idx));
                while segmentStart > lapStartIdx && ...
                        absKappa(segmentStart - 1) > obj.curvatureTol && ...
                        sign(curvature(segmentStart - 1)) == turnSign
                    segmentStart = segmentStart - 1;
                end
            else
                nextCornerOffset = find( ...
                    absKappa(idx:lapEndIdx) > obj.curvatureTol, 1, 'first');
                if isempty(nextCornerOffset)
                    return;
                end
                segmentStart = idx + nextCornerOffset - 1;
                turnSign = sign(curvature(segmentStart));
            end

            segmentEnd = segmentStart;
            while segmentEnd < lapEndIdx && ...
                    absKappa(segmentEnd + 1) > obj.curvatureTol && ...
                    sign(curvature(segmentEnd + 1)) == turnSign
                segmentEnd = segmentEnd + 1;
            end

            found = true;
        end

        function [lapStartIdx, lapEndIdx] = getLapIndexBounds(obj, idx, arcLen)
            nPts = numel(arcLen);
            lapStartIdx = 1;
            lapEndIdx = nPts;
            if ~obj.trackClosedLoop || isempty(obj.trackLapBreakS) || ...
                    numel(obj.trackLapBreakS) < 2
                return;
            end

            idx = max(1, min(idx, nPts));
            s = arcLen(idx);
            lapIdx = find(obj.trackLapBreakS <= s + 1e-9, 1, 'last');
            if isempty(lapIdx)
                lapIdx = 1;
            end
            lapIdx = min(max(lapIdx, 1), numel(obj.trackLapBreakS) - 1);

            startS = obj.trackLapBreakS(lapIdx);
            endS = obj.trackLapBreakS(lapIdx + 1);
            lapStartIdx = find(arcLen >= startS - 1e-9, 1, 'first');
            lapEndIdx = find(arcLen <= endS + 1e-9, 1, 'last');
            if isempty(lapStartIdx)
                lapStartIdx = 1;
            end
            if isempty(lapEndIdx)
                lapEndIdx = nPts;
            end
            lapStartIdx = max(1, min(lapStartIdx, nPts));
            lapEndIdx = max(lapStartIdx, min(lapEndIdx, nPts));
        end

        function apexPhaseClamped = getClampedApexPhase(obj)
            apexPhaseClamped = lts.util.clamp(obj.apexPhase, 0.05, 0.95);
        end
    end

    methods (Static)
        function halfWidth = refHalfWidthForSide(ref, lateralValue)
            % Per-side width, falling back to the symmetric width.
            fallback = [];
            if isfield(ref, 'trackHalfWidth')
                fallback = ref.trackHalfWidth;
            end
            if lateralValue >= 0
                if isfield(ref, 'leftHalfWidth') && isfinite(ref.leftHalfWidth)
                    halfWidth = ref.leftHalfWidth;
                elseif ~isempty(fallback) && isfinite(fallback)
                    halfWidth = fallback;
                else
                    halfWidth = Inf;
                end
            else
                if isfield(ref, 'rightHalfWidth') && isfinite(ref.rightHalfWidth)
                    halfWidth = ref.rightHalfWidth;
                elseif ~isempty(fallback) && isfinite(fallback)
                    halfWidth = fallback;
                else
                    halfWidth = Inf;
                end
            end
        end
    end
end
