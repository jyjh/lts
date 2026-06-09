classdef DriverModel
    % DRIVERMODEL Decides throttle and brake inputs from a racing speed profile
    %
    % The driver builds a local backward speed envelope from upcoming
    % curvature and available braking. It stays at full throttle until the
    % latest feasible braking point, then uses a high brake command. The
    % only intentional coast state is near a detected corner apex. Steering
    % is shaped by the active corner segment, peaking at a configurable
    % apex phase that defaults to the corner midpoint.

    properties
        % Reference to VehicleManager for component access
        vehicleManager

        % Tuneable driver parameters
        brakingLookahead = 1.0    % Multiplier on calculated braking distance
        lookaheadTime    = 2.0    % Minimum seconds ahead to inspect
        minLookaheadDist = 15     % Minimum lookahead distance [m]
        hysteresis       = 0.005  % Speed tolerance as a fraction of target speed
        corneringUsage   = 0.99   % Fraction of lateral grip used for speed targets
        brakingUsage     = 0.98   % Fraction of braking capability used in planning
        minBrakeCommand  = 0.85   % Minimum brake command once braking is required
        brakeBlendSpeed  = 1.0    % Speed error [m/s] that ramps brake to 100%
        throttleBand     = 0.15   % Speed band [m/s] around target before switching
        apexDistanceTol  = 0.75   % Distance around an apex allowed to coast [m]
        curvatureTol     = 1e-6   % Curvature below this is treated as straight
        steeringUsage    = 1.0    % Fraction of path curvature converted to steer
        maxSteeringAngle = 0.6    % Steering angle limit [rad]
        minLongitudinalCommandScale = 0.15 % Longitudinal command left at peak steer
        apexPhase        = 0.5    % Corner apex location as fraction from entry to exit
        yawRateFeedbackGain = 0.08 % Steering correction per yaw-rate error
        sideslipFeedbackGain = 0.70 % Steering correction per sideslip angle

        % Cached track geometry
        trackArcLen      = []
        trackCurvature   = []
    end

    methods
        function obj = DriverModel(vehicleManager)
            % DRIVERMODEL Construct with a VehicleManager reference
            obj.vehicleManager = vehicleManager;
            obj = obj.cacheTrackGeometry();
        end

        function [throttle, brake, steer] = computeInputs(obj, state)
            % COMPUTEINPUTS Decide throttle and brake for the current state
            %
            % The command policy is deliberately close to bang-bang:
            %   - brake hard if the car is above the latest-braking envelope
            %   - coast only at the local apex
            %   - otherwise use full throttle
            % Steering is a sine-shaped command through each corner segment,
            % with peak steering at the segment midpoint/apex.

            speed = max(state.speed, 0);
            s = state.s;
            [arcLen, curvature] = obj.getTrackGeometry();
            nPts = numel(curvature);

            idx = find(arcLen <= s, 1, 'last');
            if isempty(idx)
                idx = 1;
            end
            idx = max(1, min(idx, nPts));

            [maxLateralAccel, maxBrakeAccel] = obj.estimateAvailableAcceleration(state);
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
            nextTargetSpeed = profileSpeed(min(2, numel(profileSpeed)));
            speedTolerance = max(obj.throttleBand, obj.hysteresis * max(targetSpeed, 1));
            speedError = speed - targetSpeed;

            [apexDistance, atApex, inActiveCorner, afterApex] = obj.distanceToRelevantApex(idx, s);
            [steer, steeringUsageFrac] = obj.computeSteeringCommand(idx, s, state);
            longitudinalCommandScale = obj.computeLongitudinalCommandScale(steeringUsageFrac);

            throttle = 0;
            brake = 0;

            if inActiveCorner && afterApex
                throttle = 1.0;
            elseif atApex
                throttle = 0;
                brake = 0;
            elseif speedError > speedTolerance
                brake = obj.computeBrakeCommand(speedError);
            elseif nextTargetSpeed < targetSpeed - speedTolerance && ...
                    speed >= targetSpeed - speedTolerance
                brake = obj.minBrakeCommand;
            else
                throttle = 1.0;
            end

            % Do not coast just because the speed error is tiny; outside the
            % apex zone, choose either throttle or brake.
            if throttle == 0 && brake == 0 && abs(apexDistance) > obj.apexDistanceTol
                throttle = 1.0;
            end

            throttle = throttle * longitudinalCommandScale;
            brake = brake * longitudinalCommandScale;
        end
    end

    methods (Access = private)
        function obj = cacheTrackGeometry(obj)
            track = obj.vehicleManager.track;
            trackPts = track.getTrackPoints();

            dx = diff(trackPts(:,1));
            dy = diff(trackPts(:,2));
            obj.trackArcLen = [0; cumsum(sqrt(dx.^2 + dy.^2))];
            obj.trackCurvature = track.getCurvature();
            obj.trackCurvature = obj.trackCurvature(:);
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

        function [maxLateralAccel, maxBrakeAccel] = estimateAvailableAcceleration(obj, state)
            vm = obj.vehicleManager;
            aeroForces = vm.aero.computeForces(state);
            F_downforce = aeroForces.Fz_front + aeroForces.Fz_rear;
            W = vm.totalMass * 9.81;
            totalNormalLoad = W + F_downforce;

            peakMu = vm.tire.getPeakFriction(totalNormalLoad / 4);
            effectiveMu = min(max(peakMu, 0), max(state.mu, 0));
            maxTireAccel = effectiveMu * totalNormalLoad / vm.totalMass;
            maxLateralAccel = max(0.1, maxTireAccel * obj.corneringUsage);

            frontNormalLoad = max(W * vm.staticFrontWeight + aeroForces.Fz_front, 0);
            rearNormalLoad = max(W * (1 - vm.staticFrontWeight) + aeroForces.Fz_rear, 0);
            frontMu = min(max(vm.tire.getPeakFriction(frontNormalLoad / 2), 0), max(state.mu, 0));
            rearMu = min(max(vm.tire.getPeakFriction(rearNormalLoad / 2), 0), max(state.mu, 0));
            brakeBiasFront = max(0, min(1, vm.brakeBiasFront));
            brakeBiasRear = 1 - brakeBiasFront;
            brakeGripLimit = inf;
            if brakeBiasFront > eps
                brakeGripLimit = min(brakeGripLimit, frontMu * frontNormalLoad / brakeBiasFront);
            end
            if brakeBiasRear > eps
                brakeGripLimit = min(brakeGripLimit, rearMu * rearNormalLoad / brakeBiasRear);
            end

            maxBrakeForce = min(vm.brakeForceCoefficient * totalNormalLoad, brakeGripLimit);
            rollingResistance = 0.015 * totalNormalLoad;
            brakeLimitedAccel = ...
                (maxBrakeForce + aeroForces.F_drag + rollingResistance) / vm.totalMass;

            maxBrakeAccel = min(maxTireAccel, brakeLimitedAccel) * obj.brakingUsage;
            maxBrakeAccel = max(maxBrakeAccel, 0.1);
        end

        function lookAheadDist = computeLookaheadDistance(obj, speed, maxBrakeAccel)
            brakeDistance = speed^2 / (2 * maxBrakeAccel);
            lookAheadDist = max([ ...
                obj.minLookaheadDist, ...
                speed * obj.lookaheadTime, ...
                brakeDistance * obj.brakingLookahead + obj.minLookaheadDist]);
        end

        function profileSpeed = computeBackwardSpeedProfile(obj, profileS, curvature, maxLateralAccel, maxBrakeAccel)
            vm = obj.vehicleManager;
            profileSpeed = obj.computeCornerSpeedLimit(curvature, maxLateralAccel);
            profileSpeed = min(profileSpeed, vm.maxSpeed);

            for i = numel(profileSpeed)-1:-1:1
                ds = max(profileS(i+1) - profileS(i), 0.001);
                reachableSpeed = sqrt(profileSpeed(i+1)^2 + 2 * maxBrakeAccel * ds);
                profileSpeed(i) = min(profileSpeed(i), reachableSpeed);
            end
        end

        function speedLimit = computeCornerSpeedLimit(obj, curvature, maxLateralAccel)
            vm = obj.vehicleManager;
            absKappa = abs(curvature(:));
            speedLimit = vm.maxSpeed * ones(size(absKappa));

            cornerIdx = absKappa > obj.curvatureTol;
            speedLimit(cornerIdx) = sqrt(maxLateralAccel ./ absKappa(cornerIdx));
            speedLimit = min(speedLimit, vm.maxSpeed);
        end

        function brake = computeBrakeCommand(obj, speedError)
            brake = speedError / max(obj.brakeBlendSpeed, eps);
            brake = max(obj.minBrakeCommand, brake);
            brake = max(0, min(1, brake));
        end

        function [apexDistance, atApex, inActiveCorner, afterApex] = distanceToRelevantApex(obj, idx, s)
            [arcLen, curvature] = obj.getTrackGeometry();
            absKappa = abs(curvature);
            nPts = numel(curvature);
            apexDistance = inf;
            atApex = false;
            inActiveCorner = false;
            afterApex = false;

            if idx > nPts || all(absKappa <= obj.curvatureTol)
                return;
            end

            [segmentStart, segmentEnd, ~, found] = obj.findCornerSegment(idx);
            if ~found
                return;
            end

            apexS = obj.computeApexS(arcLen, segmentStart, segmentEnd);
            apexDistance = apexS - s;
            inActiveCorner = idx >= segmentStart && idx <= segmentEnd && ...
                absKappa(idx) > obj.curvatureTol;
            afterApex = inActiveCorner && s >= apexS;
            atApex = abs(apexDistance) <= obj.apexDistanceTol && ...
                inActiveCorner;
        end

        function [steer, steeringUsageFrac] = computeSteeringCommand(obj, idx, s, state)
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
            phase = max(0, min(1, phase));

            apexPhaseClamped = obj.getClampedApexPhase();
            if phase <= apexPhaseClamped
                steeringUsageFrac = sin((pi / 2) * phase / apexPhaseClamped);
            else
                steeringUsageFrac = sin((pi / 2) * (1 - phase) / (1 - apexPhaseClamped));
            end
            peakKappa = max(abs(curvature(segmentStart:segmentEnd)));
            peakSteer = atan(obj.vehicleManager.wheelbase * peakKappa) * obj.steeringUsage;
            peakSteer = min(obj.maxSteeringAngle, peakSteer);

            steer = turnSign * peakSteer * steeringUsageFrac;

            targetYawRate = state.speed * curvature(idx);
            yawRateError = targetYawRate - state.yawRate;
            steer = steer + obj.yawRateFeedbackGain * yawRateError ...
                - obj.sideslipFeedbackGain * state.sideslipAngle;
            steer = max(-obj.maxSteeringAngle, min(obj.maxSteeringAngle, steer));
        end

        function scale = computeLongitudinalCommandScale(obj, steeringUsageFrac)
            lateralUse = max(0, min(1, abs(steeringUsageFrac)));
            ellipseScale = sqrt(max(0, 1 - lateralUse^2));
            reserve = max(0, min(1, obj.minLongitudinalCommandScale));
            scale = reserve + (1 - reserve) * ellipseScale;
        end

        function [segmentStart, segmentEnd, turnSign, found] = findCornerSegment(obj, idx)
            [~, curvature] = obj.getTrackGeometry();
            absKappa = abs(curvature);
            nPts = numel(curvature);
            segmentStart = 1;
            segmentEnd = 1;
            turnSign = 0;
            found = false;

            if idx > nPts || all(absKappa <= obj.curvatureTol)
                return;
            end

            if absKappa(idx) > obj.curvatureTol
                segmentStart = idx;
                turnSign = sign(curvature(idx));
                while segmentStart > 1 && ...
                        absKappa(segmentStart - 1) > obj.curvatureTol && ...
                        sign(curvature(segmentStart - 1)) == turnSign
                    segmentStart = segmentStart - 1;
                end
            else
                nextCornerOffset = find(absKappa(idx:end) > obj.curvatureTol, 1, 'first');
                if isempty(nextCornerOffset)
                    return;
                end
                segmentStart = idx + nextCornerOffset - 1;
                turnSign = sign(curvature(segmentStart));
            end

            segmentEnd = segmentStart;
            while segmentEnd < nPts && ...
                    absKappa(segmentEnd + 1) > obj.curvatureTol && ...
                    sign(curvature(segmentEnd + 1)) == turnSign
                segmentEnd = segmentEnd + 1;
            end

            found = true;
        end

        function apexS = computeApexS(obj, arcLen, segmentStart, segmentEnd)
            apexPhaseClamped = obj.getClampedApexPhase();
            segmentStartS = arcLen(segmentStart);
            segmentEndS = arcLen(segmentEnd);
            apexS = segmentStartS + apexPhaseClamped * (segmentEndS - segmentStartS);
        end

        function apexPhaseClamped = getClampedApexPhase(obj)
            apexPhaseClamped = max(0.05, min(0.95, obj.apexPhase));
        end
    end
end
