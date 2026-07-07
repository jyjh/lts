classdef CorrelationStateInitializer
    % CORRELATIONSTATEINITIALIZER Builds lts.simulation.VehicleState from imported telemetry.
    %
    % Replays should start from measured speed/yaw/position when available;
    % otherwise they fall back to a neutral free-space state. This keeps
    % correlation runs from inheriting synthetic track-start assumptions.

    methods (Static)
        function state = fromReplayProfile(profile, track, vehicleManager, varargin)
            parser = inputParser;
            parser.addParameter('UseLoggedPosition', true, @(x) islogical(x) || isnumeric(x));
            parser.addParameter('UseLoggedYawRate', true, @(x) islogical(x) || isnumeric(x));
            parser.addParameter('UseLoggedWheelSpeeds', true, @(x) islogical(x) || isnumeric(x));
            parser.parse(varargin{:});

            if ~isa(profile, 'lts.correlation.CorrelationReplayProfile')
                profile = lts.correlation.CorrelationReplayProfile.fromCsv(profile);
            end

            speed = profile.speed(1);
            throttle = profile.throttle(1);
            brake = profile.brake(1);
            steer = profile.steer(1);

            [vx, vy, speed] = lts.correlation.CorrelationStateInitializer.initialVelocity( ...
                profile, speed, vehicleManager, logical(parser.Results.UseLoggedYawRate));

            yaw = 0;
            if profile.hasYaw()
                yaw = profile.yaw(1);
            end

            x = 0;
            y = 0;
            if profile.hasPosition() && logical(parser.Results.UseLoggedPosition)
                x = profile.x(1);
                y = profile.y(1);
            end

            state = lts.simulation.VehicleState( ...
                's', 0, ...
                'speed', speed, ...
                'vx', vx, ...
                'vy', vy, ...
                'x', x, ...
                'y', y, ...
                'yaw', yaw, ...
                'throttle', throttle, ...
                'brake', brake, ...
                'steer', steer);

            if logical(parser.Results.UseLoggedYawRate) && ...
                    ~isempty(profile.yawRate) && isfinite(profile.yawRate(1))
                state.yawRate = profile.yawRate(1);
            end

            if nargin >= 3 && ~isempty(vehicleManager)
                state.vehicleManager = vehicleManager;
                if logical(parser.Results.UseLoggedWheelSpeeds)
                    lts.correlation.CorrelationStateInitializer.seedWheelSpeeds( ...
                        profile, vehicleManager, speed);
                end
                state = lts.correlation.CorrelationStateInitializer.seedTransientCorneringState( ...
                    profile, vehicleManager, state);
            end
        end
    end

    methods (Static, Access = private)
        function [vx, vy, speed] = initialVelocity(profile, speed, vehicleManager, useLoggedYawRate)
            speed = max(0, speed);
            if profile.hasVelocity()
                vx = profile.vx(1);
                vy = profile.vy(1);
                speed = hypot(vx, vy);
                return;
            end

            if profile.hasBodySlip()
                beta = profile.bodySlip(1);
                vx = speed * cos(beta);
                vy = speed * sin(beta);
                return;
            end

            if useLoggedYawRate && ~isempty(profile.yawRate) && ...
                    isfinite(profile.yawRate(1))
                rearArm = lts.correlation.CorrelationStateInitializer.rearAxleArm(vehicleManager);
                vyEstimate = profile.yawRate(1) * rearArm;
                maxVy = 0.30 * max(speed, eps);
                vyEstimate = max(-maxVy, min(maxVy, vyEstimate));
                if isfinite(vyEstimate) && abs(vyEstimate) > eps
                    vy = vyEstimate;
                    vx = sqrt(max(speed^2 - vy^2, 0));
                    return;
                end
            end

            vx = speed;
            vy = 0;
        end

        function rearArm = rearAxleArm(vehicleManager)
            rearArm = NaN;
            if nargin < 1 || isempty(vehicleManager)
                return;
            end

            if isobject(vehicleManager) && ...
                    isprop(vehicleManager, 'wheelbase') && ...
                    isprop(vehicleManager, 'staticFrontWeight')
                rearArm = vehicleManager.wheelbase * vehicleManager.staticFrontWeight;
            elseif isstruct(vehicleManager) && ...
                    isfield(vehicleManager, 'wheelbase') && ...
                    isfield(vehicleManager, 'staticFrontWeight')
                rearArm = vehicleManager.wheelbase * vehicleManager.staticFrontWeight;
            end

            if ~isfinite(rearArm) || rearArm <= 0
                rearArm = 0;
            end
        end

        function seedWheelSpeeds(profile, vehicleManager, vehicleSpeed)
            if ~profile.hasInitialWheelSpeeds() || isempty(vehicleManager) || ...
                    isempty(vehicleManager.tire)
                return;
            end

            wheelSpeeds = profile.initialWheelSpeeds();
            corners = {'FL', 'FR', 'RL', 'RR'};
            values = [wheelSpeeds.FL, wheelSpeeds.FR, ...
                wheelSpeeds.RL, wheelSpeeds.RR];
            values = lts.correlation.CorrelationStateInitializer.rejectMovingZeroWheelSpeeds( ...
                values, vehicleSpeed);
            values = lts.correlation.CorrelationStateInitializer.estimateMissingWheelSpeeds( ...
                values, profile, vehicleManager);
            valid = isfinite(values) & values >= 0;
            fallbackSpeed = max(vehicleSpeed, 0);
            if any(valid)
                fallbackSpeed = median(values(valid));
            end

            for i = 1:numel(corners)
                cornerName = corners{i};
                wheelSpeed = values(i);
                if ~isfinite(wheelSpeed) || wheelSpeed < 0
                    wheelSpeed = fallbackSpeed;
                end

                cornerState = vehicleManager.tire.(cornerName);
                cornerState.angularVelocity = max(wheelSpeed, 0) / ...
                    max(cornerState.wheelRadius, eps);
            end

            if ~isempty(vehicleManager.powertrain) && ...
                    ismethod(vehicleManager.powertrain, 'updateStateFromDrivenWheels')
                vehicleManager.powertrain.updateStateFromDrivenWheels( ...
                    [vehicleManager.tire.RL.angularVelocity, ...
                    vehicleManager.tire.RR.angularVelocity]);
            end
        end

        function values = rejectMovingZeroWheelSpeeds(values, vehicleSpeed)
            movingThreshold = 2.0;
            stuckThreshold = 0.25;
            finiteValues = values(isfinite(values));
            maxWheelSpeed = 0;
            if ~isempty(finiteValues)
                maxWheelSpeed = max(finiteValues);
            end
            if max(vehicleSpeed, maxWheelSpeed) <= movingThreshold
                return;
            end

            for i = 1:numel(values)
                if ~isfinite(values(i)) || abs(values(i)) > stuckThreshold
                    continue;
                end

                others = values;
                others(i) = NaN;
                if any(isfinite(others) & others > movingThreshold)
                    values(i) = NaN;
                end
            end
        end

        function values = estimateMissingWheelSpeeds(values, profile, vehicleManager)
            if isempty(profile.yawRate) || ~isfinite(profile.yawRate(1)) || ...
                    isempty(vehicleManager) || ~isprop(vehicleManager, 'trackWidth') || ...
                    ~isfinite(vehicleManager.trackWidth) || vehicleManager.trackWidth <= 0
                return;
            end

            yawRate = profile.yawRate(1);
            trackWidth = vehicleManager.trackWidth;
            % Wheel-speed sign convention: right side speed is left side
            % speed plus yawRate * trackWidth for positive yaw.
            values = lts.correlation.CorrelationStateInitializer.estimateAxlePair( ...
                values, 1, 2, yawRate, trackWidth);
            values = lts.correlation.CorrelationStateInitializer.estimateAxlePair( ...
                values, 3, 4, yawRate, trackWidth);
        end

        function values = estimateAxlePair(values, leftIdx, rightIdx, yawRate, trackWidth)
            left = values(leftIdx);
            right = values(rightIdx);
            if ~isfinite(left) && isfinite(right)
                values(leftIdx) = max(0, right - yawRate * trackWidth);
            elseif isfinite(left) && ~isfinite(right)
                values(rightIdx) = max(0, left + yawRate * trackWidth);
            end
        end

        function state = seedTransientCorneringState(profile, vehicleManager, state)
            % Correlation replays can start mid-corner. Seed the internal
            % tire/chassis transients from the first logged sample so the
            % simulation does not build lateral force and roll from rest.
            if isempty(vehicleManager)
                return;
            end

            [bodyAy, frontAy, rearAy] = ...
                lts.correlation.CorrelationStateInitializer.initialLateralAccelerations( ...
                profile, state);
            if isfinite(bodyAy)
                state.ay = bodyAy;
            end
            if isfinite(frontAy)
                state.frontAxleAy = frontAy;
            end
            if isfinite(rearAy)
                state.rearAxleAy = rearAy;
            end

            state = lts.correlation.CorrelationStateInitializer.seedChassisRollState( ...
                vehicleManager, state, bodyAy, frontAy, rearAy);
            lts.correlation.CorrelationStateInitializer.seedTireRelaxationState( ...
                vehicleManager, state);
        end

        function [bodyAy, frontAy, rearAy] = initialLateralAccelerations(profile, state)
            g = 9.80665;
            bodyAy = lts.correlation.CorrelationStateInitializer.firstFinite(profile.latAccelG) * g;
            frontAy = lts.correlation.CorrelationStateInitializer.firstFinite(profile.frontLatAccelG) * g;
            rearAy = lts.correlation.CorrelationStateInitializer.firstFinite(profile.rearLatAccelG) * g;

            if ~isfinite(bodyAy) && isfinite(state.speed) && ...
                    isfinite(state.yawRate)
                bodyAy = state.speed * state.yawRate;
            end
            if ~isfinite(frontAy)
                frontAy = bodyAy;
            end
            if ~isfinite(rearAy)
                rearAy = bodyAy;
            end
        end

        function value = firstFinite(values)
            value = NaN;
            if isempty(values)
                return;
            end
            idx = find(isfinite(values), 1, 'first');
            if ~isempty(idx)
                value = values(idx);
            end
        end

        function state = seedChassisRollState(vehicleManager, state, bodyAy, frontAy, rearAy)
            if ~isfinite(bodyAy) || isempty(vehicleManager) || ...
                    isempty(vehicleManager.chassis) || isempty(vehicleManager.suspension) || ...
                    ~isprop(vehicleManager.chassis, 'state') || ...
                    ~ismethod(vehicleManager.suspension, 'getAxleRollStiffness')
                return;
            end

            [KwF, KwR] = vehicleManager.suspension.getAxleRollStiffness();
            KrollF = KwF * vehicleManager.trackWidth^2 / 2;
            KrollR = KwR * vehicleManager.trackWidth^2 / 2;
            if KrollF <= 0 && KrollR <= 0
                return;
            end

            chassis = vehicleManager.chassis;
            frontMassFrac = vehicleManager.staticFrontWeight;
            rearMassFrac = 1 - frontMassFrac;
            rollMomentF = chassis.sprungMass * frontMassFrac * frontAy * vehicleManager.cgHeight;
            rollMomentR = chassis.sprungMass * rearMassFrac * rearAy * vehicleManager.cgHeight;

            torsion = chassis.torsionalRigidity;
            if ~isfinite(torsion)
                torsion = 1e9;
            end
            A = [KrollF + torsion, -torsion; -torsion, KrollR + torsion];
            if rcond(A) < 1e-12
                return;
            end
            phi = A \ [rollMomentF; rollMomentR];
            maxRoll = deg2rad(10);
            phi = max(-maxRoll, min(maxRoll, phi));

            chassis.state.frontRollAngle = phi(1);
            chassis.state.rearRollAngle = phi(2);
            chassis.state.rollAngle = 0.5 * (phi(1) + phi(2));
            chassis.state.frontRollRate = 0;
            chassis.state.rearRollRate = 0;
            chassis.state.rollRate = 0;
            chassis.state.lateralLoadTransfer = ...
                vehicleManager.totalMass * bodyAy * vehicleManager.cgHeight / ...
                max(vehicleManager.trackWidth, eps);
            chassis.state.updateCornerKinematics( ...
                vehicleManager.wheelbase, vehicleManager.trackWidth, ...
                vehicleManager.staticFrontWeight);

            vehicleManager.suspension.computeCornerLoadsFromChassis( ...
                chassis, state.steer, 0);

            state.rollAngle = chassis.state.rollAngle;
            state.frontRollAngle = chassis.state.frontRollAngle;
            state.rearRollAngle = chassis.state.rearRollAngle;
            state.rollRate = chassis.state.rollRate;
            state.frontRollRate = chassis.state.frontRollRate;
            state.rearRollRate = chassis.state.rearRollRate;
            state.twistAngle = chassis.getTwistAngle();
            state.twistRate = chassis.getTwistRate();
            state.rideHeight = -chassis.getHeave();
        end

        function seedTireRelaxationState(vehicleManager, state)
            if isempty(vehicleManager) || isempty(vehicleManager.tire)
                return;
            end

            if ~isempty(vehicleManager.suspension) && ...
                    ismethod(vehicleManager.suspension, 'getCornerKinematics')
                kin = vehicleManager.suspension.getCornerKinematics();
            else
                kin = lts.correlation.CorrelationStateInitializer.defaultCornerKinematics( ...
                    vehicleManager, state.steer);
            end

            corners = {'FL', 'FR', 'RL', 'RR'};
            slipAngles = zeros(4, 1);
            slipRatios = zeros(4, 1);
            longSpeeds = zeros(4, 1);
            cambers = zeros(4, 1);
            loads = zeros(4, 1);
            for i = 1:numel(corners)
                corner = corners{i};
                cornerKin = kin.(corner);
                tireState = vehicleManager.tire.(corner);
                [alpha, kappa, longSpeed] = ...
                    lts.correlation.CorrelationStateInitializer.initialCornerSlip( ...
                    tireState, cornerKin, state);
                tireState.slipAngle = alpha;
                tireState.ssSlipAngle = alpha;
                tireState.slipRatio = kappa;
                tireState.ssSlipRatio = kappa;
                slipAngles(i) = alpha;
                slipRatios(i) = kappa;
                longSpeeds(i) = longSpeed;
                cambers(i) = lts.correlation.CorrelationStateInitializer.fieldOrDefault( ...
                    cornerKin, 'camberAngle', 0);
                loads(i) = tireState.normalForce;
            end

            if ismethod(vehicleManager.tire, 'updateAllCorners') && all(isfinite(loads)) && any(loads > 0)
                vehicleManager.tire.updateAllCorners( ...
                    loads(1), loads(2), loads(3), loads(4), ...
                    slipAngles(1), slipAngles(2), slipAngles(3), slipAngles(4), ...
                    slipRatios(1), slipRatios(2), slipRatios(3), slipRatios(4), ...
                    cambers(1), cambers(2), cambers(3), cambers(4), ...
                    0, longSpeeds, state.mu, true, 'steady');
                for i = 1:numel(corners)
                    tireState = vehicleManager.tire.(corners{i});
                    tireState.slipAngle = slipAngles(i);
                    tireState.ssSlipAngle = slipAngles(i);
                    tireState.slipRatio = slipRatios(i);
                    tireState.ssSlipRatio = slipRatios(i);
                end
            end
        end

        function [alpha, kappa, longSpeed] = initialCornerSlip(tireState, cornerKin, state)
            xPos = lts.correlation.CorrelationStateInitializer.fieldOrDefault( ...
                cornerKin, 'xPosition', 0);
            yPos = lts.correlation.CorrelationStateInitializer.fieldOrDefault( ...
                cornerKin, 'yPosition', 0);
            steerAngle = lts.correlation.CorrelationStateInitializer.fieldOrDefault( ...
                cornerKin, 'steerAngle', 0);
            toeAngle = lts.correlation.CorrelationStateInitializer.fieldOrDefault( ...
                cornerKin, 'toeAngle', 0);

            vxCorner = state.vx - state.yawRate * yPos;
            vyCorner = state.vy + state.yawRate * xPos;
            wheelHeading = steerAngle + toeAngle;
            longSpeed = vxCorner * cos(wheelHeading) + vyCorner * sin(wheelHeading);
            latSpeed = -vxCorner * sin(wheelHeading) + vyCorner * cos(wheelHeading);
            alpha = atan2(-latSpeed, max(abs(longSpeed), 0.1));
            alpha = max(-0.3, min(0.3, alpha));

            wheelSpeed = tireState.angularVelocity * tireState.wheelRadius;
            denom = max(abs(wheelSpeed), abs(longSpeed));
            kappa = (wheelSpeed - longSpeed) / max(denom, 1.0);
            kappa = max(-1, min(1, kappa));
        end

        function kin = defaultCornerKinematics(vehicleManager, steer)
            frontArm = vehicleManager.wheelbase * (1 - vehicleManager.staticFrontWeight);
            rearArm = vehicleManager.wheelbase * vehicleManager.staticFrontWeight;
            halfTrack = vehicleManager.trackWidth / 2;
            kin.FL = struct('camberAngle', 0, 'toeAngle', 0, ...
                'steerAngle', steer, 'xPosition', frontArm, 'yPosition', halfTrack);
            kin.FR = struct('camberAngle', 0, 'toeAngle', 0, ...
                'steerAngle', steer, 'xPosition', frontArm, 'yPosition', -halfTrack);
            kin.RL = struct('camberAngle', 0, 'toeAngle', 0, ...
                'steerAngle', 0, 'xPosition', -rearArm, 'yPosition', halfTrack);
            kin.RR = struct('camberAngle', 0, 'toeAngle', 0, ...
                'steerAngle', 0, 'xPosition', -rearArm, 'yPosition', -halfTrack);
        end

        function value = fieldOrDefault(s, fieldName, defaultValue)
            if isstruct(s) && isfield(s, fieldName) && isfinite(s.(fieldName))
                value = s.(fieldName);
            else
                value = defaultValue;
            end
        end
    end
end
