classdef CorrelationStateInitializer
    % Build simulation state from imported telemetry.

    methods (Static)
        function state = fromReplayProfile(profile, ~, vehicleManager, varargin)
            parser = inputParser;
            flags = {'UseLoggedPosition', true; 'UseLoggedYawRate', true; ...
                'UseLoggedWheelSpeeds', true; ...
                'UseLoggedDrivenWheelCarrierSpeed', false; ...
                'UseLoggedTransientState', true};
            for i = 1:size(flags, 1)
                parser.addParameter(flags{i, 1}, flags{i, 2}, ...
                    @(x) islogical(x) || isnumeric(x));
            end
            parser.addParameter('InitialTransientWindowS', 0, ...
                @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0);
            parser.parse(varargin{:});
            initialTransientWindowS = double(parser.Results.InitialTransientWindowS);

            if ~isa(profile, 'lts.correlation.CorrelationReplayProfile')
                profile = lts.correlation.CorrelationReplayProfile.fromCsv(profile);
            end

            speed = profile.speed(1);
            throttle = profile.throttle(1);
            brake = profile.brake(1);
            steer = profile.steer(1);

            [vx, vy, speed] = lts.correlation.CorrelationStateInitializer.initialVelocity( ...
                profile, speed, vehicleManager, logical(parser.Results.UseLoggedYawRate), ...
                initialTransientWindowS);

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

            yawRateSeed = lts.correlation.CorrelationStateInitializer.initialBoundaryValue( ...
                profile, profile.yawRate, initialTransientWindowS);
            if logical(parser.Results.UseLoggedYawRate) && isfinite(yawRateSeed)
                state.yawRate = yawRateSeed;
            end

            if nargin >= 3 && ~isempty(vehicleManager)
                state.vehicleManager = vehicleManager;
                wheelSpeeds = [];
                if logical(parser.Results.UseLoggedWheelSpeeds)
                    wheelSpeeds = lts.correlation.CorrelationStateInitializer.initialWheelSpeedValues( ...
                        profile, vehicleManager, state, initialTransientWindowS);
                elseif logical(parser.Results.UseLoggedDrivenWheelCarrierSpeed)
                    wheelSpeeds = lts.correlation.CorrelationStateInitializer.initialDrivenCarrierWheelSpeeds( ...
                        profile, vehicleManager, state, initialTransientWindowS);
                end
                if logical(parser.Results.UseLoggedTransientState)
                    state = lts.correlation.CorrelationStateInitializer.seedTransientCorneringState( ...
                        profile, vehicleManager, state, wheelSpeeds, initialTransientWindowS);
                else
                    if isempty(wheelSpeeds)
                        lts.correlation.CorrelationStateInitializer.seedKinematicWheelSpeeds( ...
                            vehicleManager, state);
                    else
                        lts.correlation.CorrelationStateInitializer.seedWheelSpeeds( ...
                            profile, vehicleManager, state, wheelSpeeds);
                    end
                end
            end
        end
    end

    methods (Static, Access = private)
        function [vx, vy, speed] = initialVelocity( ...
                profile, speed, vehicleManager, useLoggedYawRate, initialTransientWindowS)
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

            yawRateSeed = lts.correlation.CorrelationStateInitializer.initialBoundaryValue( ...
                profile, profile.yawRate, initialTransientWindowS);
            if useLoggedYawRate && isfinite(yawRateSeed)
                rearArm = lts.correlation.CorrelationStateInitializer.rearAxleArm(vehicleManager);
                vyEstimate = yawRateSeed * rearArm;
                maxVy = 0.30 * max(speed, eps);
                vyEstimate = lts.util.clamp(vyEstimate, -maxVy, maxVy);
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

        function values = initialWheelSpeedValues( ...
                profile, vehicleManager, state, initialTransientWindowS)
            if nargin < 4
                initialTransientWindowS = 0;
            end
            values = [];
            if ~profile.hasInitialWheelSpeeds() || isempty(vehicleManager) || ...
                    isempty(vehicleManager.tire)
                return;
            end

            wheelSpeeds = profile.initialWheelSpeeds();
            values = [wheelSpeeds.FL, wheelSpeeds.FR, ...
                wheelSpeeds.RL, wheelSpeeds.RR];
            values = lts.correlation.CorrelationStateInitializer.rejectMovingZeroWheelSpeeds( ...
                values, state.speed);
            yawRateSeed = lts.correlation.CorrelationStateInitializer.initialBoundaryValue( ...
                profile, profile.yawRate, initialTransientWindowS);
            values = lts.correlation.CorrelationStateInitializer.estimateMissingWheelSpeeds( ...
                values, yawRateSeed, vehicleManager);
            values = lts.correlation.CorrelationStateInitializer.fillMissingWheelSpeeds( ...
                values, state.speed);
        end

        function values = initialDrivenCarrierWheelSpeeds( ...
                profile, vehicleManager, state, initialTransientWindowS)
            values = [];
            if ~profile.hasMotorRpm() || isempty(vehicleManager) || ...
                    isempty(vehicleManager.tire) || isempty(vehicleManager.powertrain) || ...
                    ~ismethod(vehicleManager.powertrain, 'getTotalGearRatio')
                return;
            end

            ratio = vehicleManager.powertrain.getTotalGearRatio();
            motorRpm = lts.correlation.CorrelationStateInitializer.initialBoundaryValue( ...
                profile, profile.motorRpm, initialTransientWindowS);
            if ~isfinite(ratio) || ratio <= 0 || ~isfinite(motorRpm) || motorRpm < 0
                return;
            end

            kin = lts.correlation.CorrelationStateInitializer.cornerKinematics( ...
                vehicleManager, state.steer);
            corners = {'FL', 'FR', 'RL', 'RR'};
            values = zeros(1, 4);
            omega = zeros(1, 4);
            for i = 1:numel(corners)
                corner = corners{i};
                tireState = vehicleManager.tire.(corner);
                [~, ~, longSpeed] = ...
                    lts.correlation.CorrelationStateInitializer.initialCornerSlip( ...
                        tireState, kin.(corner), state);
                values(i) = max(longSpeed, 0);
                omega(i) = values(i) / max(tireState.wheelRadius, eps);
            end

            carrierOmega = motorRpm * 2 * pi / 60 / ratio;
            rearDeltaOmega = omega(4) - omega(3);
            omega(3) = max(0, carrierOmega - 0.5 * rearDeltaOmega);
            omega(4) = max(0, carrierOmega + 0.5 * rearDeltaOmega);
            values(3) = omega(3) * vehicleManager.tire.RL.wheelRadius;
            values(4) = omega(4) * vehicleManager.tire.RR.wheelRadius;
        end

        function seedWheelSpeeds(profile, vehicleManager, state, values)
            if nargin < 4
                values = lts.correlation.CorrelationStateInitializer.initialWheelSpeedValues( ...
                    profile, vehicleManager, state);
            end
            if isempty(values) || isempty(vehicleManager) || isempty(vehicleManager.tire)
                return;
            end

            corners = {'FL', 'FR', 'RL', 'RR'};
            for i = 1:numel(corners)
                cornerState = vehicleManager.tire.(corners{i});
                cornerState.angularVelocity = max(values(i), 0) / ...
                    max(cornerState.wheelRadius, eps);
            end

            if ~isempty(vehicleManager.powertrain) && ...
                    ismethod(vehicleManager.powertrain, 'updateStateFromDrivenWheels')
                vehicleManager.powertrain.updateStateFromDrivenWheels( ...
                    [vehicleManager.tire.RL.angularVelocity, ...
                    vehicleManager.tire.RR.angularVelocity]);
            end
        end

        function values = fillMissingWheelSpeeds(values, vehicleSpeed)
            valid = isfinite(values) & values >= 0;
            fallbackSpeed = max(vehicleSpeed, 0);
            if any(valid)
                fallbackSpeed = median(values(valid));
            end
            values(~valid) = fallbackSpeed;
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

            if any(values > movingThreshold)
                values(isfinite(values) & abs(values) <= stuckThreshold) = NaN;
            end
        end

        function values = estimateMissingWheelSpeeds(values, yawRate, vehicleManager)
            if ~isfinite(yawRate) || isempty(vehicleManager) || ...
                    ~isprop(vehicleManager, 'trackWidth') || ...
                    ~isfinite(vehicleManager.trackWidth) || vehicleManager.trackWidth <= 0
                return;
            end

            trackWidth = vehicleManager.trackWidth;
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

        function state = seedTransientCorneringState( ...
                profile, vehicleManager, state, wheelSpeeds, initialTransientWindowS)
            % Seed tire and chassis transients for mid-corner starts.
            if isempty(vehicleManager)
                return;
            end
            if nargin < 4
                wheelSpeeds = [];
            end
            if nargin < 5
                initialTransientWindowS = 0;
            end

            [bodyAy, frontAy, rearAy, yawAccel] = ...
                lts.correlation.CorrelationStateInitializer.initialLateralAccelerations( ...
                profile, vehicleManager, state, initialTransientWindowS);
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
            state = lts.correlation.CorrelationStateInitializer.seedSideslipFromAxleAccelerations( ...
                vehicleManager, state, wheelSpeeds, bodyAy, yawAccel);
            if isempty(wheelSpeeds)
                lts.correlation.CorrelationStateInitializer.seedKinematicWheelSpeeds( ...
                    vehicleManager, state);
            else
                lts.correlation.CorrelationStateInitializer.seedWheelSpeeds( ...
                    profile, vehicleManager, state, wheelSpeeds);
            end
            lts.correlation.CorrelationStateInitializer.seedTireRelaxationState( ...
                vehicleManager, state, bodyAy, yawAccel);
        end

        function [bodyAy, frontAy, rearAy, yawAccel] = initialLateralAccelerations( ...
                profile, vehicleManager, state, initialTransientWindowS)
            g = 9.80665;
            bodyAy = lts.correlation.CorrelationStateInitializer.initialBoundaryValue( ...
                profile, profile.latAccelG, initialTransientWindowS) * g;
            frontAy = lts.correlation.CorrelationStateInitializer.initialBoundaryValue( ...
                profile, profile.frontLatAccelG, initialTransientWindowS) * g;
            rearAy = lts.correlation.CorrelationStateInitializer.initialBoundaryValue( ...
                profile, profile.rearLatAccelG, initialTransientWindowS) * g;

            if isfinite(frontAy) && isfinite(rearAy) && ...
                    ~isempty(vehicleManager) && isfinite(vehicleManager.wheelbase) && ...
                    vehicleManager.wheelbase > 0
                [frontArm, rearArm] = ...
                    lts.correlation.CorrelationStateInitializer.axleArms(vehicleManager);
                bodyAy = (rearArm * frontAy + frontArm * rearAy) / ...
                    vehicleManager.wheelbase;
                yawAccel = (frontAy - rearAy) / vehicleManager.wheelbase;
            else
                yawAccel = lts.correlation.CorrelationStateInitializer.initialBoundarySlope( ...
                    profile, profile.yawRate, initialTransientWindowS);
            end

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
            if ~isfinite(yawAccel) && isfinite(frontAy) && isfinite(rearAy) && ...
                    ~isempty(vehicleManager) && isfinite(vehicleManager.wheelbase) && ...
                    vehicleManager.wheelbase > 0
                yawAccel = (frontAy - rearAy) / vehicleManager.wheelbase;
            end
        end

        function value = firstFinite(values)
            idx = find(isfinite(values), 1, 'first');
            if isempty(idx)
                value = NaN;
            else
                value = values(idx);
            end
        end

        function value = initialBoundaryValue(profile, values, windowS)
            if nargin < 3 || isempty(windowS) || windowS <= 0
                value = lts.correlation.CorrelationStateInitializer.firstFinite(values);
                return;
            end

            value = NaN;
            if isempty(values)
                return;
            end

            values = double(values(:));
            mask = false(size(values));
            if ~isempty(profile.time) && numel(profile.time) == numel(values)
                time = profile.time(:);
                fitWindowS = min(windowS, 0.05);
                mask = isfinite(time) & time >= time(1) & ...
                    time <= time(1) + fitWindowS + eps(max(fitWindowS, 1));
            else
                mask(1) = true;
            end

            valid = mask & isfinite(values);
            windowValues = values(valid);
            if isempty(windowValues)
                value = lts.correlation.CorrelationStateInitializer.firstFinite(values);
            elseif nnz(valid) < 3
                value = windowValues(1);
            else
                time = profile.time(:);
                localTime = time(valid) - time(1);
                coeff = [ones(size(localTime)), localTime] \ windowValues;
                value = coeff(1);
            end
        end

        function slope = initialBoundarySlope(profile, values, windowS)
            slope = NaN;
            if nargin < 3 || isempty(windowS) || windowS <= 0 || ...
                    isempty(values) || isempty(profile.time) || ...
                    numel(profile.time) ~= numel(values)
                return;
            end
            time = double(profile.time(:));
            values = double(values(:));
            fitWindowS = min(windowS, 0.05);
            valid = isfinite(time) & isfinite(values) & time >= time(1) & ...
                time <= time(1) + fitWindowS + eps(max(fitWindowS, 1));
            if nnz(valid) < 3
                return;
            end
            localTime = time(valid) - time(1);
            coeff = [ones(size(localTime)), localTime] \ values(valid);
            slope = coeff(2);
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
            phi = lts.util.clamp(phi, -maxRoll, maxRoll);

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

            if ismethod(vehicleManager.suspension, ...
                    'initializeCornerLoadsFromChassis')
                vehicleManager.suspension.initializeCornerLoadsFromChassis( ...
                    chassis, state.steer);
            else
                vehicleManager.suspension.computeCornerLoadsFromChassis( ...
                    chassis, state.steer, 0);
            end

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

        function state = seedSideslipFromAxleAccelerations( ...
                vehicleManager, state, wheelSpeeds, bodyAy, yawAccel)
            if isempty(vehicleManager) || isempty(vehicleManager.tire) || ...
                    ~isfinite(bodyAy) || ~isfinite(yawAccel) || ...
                    ~isfinite(state.speed) || state.speed <= 2 || ...
                    ~isfinite(state.yawRate) || abs(state.yawRate) < 1e-6
                return;
            end

            beta0 = atan2(state.vy, max(state.vx, 0.1));
            maxBeta = 0.25;
            span = deg2rad(10);
            candidates = linspace( ...
                max(-maxBeta, beta0 - span), ...
                min(maxBeta, beta0 + span), 31);

            bestScore = inf;
            bestBeta = beta0;
            for i = 1:numel(candidates)
                estimate = lts.correlation.CorrelationStateInitializer.estimateInitialAccelerations( ...
                    vehicleManager, state, candidates(i), wheelSpeeds);
                if ~all(isfinite([estimate.ay, estimate.yawAccel]))
                    continue;
                end
                ayErrorG = (estimate.ay - bodyAy) / 9.80665;
                yawErrorG = (estimate.yawAccel - yawAccel) * ...
                    vehicleManager.wheelbase / 9.80665;
                score = ayErrorG^2 + yawErrorG^2;
                if score < bestScore
                    bestScore = score;
                    bestBeta = candidates(i);
                end
            end

            if isfinite(bestScore)
                state = lts.correlation.CorrelationStateInitializer.setStateBodySlip( ...
                    state, bestBeta);
            end
        end

        function estimate = estimateInitialAccelerations( ...
                vehicleManager, state, beta, wheelSpeeds)
            candidate = lts.correlation.CorrelationStateInitializer.setStateBodySlip( ...
                state, beta);
            values = wheelSpeeds;
            useProvidedWheelSpeeds = ~isempty(values);
            if isempty(values)
                values = NaN(1, 4);
            end

            kin = lts.correlation.CorrelationStateInitializer.cornerKinematics( ...
                vehicleManager, candidate.steer);
            corners = {'FL', 'FR', 'RL', 'RR'};
            loads = zeros(4, 1);
            slipAngles = zeros(4, 1);
            slipRatios = zeros(4, 1);
            longSpeeds = zeros(4, 1);
            cambers = zeros(4, 1);
            for i = 1:numel(corners)
                corner = corners{i};
                tireState = vehicleManager.tire.(corner);
                [alpha, ~, longSpeed] = ...
                    lts.correlation.CorrelationStateInitializer.initialCornerSlip( ...
                    tireState, kin.(corner), candidate);
                wheelSpeed = NaN;
                if numel(values) >= i
                    wheelSpeed = values(i);
                end
                if ~isfinite(wheelSpeed) && useProvidedWheelSpeeds
                    wheelSpeed = tireState.angularVelocity * tireState.wheelRadius;
                end
                if ~isfinite(wheelSpeed)
                    wheelSpeed = longSpeed;
                end
                kappa = lts.correlation.CorrelationStateInitializer.slipRatioFromWheelSpeed( ...
                    wheelSpeed, longSpeed);
                loads(i) = lts.correlation.CorrelationStateInitializer.cornerNormalLoad( ...
                    vehicleManager, corner, tireState);
                slipAngles(i) = alpha;
                slipRatios(i) = kappa;
                longSpeeds(i) = longSpeed;
                cambers(i) = lts.correlation.CorrelationStateInitializer.fieldOrDefault( ...
                    kin.(corner), 'camberAngle', 0);
            end

            estimate = struct('ay', NaN, 'yawAccel', NaN, ...
                'frontAy', NaN, 'rearAy', NaN);
            if ~ismethod(vehicleManager.tire, 'updateAllCorners') || ...
                    ~all(isfinite(loads)) || ~any(loads > 0)
                return;
            end

            vehicleManager.tire.updateAllCorners( ...
                loads(1), loads(2), loads(3), loads(4), ...
                slipAngles(1), slipAngles(2), slipAngles(3), slipAngles(4), ...
                slipRatios(1), slipRatios(2), slipRatios(3), slipRatios(4), ...
                cambers(1), cambers(2), cambers(3), cambers(4), ...
                0, longSpeeds, true, 'steady');

            [sumFyBody, yawMoment] = ...
                lts.correlation.CorrelationStateInitializer.planarLateralForceAndMoment( ...
                vehicleManager, kin, corners);
            ay = sumFyBody / max(vehicleManager.totalMass, eps);
            yawAccel = yawMoment / max(vehicleManager.yawInertia, eps);
            [frontArm, rearArm] = ...
                lts.correlation.CorrelationStateInitializer.axleArms(vehicleManager);
            estimate.ay = ay;
            estimate.yawAccel = yawAccel;
            estimate.frontAy = ay + yawAccel * frontArm;
            estimate.rearAy = ay - yawAccel * rearArm;
        end

        function seedKinematicWheelSpeeds(vehicleManager, state)
            if isempty(vehicleManager) || isempty(vehicleManager.tire)
                return;
            end
            kin = lts.correlation.CorrelationStateInitializer.cornerKinematics( ...
                vehicleManager, state.steer);
            corners = {'FL', 'FR', 'RL', 'RR'};
            for i = 1:numel(corners)
                corner = corners{i};
                tireState = vehicleManager.tire.(corner);
                [~, ~, longSpeed] = ...
                    lts.correlation.CorrelationStateInitializer.initialCornerSlip( ...
                    tireState, kin.(corner), state);
                tireState.angularVelocity = max(longSpeed, 0) / ...
                    max(tireState.wheelRadius, eps);
            end
            if ~isempty(vehicleManager.powertrain) && ...
                    ismethod(vehicleManager.powertrain, 'updateStateFromDrivenWheels')
                vehicleManager.powertrain.updateStateFromDrivenWheels( ...
                    [vehicleManager.tire.RL.angularVelocity, ...
                    vehicleManager.tire.RR.angularVelocity]);
            end
        end

        function [sumFyBody, yawMoment] = planarLateralForceAndMoment( ...
                vehicleManager, kin, corners)
            sumFyBody = 0;
            yawMoment = 0;
            for i = 1:numel(corners)
                corner = corners{i};
                tireState = vehicleManager.tire.(corner);
                cornerKin = kin.(corner);
                wheelHeading = ...
                    lts.correlation.CorrelationStateInitializer.fieldOrDefault(cornerKin, 'steerAngle', 0) + ...
                    lts.correlation.CorrelationStateInitializer.fieldOrDefault(cornerKin, 'toeAngle', 0);
                FxBody = tireState.Fx * cos(wheelHeading) - tireState.Fy * sin(wheelHeading);
                FyBody = tireState.Fx * sin(wheelHeading) + tireState.Fy * cos(wheelHeading);
                xPos = lts.correlation.CorrelationStateInitializer.fieldOrDefault( ...
                    cornerKin, 'xPosition', 0);
                yPos = lts.correlation.CorrelationStateInitializer.fieldOrDefault( ...
                    cornerKin, 'yPosition', 0);
                sumFyBody = sumFyBody + FyBody;
                yawMoment = yawMoment + xPos * FyBody - yPos * FxBody + tireState.Mz;
            end
        end

        function state = setStateBodySlip(state, beta)
            speed = max(state.speed, 0);
            beta = lts.util.clamp(beta, -0.25, 0.25);
            state.vx = speed * cos(beta);
            state.vy = speed * sin(beta);
            state.bodySlipAngle = beta;
        end

        function seedTireRelaxationState(vehicleManager, state, targetAy, targetYawAccel)
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
                loads(i) = lts.correlation.CorrelationStateInitializer.cornerNormalLoad( ...
                    vehicleManager, corner, tireState);
            end

            if ismethod(vehicleManager.tire, 'updateAllCorners') && all(isfinite(loads)) && any(loads > 0)
                forceSlipAngles = slipAngles;
                if nargin >= 4 && isfinite(targetAy) && isfinite(targetYawAccel)
                    objective = @(offsets) ...
                        lts.correlation.CorrelationStateInitializer.transientSlipScore( ...
                        offsets, vehicleManager, kin, corners, loads, slipAngles, ...
                        slipRatios, cambers, longSpeeds, targetAy, targetYawAccel);
                    options = optimset('Display', 'off', 'MaxIter', 100, ...
                        'MaxFunEvals', 300, 'TolX', 1e-7, 'TolFun', 1e-8);
                    offsets = fminsearch(objective, [0, 0], options);
                    offsets = lts.util.clamp(offsets, -0.3, 0.3);
                    forceSlipAngles(1:2) = lts.util.clamp( ...
                        slipAngles(1:2) + offsets(1), -0.3, 0.3);
                    forceSlipAngles(3:4) = lts.util.clamp( ...
                        slipAngles(3:4) + offsets(2), -0.3, 0.3);
                end
                vehicleManager.tire.updateAllCorners( ...
                    loads(1), loads(2), loads(3), loads(4), ...
                    forceSlipAngles(1), forceSlipAngles(2), ...
                    forceSlipAngles(3), forceSlipAngles(4), ...
                    slipRatios(1), slipRatios(2), slipRatios(3), slipRatios(4), ...
                    cambers(1), cambers(2), cambers(3), cambers(4), ...
                    0, longSpeeds, true, 'steady');
                for i = 1:numel(corners)
                    tireState = vehicleManager.tire.(corners{i});
                    tireState.slipAngle = forceSlipAngles(i);
                    tireState.ssSlipAngle = slipAngles(i);
                    tireState.slipRatio = slipRatios(i);
                    tireState.ssSlipRatio = slipRatios(i);
                end
            end
        end

        function score = transientSlipScore(offsets, vehicleManager, kin, corners, ...
                loads, slipAngles, slipRatios, cambers, longSpeeds, ...
                targetAy, targetYawAccel)
            offsets = lts.util.clamp(offsets, -0.3, 0.3);
            candidate = slipAngles;
            candidate(1:2) = lts.util.clamp( ...
                candidate(1:2) + offsets(1), -0.3, 0.3);
            candidate(3:4) = lts.util.clamp( ...
                candidate(3:4) + offsets(2), -0.3, 0.3);
            vehicleManager.tire.updateAllCorners( ...
                loads(1), loads(2), loads(3), loads(4), ...
                candidate(1), candidate(2), candidate(3), candidate(4), ...
                slipRatios(1), slipRatios(2), slipRatios(3), slipRatios(4), ...
                cambers(1), cambers(2), cambers(3), cambers(4), ...
                0, longSpeeds, true, 'steady');
            [sumFyBody, yawMoment] = ...
                lts.correlation.CorrelationStateInitializer.planarLateralForceAndMoment( ...
                vehicleManager, kin, corners);
            ay = sumFyBody / max(vehicleManager.totalMass, eps);
            yawAccel = yawMoment / max(vehicleManager.yawInertia, eps);
            ayErrorG = (ay - targetAy) / 9.80665;
            yawErrorG = (yawAccel - targetYawAccel) * ...
                vehicleManager.wheelbase / 9.80665;
            score = ayErrorG^2 + yawErrorG^2;
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
            alpha = lts.util.clamp(alpha, -0.3, 0.3);

            wheelSpeed = tireState.angularVelocity * tireState.wheelRadius;
            kappa = lts.correlation.CorrelationStateInitializer.slipRatioFromWheelSpeed( ...
                wheelSpeed, longSpeed);
        end

        function load = cornerNormalLoad(vehicleManager, corner, tireState)
            load = NaN;
            if nargin >= 3 && ~isempty(tireState) && ...
                    isprop(tireState, 'normalForce') && isfinite(tireState.normalForce) && ...
                    tireState.normalForce > 0
                load = tireState.normalForce;
                return;
            end

            if isempty(vehicleManager) || isempty(vehicleManager.suspension)
                load = 0;
                return;
            end

            names = {'FL', 'FR', 'RL', 'RR'};
            fields = {'frontLeft', 'frontRight', 'rearLeft', 'rearRight'};
            idx = find(strcmpi(corner, names), 1);
            suspensionField = '';
            if ~isempty(idx)
                suspensionField = fields{idx};
            end

            if ~isempty(suspensionField) && ...
                    isprop(vehicleManager.suspension, suspensionField)
                cornerSuspension = vehicleManager.suspension.(suspensionField);
                if ~isempty(cornerSuspension) && isprop(cornerSuspension, 'state') && ...
                        isprop(cornerSuspension.state, 'tireNormalForce') && ...
                        isfinite(cornerSuspension.state.tireNormalForce)
                    load = max(cornerSuspension.state.tireNormalForce, 0);
                end
            end

            if ~isfinite(load)
                load = 0;
            end
        end

        function kappa = slipRatioFromWheelSpeed(wheelSpeed, longSpeed)
            denom = max(abs(wheelSpeed), abs(longSpeed));
            kappa = (wheelSpeed - longSpeed) / max(denom, 1.0);
            kappa = lts.util.clamp(kappa, -1, 1);
        end

        function kin = cornerKinematics(vehicleManager, steer)
            if ~isempty(vehicleManager.suspension) && ...
                    ismethod(vehicleManager.suspension, 'getCornerKinematics')
                kin = vehicleManager.suspension.getCornerKinematics();
            else
                kin = lts.correlation.CorrelationStateInitializer.defaultCornerKinematics( ...
                    vehicleManager, steer);
            end
        end

        function [frontArm, rearArm] = axleArms(vehicleManager)
            frontArm = vehicleManager.wheelbase * (1 - vehicleManager.staticFrontWeight);
            rearArm = vehicleManager.wheelbase * vehicleManager.staticFrontWeight;
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
