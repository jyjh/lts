classdef SimpleTire < components.Tire.TireModel
    % SIMPLETIRE Linear tire model with saturation
    % Uses a linear region up to a peak slip, then saturates
    % Includes basic load sensitivity (friction decreases with load)

    properties
        corneringStiffness = 45000 % Cornering stiffness per tire [N/rad]
        longitudinalStiffness = 10000 % Longitudinal stiffness per tire [N/unit slip]
        peakMuLat          = 1.8   % Peak lateral friction coefficient
        peakMuLong         = 1.8   % Peak longitudinal friction coefficient
        peakSlipAngle      = 5.0   % Slip angle at peak lateral force [deg]
        peakSlipRatio      = 0.10  % Slip ratio at peak longitudinal force
        loadSensitivityExp = -0.1  % Load sensitivity exponent (negative = mu drops with load)
        wheelInertia       = 0.5   % Wheel rotational inertia per corner [kg*m^2]
        enableRelaxation   = true  % Apply tire relaxation length to slip inputs
        lateralRelaxationLength = 2.5 % Lateral relaxation length [m]
        longitudinalRelaxationLength = 0.8 % Longitudinal relaxation length [m]
        FL                        % TireState front-left
        FR                        % TireState front-right
        RL                        % TireState rear-left
        RR                        % TireState rear-right
    end

    methods
        function obj = SimpleTire(corneringStiffness, longitudinalStiffness, peakMuLat, loadSensitivityExp)
            % SIMPLETIRE Construct with fixed parameters
            %   SimpleTire(corneringStiffness, longitudinalStiffness, peakMuLat, loadSensitivityExp)
            if nargin >= 1 && ~isempty(corneringStiffness)
                obj.corneringStiffness = utils.nonnegativeScalarOrDefault( ...
                    corneringStiffness, obj.corneringStiffness);
            end
            if nargin >= 2 && ~isempty(longitudinalStiffness)
                obj.longitudinalStiffness = utils.nonnegativeScalarOrDefault( ...
                    longitudinalStiffness, obj.longitudinalStiffness);
            end
            if nargin >= 3 && ~isempty(peakMuLat)
                peakMuLat = utils.nonnegativeScalarOrDefault( ...
                    peakMuLat, obj.peakMuLat);
                obj.peakMuLat = peakMuLat;
                obj.peakMuLong = peakMuLat;
            end
            if nargin >= 4 && ~isempty(loadSensitivityExp)
                obj.loadSensitivityExp = utils.scalarOrDefault( ...
                    loadSensitivityExp, obj.loadSensitivityExp);
            end

            obj.FL = components.Tire.TireState();
            obj.FR = components.Tire.TireState();
            obj.RL = components.Tire.TireState();
            obj.RR = components.Tire.TireState();
        end

        function Fy = computeLateralForce(obj, normalLoad, slipAngle, mu)
            % Compute lateral force using linear-saturation model
            %   Fy = min(Calpha * alpha, mu * Fz)
            normalLoad = utils.nonnegativeScalarOrDefault(normalLoad, 0);
            slipAngle = utils.scalarOrDefault(slipAngle, 0);
            corneringStiffness = utils.nonnegativeScalarOrDefault( ...
                obj.corneringStiffness, 45000);
            if normalLoad <= 0
                Fy = 0;
                return;
            end

            adjustedMu = obj.getAdjustedLateralMu(normalLoad, mu);

            % Linear force
            Fy_linear = corneringStiffness * abs(slipAngle);

            % Maximum force (saturation)
            Fy_max = adjustedMu * normalLoad;

            % Take minimum and apply sign
            Fy = sign(slipAngle) * min(Fy_linear, Fy_max);
        end

        function Fx = computeLongitudinalForce(obj, normalLoad, slipRatio, mu)
            % Compute longitudinal force using linear-saturation model
            normalLoad = utils.nonnegativeScalarOrDefault(normalLoad, 0);
            slipRatio = utils.unitSignedScalarOrDefault(slipRatio, 0);
            longitudinalStiffness = utils.nonnegativeScalarOrDefault( ...
                obj.longitudinalStiffness, 10000);
            if normalLoad <= 0
                Fx = 0;
                return;
            end

            adjustedMu = obj.getAdjustedLongitudinalMu(normalLoad, mu);

            % Linear force
            Fx_linear = longitudinalStiffness * abs(slipRatio);

            % Maximum force (saturation)
            Fx_max = adjustedMu * normalLoad;

            % Take minimum and apply sign
            Fx = sign(slipRatio) * min(Fx_linear, Fx_max);
        end

        function mu = getPeakFriction(obj, normalLoad)
            % Get peak friction coefficient adjusted for load
            normalLoad = utils.nonnegativeScalarOrDefault(normalLoad, 0);
            if normalLoad <= 0
                mu = 0;
                return;
            end
            refLoad = 1500;
            peakMu = utils.nonnegativeScalarOrDefault(obj.peakMuLat, 1.8);
            loadSensitivityExp = utils.scalarOrDefault( ...
                obj.loadSensitivityExp, -0.1);
            % Compute load sensitivity with bounds to prevent overflow for small loads
            % Negative load sensitivity means mu decreases as load increases
            % Clamp the load ratio to a reasonable range to prevent extreme values
            loadRatio = normalLoad / refLoad;
            loadRatio = max(loadRatio, 0.01);  % Prevent overflow for very small loads
            loadRatio = min(loadRatio, 10);     % Prevent overflow for very large loads
            mu = peakMu * loadRatio^loadSensitivityExp;
            % Clamp to physically reasonable range for racing tires
            mu = max(mu, 0.5);  % Minimum friction even at very low loads
            mu = min(mu, 2.5);  % Maximum realistic peak friction
        end

        function mu = getPeakLongitudinalFriction(obj, normalLoad)
            % Get longitudinal peak friction coefficient adjusted for load
            normalLoad = utils.nonnegativeScalarOrDefault(normalLoad, 0);
            if normalLoad <= 0
                mu = 0;
                return;
            end
            refLoad = 1500;
            peakMu = utils.nonnegativeScalarOrDefault(obj.peakMuLong, 1.8);
            loadSensitivityExp = utils.scalarOrDefault( ...
                obj.loadSensitivityExp, -0.1);
            % Compute load sensitivity with bounds to prevent overflow for small loads
            loadRatio = normalLoad / refLoad;
            loadRatio = max(loadRatio, 0.01);  % Prevent overflow for very small loads
            loadRatio = min(loadRatio, 10);     % Prevent overflow for very large loads
            mu = peakMu * loadRatio^loadSensitivityExp;
            % Clamp to physically reasonable range for racing tires
            mu = max(mu, 0.5);  % Minimum friction even at very low loads
            mu = min(mu, 2.5);  % Maximum realistic peak friction
        end

        function kappa = computeSlipRatio(obj, cornerState, vehicleSpeed)
            % COMPUTESLIPRATIO Compute longitudinal slip ratio for one corner
            omega = utils.nonnegativeScalarOrDefault( ...
                cornerState.angularVelocity, 0);
            R = utils.positiveScalarOrDefault(cornerState.wheelRadius, 0.241935);
            V = utils.nonnegativeScalarOrDefault(vehicleSpeed, 0);

            wheelSpeed = omega * R;
            denom = max(abs(wheelSpeed), abs(V));

            if denom < 0.1
                kappa = 0;
            else
                kappa = (wheelSpeed - V) / denom;
            end

            kappa = max(-1, min(1, kappa));
        end

        function omegaTelemetry = updateWheelDynamics(obj, cornerState, driveTorque, brakeTorque, dt)
            % UPDATEWHEELDYNAMICS Integrate wheel angular velocity forward
            omega = utils.nonnegativeScalarOrDefault( ...
                cornerState.angularVelocity, 0);
            R = utils.positiveScalarOrDefault(cornerState.wheelRadius, 0.241935);
            Fx = utils.scalarOrDefault(cornerState.Fx, 0);
            driveTorque = utils.scalarOrDefault(driveTorque, 0);
            brakeTorque = utils.nonnegativeScalarOrDefault(brakeTorque, 0);
            dt = utils.nonnegativeScalarOrDefault(dt, 0);
            wheelInertia = utils.positiveScalarOrDefault(obj.wheelInertia, 0.5);

            % Brake torque opposes forward rolling. When a wheel is locked
            % omega is exactly zero, but in this forward-only lap model the
            % caliper still resists the road torque that would spin it forward.
            brakeDirection = sign(omega);
            if brakeDirection == 0 && brakeTorque > 0
                brakeDirection = 1;
            end

            netTorque = driveTorque - brakeDirection * brakeTorque - Fx * R;
            omegaUnclamped = omega + (netTorque / wheelInertia) * dt;

            cornerState.angularVelocity = max(0, omegaUnclamped);
            omegaTelemetry.omegaBefore = max(omega, 0);
            omegaTelemetry.omegaUnclamped = omegaUnclamped;
            omegaTelemetry.omegaAfter = cornerState.angularVelocity;
            omegaTelemetry.omegaMean = obj.computeMeanPositiveAngularVelocity( ...
                omega, omegaUnclamped);
        end

        function updateAllFromState(obj, state, vehicleManager, cornerLoads, mu, dt)
            % UPDATEALLFROMSTATE Update all four tire states from vehicle state
            if nargin < 6
                dt = 0;
            end
            wheelKinematics = obj.computeWheelKinematics( ...
                state.speed, state.vy, state.yawRate, state.steer, ...
                vehicleManager.wheelbase, vehicleManager.staticFrontWeight, ...
                vehicleManager.trackWidth);

            % Relaxation length is distance travelled by that tire contact
            % patch, so use each wheel's local longitudinal speed rather than
            % the CG speed.
            [alpha_FL, kappa_FL] = obj.relaxSlipInputs(obj.FL, wheelKinematics.FL.slipAngle, ...
                obj.computeSlipRatio(obj.FL, wheelKinematics.FL.longitudinalSpeed), ...
                wheelKinematics.FL.longitudinalSpeed, dt);
            [alpha_FR, kappa_FR] = obj.relaxSlipInputs(obj.FR, wheelKinematics.FR.slipAngle, ...
                obj.computeSlipRatio(obj.FR, wheelKinematics.FR.longitudinalSpeed), ...
                wheelKinematics.FR.longitudinalSpeed, dt);
            [alpha_RL, kappa_RL] = obj.relaxSlipInputs(obj.RL, wheelKinematics.RL.slipAngle, ...
                obj.computeSlipRatio(obj.RL, wheelKinematics.RL.longitudinalSpeed), ...
                wheelKinematics.RL.longitudinalSpeed, dt);
            [alpha_RR, kappa_RR] = obj.relaxSlipInputs(obj.RR, wheelKinematics.RR.slipAngle, ...
                obj.computeSlipRatio(obj.RR, wheelKinematics.RR.longitudinalSpeed), ...
                wheelKinematics.RR.longitudinalSpeed, dt);

            obj.updateCorner(obj.FL, utils.cornerLoadOrDefault(cornerLoads, 'FL'), alpha_FL, ...
                kappa_FL, mu);
            obj.updateCorner(obj.FR, utils.cornerLoadOrDefault(cornerLoads, 'FR'), alpha_FR, ...
                kappa_FR, mu);
            obj.updateCorner(obj.RL, utils.cornerLoadOrDefault(cornerLoads, 'RL'), alpha_RL, ...
                kappa_RL, mu);
            obj.updateCorner(obj.RR, utils.cornerLoadOrDefault(cornerLoads, 'RR'), alpha_RR, ...
                kappa_RR, mu);
        end

        function updateCorner(obj, cornerState, normalLoad, slipAngle, slipRatio, mu)
            % UPDATECORNER Evaluate the simple tire model for one corner
            normalLoad = utils.nonnegativeScalarOrDefault(normalLoad, 0);
            slipAngle = utils.scalarOrDefault(slipAngle, 0);
            slipRatio = utils.unitSignedScalarOrDefault(slipRatio, 0);
            mu = utils.nonnegativeScalarOrDefault(mu, 0);
            cornerState.normalForce = normalLoad;
            cornerState.slipAngle = slipAngle;
            cornerState.slipRatio = slipRatio;
            cornerState.camberAngle = 0;
            cornerState.Fx = obj.computeLongitudinalForce(normalLoad, slipRatio, mu);
            cornerState.Fy = obj.computeLateralForce(normalLoad, slipAngle, mu);

            % Combined slip is limited by a friction ellipse. This keeps the
            % simple tire from producing full braking/drive force and full
            % cornering force at the same time, while still allowing separate
            % longitudinal and lateral peak coefficients.
            [usage, directionalLimit] = obj.computeFrictionEllipseUsage( ...
                cornerState.Fx, cornerState.Fy, normalLoad, mu);
            if usage > 1
                scale = 1 / usage;
                cornerState.Fx = cornerState.Fx * scale;
                cornerState.Fy = cornerState.Fy * scale;
                usage = 1;
            end

            cornerState.Mx = 0;
            cornerState.My = 0;
            cornerState.Mz = 0;
            cornerState.peakMu = obj.getAdjustedLateralMu(normalLoad, mu);
            cornerState.peakMuLong = obj.getAdjustedLongitudinalMu(normalLoad, mu);
            cornerState.frictionLimit = directionalLimit;
            cornerState.frictionUsage = usage;
        end

        function slipAngles = computeSlipAngles(obj, vx, vy, yawRate, steerInput, wheelbase, frontWeightFrac)
            % COMPUTESLIPANGLES Per-corner slip angles for all corners
            wheelKinematics = obj.computeWheelKinematics( ...
                vx, vy, yawRate, steerInput, wheelbase, frontWeightFrac, 0);
            slipAngles = struct('FL', 0, 'FR', 0, 'RL', 0, 'RR', 0);
            slipAngles.FL = wheelKinematics.FL.slipAngle;
            slipAngles.FR = wheelKinematics.FR.slipAngle;
            slipAngles.RL = wheelKinematics.RL.slipAngle;
            slipAngles.RR = wheelKinematics.RR.slipAngle;
        end

        function wheelKinematics = computeWheelKinematics(obj, vx, vy, yawRate, steerInput, wheelbase, frontWeightFrac, trackWidth)
            % COMPUTEWHEELKINEMATICS Per-corner slip angle and wheel-plane speed
            vx = utils.scalarOrDefault(vx, 0);
            vy = utils.scalarOrDefault(vy, 0);
            yawRate = utils.scalarOrDefault(yawRate, 0);
            steerInput = utils.scalarOrDefault(steerInput, 0);
            wheelbase = utils.positiveScalarOrDefault(wheelbase, 1.55);
            frontWeightFrac = utils.unitScalarOrDefault(frontWeightFrac, 0.45);
            wheelKinematics = obj.emptyWheelKinematics(hypot(vx, vy));
            if nargin < 8 || isempty(trackWidth)
                trackWidth = 0;
            end
            trackWidth = utils.nonnegativeScalarOrDefault(trackWidth, 0);

            lf = wheelbase * (1 - frontWeightFrac);
            lr = wheelbase * frontWeightFrac;
            halfTrack = max(trackWidth, 0) / 2;
            [deltaFL, deltaFR] = obj.computeAckermannSteer(steerInput, wheelbase, trackWidth);

            wheelKinematics.FL = obj.computeCornerKinematics(vx, vy, yawRate, lf, halfTrack, deltaFL);
            wheelKinematics.FR = obj.computeCornerKinematics(vx, vy, yawRate, lf, -halfTrack, deltaFR);
            wheelKinematics.RL = obj.computeCornerKinematics(vx, vy, yawRate, -lr, halfTrack, 0);
            wheelKinematics.RR = obj.computeCornerKinematics(vx, vy, yawRate, -lr, -halfTrack, 0);
        end

        function wheel = computeCornerKinematics(obj, vx, vy, yawRate, xOffset, yOffset, steerAngle)
            vx = utils.scalarOrDefault(vx, 0);
            vy = utils.scalarOrDefault(vy, 0);
            yawRate = utils.scalarOrDefault(yawRate, 0);
            xOffset = utils.scalarOrDefault(xOffset, 0);
            yOffset = utils.scalarOrDefault(yOffset, 0);
            steerAngle = utils.scalarOrDefault(steerAngle, 0);
            wheelVx = vx - yawRate * yOffset;
            wheelVy = vy + yawRate * xOffset;

            % Work in the tire's local axes. The old low-CG-speed guard threw
            % away slip whenever vx was small, but an autocross car can still
            % be sliding laterally or yawing with meaningful contact-patch
            % speed. Slip angle is undefined only when the local patch velocity
            % itself is nearly zero.
            localLongitudinalSpeed = wheelVx * cos(steerAngle) + wheelVy * sin(steerAngle);
            localLateralSpeed = -wheelVx * sin(steerAngle) + wheelVy * cos(steerAngle);
            patchSpeed = hypot(localLongitudinalSpeed, localLateralSpeed);
            slipAngle = 0;
            if patchSpeed >= 0.1
                % Positive slip angle should create positive body-left tire
                % force. localLateralSpeed is positive when the contact patch
                % velocity points left of the wheel plane, so the tire force is
                % to the right and the slip angle is negative.
                slipAngle = -atan2(localLateralSpeed, max(localLongitudinalSpeed, eps));
            end

            wheel = struct( ...
                'slipAngle', slipAngle, ...
                'longitudinalSpeed', max(localLongitudinalSpeed, 0));
        end

        function wheelKinematics = emptyWheelKinematics(~, vehicleSpeed)
            vehicleSpeed = utils.nonnegativeScalarOrDefault( ...
                vehicleSpeed, 0);
            zeroWheel = struct('slipAngle', 0, 'longitudinalSpeed', max(vehicleSpeed, 0));
            wheelKinematics = struct('FL', zeroWheel, 'FR', zeroWheel, ...
                'RL', zeroWheel, 'RR', zeroWheel);
        end

        function [deltaFL, deltaFR] = computeAckermannSteer(obj, steerInput, wheelbase, trackWidth)
            % COMPUTEACKERMANNSTEER Convert bicycle steer to front wheel angles.
            steerInput = utils.scalarOrDefault(steerInput, 0);
            wheelbase = utils.positiveScalarOrDefault(wheelbase, 1.55);
            trackWidth = utils.nonnegativeScalarOrDefault(trackWidth, 0);
            if abs(steerInput) < 1e-6 || trackWidth <= 0 || wheelbase <= 0
                deltaFL = steerInput;
                deltaFR = steerInput;
                return;
            end

            turnRadius = wheelbase / tan(abs(steerInput));
            halfTrack = trackWidth / 2;
            innerRadius = max(turnRadius - halfTrack, 0.1);
            outerRadius = turnRadius + halfTrack;
            innerAngle = atan(wheelbase / innerRadius);
            outerAngle = atan(wheelbase / outerRadius);

            if steerInput > 0
                deltaFL = innerAngle;
                deltaFR = outerAngle;
            else
                deltaFL = -outerAngle;
                deltaFR = -innerAngle;
            end
        end

        function [alpha, kappa] = relaxSlipInputs(obj, cornerState, targetAlpha, targetKappa, vehicleSpeed, dt)
            targetAlpha = utils.scalarOrDefault(targetAlpha, 0);
            targetKappa = utils.unitSignedScalarOrDefault(targetKappa, 0);
            vehicleSpeed = utils.nonnegativeScalarOrDefault(vehicleSpeed, 0);
            dt = utils.nonnegativeScalarOrDefault(dt, 0);
            cornerState.targetSlipAngle = targetAlpha;
            cornerState.targetSlipRatio = targetKappa;

            enableRelaxation = utils.logicalScalarOrDefault( ...
                obj.enableRelaxation, true);
            if ~enableRelaxation || dt <= 0 || vehicleSpeed < 0.5
                cornerState.relaxedSlipAngle = targetAlpha;
                cornerState.relaxedSlipRatio = targetKappa;
                cornerState.slipStateInitialized = true;
                alpha = targetAlpha;
                kappa = targetKappa;
                return;
            end

            if ~cornerState.slipStateInitialized
                cornerState.relaxedSlipAngle = targetAlpha;
                cornerState.relaxedSlipRatio = targetKappa;
                cornerState.slipStateInitialized = true;
            end

            lateralRelaxationLength = utils.positiveScalarOrDefault( ...
                obj.lateralRelaxationLength, 2.5);
            longitudinalRelaxationLength = utils.positiveScalarOrDefault( ...
                obj.longitudinalRelaxationLength, 0.8);
            alphaGain = min(1, vehicleSpeed * dt / max(lateralRelaxationLength, eps));
            kappaGain = min(1, vehicleSpeed * dt / max(longitudinalRelaxationLength, eps));

            cornerState.relaxedSlipAngle = cornerState.relaxedSlipAngle + ...
                alphaGain * (targetAlpha - cornerState.relaxedSlipAngle);
            cornerState.relaxedSlipRatio = cornerState.relaxedSlipRatio + ...
                kappaGain * (targetKappa - cornerState.relaxedSlipRatio);

            alpha = cornerState.relaxedSlipAngle;
            kappa = cornerState.relaxedSlipRatio;
        end

        function mu = getAdjustedLateralMu(obj, normalLoad, surfaceMu)
            % GETADJUSTEDLATERALMU Apply load sensitivity and surface grip cap.
            normalLoad = utils.nonnegativeScalarOrDefault(normalLoad, 0);
            surfaceMu = utils.nonnegativeScalarOrDefault(surfaceMu, 0);
            mu = min(max(surfaceMu, 0), obj.getPeakFriction(normalLoad));
        end

        function mu = getAdjustedLongitudinalMu(obj, normalLoad, surfaceMu)
            % GETADJUSTEDLONGITUDINALMU Apply load sensitivity and surface grip cap.
            normalLoad = utils.nonnegativeScalarOrDefault(normalLoad, 0);
            surfaceMu = utils.nonnegativeScalarOrDefault(surfaceMu, 0);
            mu = min(max(surfaceMu, 0), obj.getPeakLongitudinalFriction(normalLoad));
        end

        function [usage, directionalLimit] = computeFrictionEllipseUsage( ...
                obj, Fx, Fy, normalLoad, surfaceMu)
            % COMPUTEFRICTIONELLIPSEUSAGE Combined longitudinal/lateral usage.
            Fx = utils.scalarOrDefault(Fx, 0);
            Fy = utils.scalarOrDefault(Fy, 0);
            normalLoad = utils.nonnegativeScalarOrDefault(normalLoad, 0);
            surfaceMu = utils.nonnegativeScalarOrDefault(surfaceMu, 0);
            FxLimit = obj.getAdjustedLongitudinalMu(normalLoad, surfaceMu) ...
                * max(normalLoad, 0);
            FyLimit = obj.getAdjustedLateralMu(normalLoad, surfaceMu) ...
                * max(normalLoad, 0);

            if FxLimit <= 0 || FyLimit <= 0
                usage = 0;
                directionalLimit = 0;
                return;
            end

            usage = hypot(Fx / FxLimit, Fy / FyLimit);
            forceMagnitude = hypot(Fx, Fy);
            if forceMagnitude > 0
                cosForce = Fx / forceMagnitude;
                sinForce = Fy / forceMagnitude;
                directionalLimit = 1 / sqrt((cosForce / FxLimit)^2 ...
                    + (sinForce / FyLimit)^2);
            else
                directionalLimit = min(FxLimit, FyLimit);
            end
        end
    end
end
