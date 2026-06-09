classdef SimpleTire < components.Tire.TireModel
    % SIMPLETIRE Linear tire model with saturation
    % Uses a linear region up to a peak slip, then saturates
    % Includes basic load sensitivity (friction decreases with load)
    
    properties
        corneringStiffness = 800   % Cornering stiffness per tire [N/deg] (per side of axle)
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
            if nargin >= 1
                obj.corneringStiffness = corneringStiffness;
            end
            if nargin >= 2
                obj.longitudinalStiffness = longitudinalStiffness;
            end
            if nargin >= 3
                obj.peakMuLat = peakMuLat;
                obj.peakMuLong = peakMuLat;
            end
            if nargin >= 4
                obj.loadSensitivityExp = loadSensitivityExp;
            end

            obj.FL = components.Tire.TireState();
            obj.FR = components.Tire.TireState();
            obj.RL = components.Tire.TireState();
            obj.RR = components.Tire.TireState();
        end
        
        function Fy = computeLateralForce(obj, normalLoad, slipAngle, mu)
            % Compute lateral force using linear-saturation model
            %   Fy = min(Calpha * alpha, mu * Fz)
            if normalLoad <= 0
                Fy = 0;
                return;
            end
            
            % Adjust friction for load sensitivity
            % Reference load = 1500 N (typical FSAE corner weight)
            refLoad = 1500;
            adjustedMu = min(max(mu, 0), ...
                obj.peakMuLat * (normalLoad / refLoad)^obj.loadSensitivityExp);
            
            % Linear force
            Fy_linear = obj.corneringStiffness * abs(slipAngle);
            
            % Maximum force (saturation)
            Fy_max = adjustedMu * normalLoad;
            
            % Take minimum and apply sign
            Fy = sign(slipAngle) * min(Fy_linear, Fy_max);
        end
        
        function Fx = computeLongitudinalForce(obj, normalLoad, slipRatio, mu)
            % Compute longitudinal force using linear-saturation model
            if normalLoad <= 0
                Fx = 0;
                return;
            end
            
            refLoad = 1500;
            adjustedMu = min(max(mu, 0), ...
                obj.peakMuLong * (normalLoad / refLoad)^obj.loadSensitivityExp);
            
            % Linear force
            Fx_linear = obj.longitudinalStiffness * abs(slipRatio);
            
            % Maximum force (saturation)
            Fx_max = adjustedMu * normalLoad;
            
            % Take minimum and apply sign
            Fx = sign(slipRatio) * min(Fx_linear, Fx_max);
        end
        
        function mu = getPeakFriction(obj, normalLoad)
            % Get peak friction coefficient adjusted for load
            if normalLoad <= 0
                mu = 0;
                return;
            end
            refLoad = 1500;
            mu = obj.peakMuLat * (normalLoad / refLoad)^obj.loadSensitivityExp;
        end

        function kappa = computeSlipRatio(obj, cornerState, vehicleSpeed)
            % COMPUTESLIPRATIO Compute longitudinal slip ratio for one corner
            omega = cornerState.angularVelocity;
            R = cornerState.wheelRadius;
            V = max(vehicleSpeed, 0);

            wheelSpeed = omega * R;
            denom = max(abs(wheelSpeed), abs(V));

            if denom < 0.1
                kappa = 0;
            else
                kappa = (wheelSpeed - V) / denom;
            end

            kappa = max(-1, min(1, kappa));
        end

        function updateWheelDynamics(obj, cornerState, driveTorque, brakeTorque, dt)
            % UPDATEWHEELDYNAMICS Integrate wheel angular velocity forward
            omega = cornerState.angularVelocity;
            R = cornerState.wheelRadius;
            Fx = cornerState.Fx;

            netTorque = driveTorque - sign(omega) * brakeTorque - Fx * R;
            omega = omega + (netTorque / obj.wheelInertia) * dt;

            cornerState.angularVelocity = max(0, omega);
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

            [alpha_FL, kappa_FL] = obj.relaxSlipInputs(obj.FL, wheelKinematics.FL.slipAngle, ...
                obj.computeSlipRatio(obj.FL, wheelKinematics.FL.longitudinalSpeed), state.speed, dt);
            [alpha_FR, kappa_FR] = obj.relaxSlipInputs(obj.FR, wheelKinematics.FR.slipAngle, ...
                obj.computeSlipRatio(obj.FR, wheelKinematics.FR.longitudinalSpeed), state.speed, dt);
            [alpha_RL, kappa_RL] = obj.relaxSlipInputs(obj.RL, wheelKinematics.RL.slipAngle, ...
                obj.computeSlipRatio(obj.RL, wheelKinematics.RL.longitudinalSpeed), state.speed, dt);
            [alpha_RR, kappa_RR] = obj.relaxSlipInputs(obj.RR, wheelKinematics.RR.slipAngle, ...
                obj.computeSlipRatio(obj.RR, wheelKinematics.RR.longitudinalSpeed), state.speed, dt);

            obj.updateCorner(obj.FL, cornerLoads.FL, alpha_FL, ...
                kappa_FL, mu);
            obj.updateCorner(obj.FR, cornerLoads.FR, alpha_FR, ...
                kappa_FR, mu);
            obj.updateCorner(obj.RL, cornerLoads.RL, alpha_RL, ...
                kappa_RL, mu);
            obj.updateCorner(obj.RR, cornerLoads.RR, alpha_RR, ...
                kappa_RR, mu);
        end

        function updateCorner(obj, cornerState, normalLoad, slipAngle, slipRatio, mu)
            % UPDATECORNER Evaluate the simple tire model for one corner
            cornerState.normalForce = normalLoad;
            cornerState.slipAngle = slipAngle;
            cornerState.slipRatio = slipRatio;
            cornerState.camberAngle = 0;
            cornerState.Fx = obj.computeLongitudinalForce(normalLoad, slipRatio, mu);
            cornerState.Fy = obj.computeLateralForce(normalLoad, slipAngle, mu);

            effectiveMu = min(obj.getPeakFriction(normalLoad), max(mu, 0));
            frictionLimit = effectiveMu * max(normalLoad, 0);
            forceMagnitude = hypot(cornerState.Fx, cornerState.Fy);
            if frictionLimit > 0 && forceMagnitude > frictionLimit
                scale = frictionLimit / forceMagnitude;
                cornerState.Fx = cornerState.Fx * scale;
                cornerState.Fy = cornerState.Fy * scale;
                forceMagnitude = frictionLimit;
            end

            cornerState.Mx = 0;
            cornerState.My = 0;
            cornerState.Mz = 0;
            cornerState.peakMu = effectiveMu;
            cornerState.frictionLimit = frictionLimit;
            if frictionLimit > 0
                cornerState.frictionUsage = forceMagnitude / frictionLimit;
            else
                cornerState.frictionUsage = 0;
            end
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
            wheelKinematics = obj.emptyWheelKinematics(max(vx, 0));

            if vx < 0.5
                return;
            end
            if nargin < 8 || isempty(trackWidth)
                trackWidth = 0;
            end

            lf = wheelbase * (1 - frontWeightFrac);
            lr = wheelbase * frontWeightFrac;
            halfTrack = max(trackWidth, 0) / 2;
            [deltaFL, deltaFR] = obj.computeAckermannSteer(steerInput, wheelbase, trackWidth);

            wheelKinematics.FL = obj.computeCornerKinematics(vx, vy, yawRate, lf, halfTrack, deltaFL);
            wheelKinematics.FR = obj.computeCornerKinematics(vx, vy, yawRate, lf, -halfTrack, deltaFR);
            wheelKinematics.RL = obj.computeCornerKinematics(vx, vy, yawRate, -lr, halfTrack, 0);
            wheelKinematics.RR = obj.computeCornerKinematics(vx, vy, yawRate, -lr, -halfTrack, 0);
        end

        function wheel = computeCornerKinematics(~, vx, vy, yawRate, xOffset, yOffset, steerAngle)
            wheelVx = vx - yawRate * yOffset;
            wheelVy = vy + yawRate * xOffset;
            pathAngle = atan2(wheelVy, max(wheelVx, eps));
            localLongitudinalSpeed = wheelVx * cos(steerAngle) + wheelVy * sin(steerAngle);

            wheel = struct( ...
                'slipAngle', steerAngle - pathAngle, ...
                'longitudinalSpeed', max(localLongitudinalSpeed, 0));
        end

        function wheelKinematics = emptyWheelKinematics(~, vehicleSpeed)
            zeroWheel = struct('slipAngle', 0, 'longitudinalSpeed', max(vehicleSpeed, 0));
            wheelKinematics = struct('FL', zeroWheel, 'FR', zeroWheel, ...
                'RL', zeroWheel, 'RR', zeroWheel);
        end

        function [deltaFL, deltaFR] = computeAckermannSteer(~, steerInput, wheelbase, trackWidth)
            % COMPUTEACKERMANNSTEER Convert bicycle steer to front wheel angles.
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
            cornerState.targetSlipAngle = targetAlpha;
            cornerState.targetSlipRatio = targetKappa;

            if ~obj.enableRelaxation || dt <= 0 || vehicleSpeed < 0.5
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

            alphaGain = min(1, vehicleSpeed * dt / max(obj.lateralRelaxationLength, eps));
            kappaGain = min(1, vehicleSpeed * dt / max(obj.longitudinalRelaxationLength, eps));

            cornerState.relaxedSlipAngle = cornerState.relaxedSlipAngle + ...
                alphaGain * (targetAlpha - cornerState.relaxedSlipAngle);
            cornerState.relaxedSlipRatio = cornerState.relaxedSlipRatio + ...
                kappaGain * (targetKappa - cornerState.relaxedSlipRatio);

            alpha = cornerState.relaxedSlipAngle;
            kappa = cornerState.relaxedSlipRatio;
        end
    end
end
