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
            adjustedMu = mu * (normalLoad / refLoad)^obj.loadSensitivityExp;
            
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
            adjustedMu = mu * (normalLoad / refLoad)^obj.loadSensitivityExp;
            
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

        function updateAllFromState(obj, state, vehicleManager, cornerLoads, mu)
            % UPDATEALLFROMSTATE Update all four tire states from vehicle state
            slipAngles = obj.computeSlipAngles( ...
                state.speed, state.vy, state.yawRate, state.steer, vehicleManager);
            suspensionKinematics = components.Tire.SimpleTire.getSuspensionKinematics( ...
                vehicleManager, state.steer);

            obj.updateCorner(obj.FL, cornerLoads.FL, slipAngles.FL, ...
                obj.computeSlipRatio(obj.FL, state.speed), ...
                suspensionKinematics.FL.camberAngle, mu);
            obj.updateCorner(obj.FR, cornerLoads.FR, slipAngles.FR, ...
                obj.computeSlipRatio(obj.FR, state.speed), ...
                suspensionKinematics.FR.camberAngle, mu);
            obj.updateCorner(obj.RL, cornerLoads.RL, slipAngles.RL, ...
                obj.computeSlipRatio(obj.RL, state.speed), ...
                suspensionKinematics.RL.camberAngle, mu);
            obj.updateCorner(obj.RR, cornerLoads.RR, slipAngles.RR, ...
                obj.computeSlipRatio(obj.RR, state.speed), ...
                suspensionKinematics.RR.camberAngle, mu);
        end

        function updateCorner(obj, cornerState, normalLoad, slipAngle, slipRatio, camberAngle, mu)
            % UPDATECORNER Evaluate the simple tire model for one corner
            cornerState.normalForce = normalLoad;
            cornerState.slipAngle = slipAngle;
            cornerState.slipRatio = slipRatio;
            cornerState.camberAngle = camberAngle;
            cornerState.Fx = obj.computeLongitudinalForce(normalLoad, slipRatio, mu);
            cornerState.Fy = obj.computeLateralForce(normalLoad, slipAngle, mu);
            cornerState.Mx = 0;
            cornerState.My = 0;
            cornerState.Mz = 0;
            cornerState.peakMu = obj.getPeakFriction(normalLoad);
        end

        function slipAngles = computeSlipAngles(obj, vx, vy, yawRate, steerInput, vehicleManager)
            % COMPUTESLIPANGLES Per-corner slip angles from wheel kinematics
            slipAngles = struct('FL', 0, 'FR', 0, 'RL', 0, 'RR', 0);

            if vx < 0.5
                return;
            end

            suspensionKinematics = components.Tire.SimpleTire.getSuspensionKinematics( ...
                vehicleManager, steerInput);
            [xFL, yFL] = components.Tire.SimpleTire.getWheelPosition(vehicleManager, 'FL');
            [xFR, yFR] = components.Tire.SimpleTire.getWheelPosition(vehicleManager, 'FR');
            [xRL, yRL] = components.Tire.SimpleTire.getWheelPosition(vehicleManager, 'RL');
            [xRR, yRR] = components.Tire.SimpleTire.getWheelPosition(vehicleManager, 'RR');

            slipAngles.FL = components.Tire.SimpleTire.computeCornerSlipAngle(vx, vy, yawRate, ...
                xFL, yFL, suspensionKinematics.FL);
            slipAngles.FR = components.Tire.SimpleTire.computeCornerSlipAngle(vx, vy, yawRate, ...
                xFR, yFR, suspensionKinematics.FR);
            slipAngles.RL = components.Tire.SimpleTire.computeCornerSlipAngle(vx, vy, yawRate, ...
                xRL, yRL, suspensionKinematics.RL);
            slipAngles.RR = components.Tire.SimpleTire.computeCornerSlipAngle(vx, vy, yawRate, ...
                xRR, yRR, suspensionKinematics.RR);
        end
    end

    methods (Static, Access = private)
        function suspensionKinematics = getSuspensionKinematics(vehicleManager, steerInput)
            if ~isempty(vehicleManager.suspension) && ...
                    ismethod(vehicleManager.suspension, 'getCornerKinematics')
                suspensionKinematics = vehicleManager.suspension.getCornerKinematics();
                return;
            end

            suspensionKinematics = struct();
            suspensionKinematics.FL = struct('camberAngle', 0, 'toeAngle', 0, 'steerAngle', steerInput);
            suspensionKinematics.FR = struct('camberAngle', 0, 'toeAngle', 0, 'steerAngle', steerInput);
            suspensionKinematics.RL = struct('camberAngle', 0, 'toeAngle', 0, 'steerAngle', 0);
            suspensionKinematics.RR = struct('camberAngle', 0, 'toeAngle', 0, 'steerAngle', 0);
        end

        function [x, y] = getWheelPosition(vehicleManager, corner)
            frontArm = vehicleManager.wheelbase * (1 - vehicleManager.staticFrontWeight);
            rearArm = vehicleManager.wheelbase * vehicleManager.staticFrontWeight;
            halfTrack = vehicleManager.trackWidth / 2;

            switch upper(corner)
                case 'FL'
                    x = frontArm;
                    y = halfTrack;
                case 'FR'
                    x = frontArm;
                    y = -halfTrack;
                case 'RL'
                    x = -rearArm;
                    y = halfTrack;
                otherwise
                    x = -rearArm;
                    y = -halfTrack;
            end
        end

        function alpha = computeCornerSlipAngle(vx, vy, yawRate, x, y, kin)
            vxCorner = vx - yawRate * y;
            vyCorner = vy + yawRate * x;
            wheelHeading = kin.steerAngle + kin.toeAngle;
            alpha = wheelHeading - atan2(vyCorner, max(vxCorner, eps));
        end
    end
end
