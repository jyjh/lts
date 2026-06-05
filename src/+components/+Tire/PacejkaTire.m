classdef PacejkaTire < components.Tire.TireModel
    % PACEJKATIRE Pacejka Magic Formula tire model via MFeval (4-corner manager)
    %
    % Manages four per-corner TireState objects (FL, FR, RL, RR), each with
    % independent inputs (slip angle, slip ratio, camber, normal load) and
    % outputs (Fx, Fy, Mz, etc.). All corners share a single TireConstants
    % object that holds the parsed .tir file coefficients.
    %
    % Architecture mirrors SuspensionManager:
    %   TireConstants — shared immutable Pacejka coefficients (like suspension params)
    %   TireState     — per-corner mutable state (like SuspensionState)
    %   PacejkaTire   — manager that creates states and evaluates MFeval
    %
    % Dependencies:
    %   MFeval toolbox — https://www.mathworks.com/matlabcentral/fileexchange/63618-mfeval
    %
    % Usage:
    %   tire = components.Tire.PacejkaTire('43105_18x7.5_10_R25B_7.tir')
    %   tire.updateCorner(tire.FL, Fz, alpha, kappa, gamma, mu)
    %   tire.FL.Fy   % lateral force on front-left
    %   mu = tire.getPeakFriction(Fz)
    
    properties
        % Shared tire coefficients (from .tir file)
        tireConstants
        
        % Per-corner tire state objects (handle objects, mutated in-place)
        FL   % TireState — front-left
        FR   % TireState — front-right
        RL   % TireState — rear-left
        RR   % TireState — rear-right
        
        % Wheel rotational inertia per corner [kg·m^2]
        % (wheel + tire + brake disc rotating assembly)
        wheelInertia = 0.5
    end
    
    methods
        function obj = PacejkaTire(tirFilePath)
            % PACEJKATIRE Construct from a .tir file, creating 4 corner states
            %   PacejkaTire(tirFilePath)
            %
            %   tirFilePath — path to the .tir file. If relative, resolved
            %                 relative to the +Tire/ folder.
            
            % Load shared tire constants
            obj.tireConstants = components.Tire.TireConstants(tirFilePath);
            
            % Create per-corner state objects
            obj.FL = components.Tire.TireState();
            obj.FR = components.Tire.TireState();
            obj.RL = components.Tire.TireState();
            obj.RR = components.Tire.TireState();
            
            fprintf('  PacejkaTire: 4 corner states created (FL, FR, RL, RR)\n');
        end
        
        %% ---- Per-corner evaluation ----
        
        function updateCorner(obj, cornerState, normalLoad, slipAngle, slipRatio, camberAngle, mu)
            % UPDATECORNER Evaluate MFeval for one corner and update its state
            %   updateCorner(cornerState, normalLoad, slipAngle, slipRatio, camberAngle, mu)
            %
            %   cornerState  — TireState handle for this corner
            %   normalLoad   — Tire normal force Fz [N]
            %   slipAngle    — Tire slip angle alpha [rad]
            %   slipRatio    — Tire slip ratio kappa [-1 to 1]
            %   camberAngle  — Inclination angle gamma [rad]
            %   mu           — Surface friction multiplier (1.0 = nominal)
            %
            %   Mutates cornerState in-place with computed forces and moments.
            
            % Store inputs
            cornerState.normalForce = normalLoad;
            cornerState.slipAngle   = slipAngle;
            cornerState.slipRatio   = slipRatio;
            cornerState.camberAngle = camberAngle;
            
            if normalLoad <= 0
                cornerState.Fy = 0;
                cornerState.Fx = 0;
                cornerState.Mx = 0;
                cornerState.My = 0;
                cornerState.Mz = 0;
                cornerState.peakMu = 0;
                return;
            end
            
            % Unpack for MFeval call
            kappa = slipRatio;
            alpha = slipAngle;
            Fz    = normalLoad;
            gamma = camberAngle;
            V     = obj.tireConstants.refVelocity;
            P     = obj.tireConstants.nomPressure;
            params = obj.tireConstants.params;
            
            % Build MFeval inputs row: [Fz, kappa, alpha, gamma, phit, Vx, P]
            inputsMF = [Fz, kappa, alpha, gamma, 0, V, P];
            
            % Evaluate Pacejka Magic Formula via MFeval (useMode=1: combined)
            outputs = mfeval(params, inputsMF, 111);
            
            % Store outputs (apply surface friction multiplier)
            cornerState.Fy = outputs(:,2) * mu;
            cornerState.Fx = outputs(:,1) * mu;
            cornerState.Mx = outputs(:,4) * mu;
            cornerState.My = outputs(:,5) * mu;
            cornerState.Mz = outputs(:,6) * mu;
            
            % Compute peak mu at this load
            cornerState.peakMu = obj.computePeakMuInternal(Fz, gamma, P, params);
        end
        
        %% ---- TireModel interface methods ----
        
        function Fy = computeLateralForce(obj, normalLoad, slipAngle, mu)
            % COMPUTELATERALFORCE Lateral force [N] for a single evaluation
            %   Fy = computeLateralForce(obj, normalLoad, slipAngle, mu)
            %
            %   This is the TireModel interface method for standalone queries.
            %   For per-corner state tracking, use updateCorner() instead.
            
            if normalLoad <= 0
                Fy = 0;
                return;
            end
            
            inputsMF = [normalLoad, 0, slipAngle, 0, 0, ...
                obj.tireConstants.refVelocity, obj.tireConstants.nomPressure];
            outputs = mfeval(obj.tireConstants.params, inputsMF, 111);
            
            Fy = outputs.Fy * mu;
        end
        
        function Fx = computeLongitudinalForce(obj, normalLoad, slipRatio, mu)
            % COMPUTELONGITUDINALFORCE Longitudinal force [N] for a single evaluation
            %   Fx = computeLongitudinalForce(obj, normalLoad, slipRatio, mu)
            %
            %   This is the TireModel interface method for standalone queries.
            %   For per-corner state tracking, use updateCorner() instead.
            
            if normalLoad <= 0
                Fx = 0;
                return;
            end
            
            inputsMF = [normalLoad, slipRatio, 0, 0, 0, ...
                obj.tireConstants.refVelocity, obj.tireConstants.nomPressure];
            outputs = mfeval(obj.tireConstants.params, inputsMF, 1);
            
            Fx = outputs.Fx * mu;
        end
        
        function peakMu = getPeakFriction(obj, normalLoad)
            % GETPEAKFRICTION Peak friction coefficient at given load
            %   peakMu = getPeakFriction(obj, normalLoad)
            %
            %   Scans the lateral force curve to find max |Fy|/Fz.
            %   Accounts for load sensitivity inherent in the Magic Formula.
            
            if normalLoad <= 0
                peakMu = 0;
                return;
            end
            
            peakMu = obj.computePeakMuInternal(normalLoad, 0, ...
                obj.tireConstants.nomPressure, obj.tireConstants.params);
        end
        
        %% ---- Slip angle computation ----
        
        function slipAngles = computeSlipAngles(obj, vx, vy, yawRate, steerInput, wheelbase, frontWeightFrac)
            % COMPUTESLIPANGLES Compute per-corner tire slip angles [rad]
            %   slipAngles = computeSlipAngles(vx, vy, yawRate, steerInput, wheelbase, frontWeightFrac)
            %
            %   Uses bicycle model kinematics:
            %     Rear:  alpha = -atan((vy - lr*yawRate) / vx)
            %     Front: alpha = delta - atan((vy + lf*yawRate) / vx)
            %
            %   Left and right tires on the same axle share the same slip
            %   angle (Ackermann steering geometry is not yet modelled).
            %
            %   Inputs:
            %     vx              - forward velocity [m/s]
            %     vy              - lateral velocity at CG [m/s]
            %     yawRate         - yaw rate [rad/s]
            %     steerInput      - driver steering input [rad]
            %     wheelbase       - vehicle wheelbase [m]
            %     frontWeightFrac - static front weight distribution [0-1]
            %
            %   Returns struct with:
            %     slipAngles.FL, .FR, .RL, .RR  [rad]

            slipAngles = struct('FL', 0, 'FR', 0, 'RL', 0, 'RR', 0);
            
            % At very low speed, slip angles are undefined → return zeros
            if vx < 0.5
                return;
            end
            
            % CG-to-axle distances
            lf = wheelbase * frontWeightFrac;       % CG to front axle
            lr = wheelbase * (1 - frontWeightFrac); % CG to rear axle
            
            % Steering angle with geometry (TODO: Ackermann, rack ratio)
            delta = obj.computeSteeringAngle(steerInput);
            
            % Rear slip angle (both rear tyres identical)
            alpha_rear = -atan((vy - lr * yawRate) / vx);
            
            % Front slip angle (both front tyres identical)
            alpha_front = delta - atan((vy + lf * yawRate) / vx);
            
            slipAngles.FL = alpha_front;
            slipAngles.FR = alpha_front;
            slipAngles.RL = alpha_rear;
            slipAngles.RR = alpha_rear;
        end
        
        %% ---- Slip ratio computation ----
        
        function kappa = computeSlipRatio(obj, cornerState, vehicleSpeed)
            % COMPUTESLIPRATIO Compute longitudinal slip ratio for one corner
            %   kappa = computeSlipRatio(cornerState, vehicleSpeed)
            %
            %   Slip ratio definition:
            %     kappa = (omega * R - V) / max(|omega * R|, |V|, epsilon)
            %
            %   kappa > 0 → driving (wheel faster than vehicle)
            %   kappa < 0 → braking (wheel slower than vehicle)
            %
            %   Inputs:
            %     cornerState  - TireState with angularVelocity and wheelRadius
            %     vehicleSpeed - Vehicle forward speed [m/s]
            %
            %   Returns:
            %     kappa - Slip ratio [-1, 1]
            
            omega = cornerState.angularVelocity;
            R     = cornerState.wheelRadius;
            V     = max(vehicleSpeed, 0);   % no reverse
            
            wheelSpeed = omega * R;
            denom = max(abs(wheelSpeed), abs(V));
            
            if denom < 0.1
                % At very low speed, slip ratio is ill-defined
                kappa = 0;
            else
                kappa = (wheelSpeed - V) / denom;
            end
            
            % Clamp to [-1, 1]
            kappa = max(-1, min(1, kappa));
        end
        
        function updateWheelDynamics(obj, cornerState, driveTorque, brakeTorque, dt)
            % UPDATEWHEELDYNAMICS Integrate wheel angular velocity forward
            %   updateWheelDynamics(cornerState, driveTorque, brakeTorque, dt)
            %
            %   Rotational equation of motion:
            %     I * d(omega)/dt = T_drive - T_brake - Fx * R
            %
            %   where:
            %     T_drive = applied drive torque at this wheel [Nm]
            %     T_brake = applied brake torque at this wheel [Nm] (positive value)
            %     Fx      = longitudinal tire force from previous evaluation [N]
            %     R       = effective wheel radius [m]
            %     I       = wheel rotational inertia [kg·m^2]
            %
            %   Uses explicit Euler integration.
            %
            %   Inputs:
            %     cornerState - TireState handle (angularVelocity is mutated)
            %     driveTorque - Net drive torque at this wheel [Nm]
            %     brakeTorque - Brake torque at this wheel [Nm] (positive magnitude)
            %     dt          - Timestep [s]
            
            omega = cornerState.angularVelocity;
            R     = cornerState.wheelRadius;
            I     = obj.wheelInertia;
            Fx    = cornerState.Fx;  % from previous tire evaluation
            
            % Net torque: drive accelerates, brake and tire Fx decelerate
            % Fx > 0 means driving force → reaction torque opposes wheel spin
            netTorque = driveTorque - sign(omega) * brakeTorque - Fx * R;
            
            % Angular acceleration
            alpha = netTorque / I;
            
            % Euler integration
            omega_new = omega + alpha * dt;
            
            % Prevent wheel from spinning backwards (one-direction clutch)
            if omega_new < 0
                omega_new = 0;
            end
            
            cornerState.angularVelocity = omega_new;
        end
        
        %% ---- All-corners batch update ----
        
        function updateAllCorners(obj, Fz_FL, Fz_FR, Fz_RL, Fz_RR, ...
                slipAngle_FL, slipAngle_FR, slipAngle_RL, slipAngle_RR, ...
                kappa_FL, kappa_FR, kappa_RL, kappa_RR, mu)
            % UPDATEALLCORNERS Evaluate all four corners at once
            %   updateAllCorners(Fz_FL, Fz_FR, Fz_RL, Fz_RR, ...
            %       slipAngle_FL, slipAngle_FR, slipAngle_RL, slipAngle_RR, ...
            %       kappa_FL, kappa_FR, kappa_RL, kappa_RR, mu)
            %
            %   Updates all four corner states with per-corner slip ratios.
            %   Camber defaults to 0 for all corners.
            
            obj.updateCorner(obj.FL, Fz_FL, slipAngle_FL, kappa_FL, 0, mu);
            obj.updateCorner(obj.FR, Fz_FR, slipAngle_FR, kappa_FR, 0, mu);
            obj.updateCorner(obj.RL, Fz_RL, slipAngle_RL, kappa_RL, 0, mu);
            obj.updateCorner(obj.RR, Fz_RR, slipAngle_RR, kappa_RR, 0, mu);
        end
        
        function updateAllFromState(obj, state, vehicleManager, cornerLoads, mu)
            % UPDATEALLFROMSTATE Compute slip angles/ratios and update all corners
            %   updateAllFromState(state, vehicleManager, cornerLoads, mu)
            %
            %   Computes per-corner slip angles from vehicle kinematics and
            %   per-corner slip ratios from wheel rotational state, then
            %   delegates to updateAllCorners().
            %
            %   Inputs:
            %     state          - VehicleState with speed, vy, yawRate, steer
            %     vehicleManager - VehicleManager for geometry (wheelbase, weight dist)
            %     cornerLoads    - struct with .FL, .FR, .RL, .RR normal forces [N]
            %     mu             - Surface friction multiplier
            
            % Compute per-corner slip angles
            slipAngles = obj.computeSlipAngles( ...
                state.speed, state.vy, state.yawRate, state.steer, ...
                vehicleManager.wheelbase, vehicleManager.staticFrontWeight);
            
            % Compute per-corner slip ratios from wheel rotational state
            kappa_FL = obj.computeSlipRatio(obj.FL, state.speed);
            kappa_FR = obj.computeSlipRatio(obj.FR, state.speed);
            kappa_RL = obj.computeSlipRatio(obj.RL, state.speed);
            kappa_RR = obj.computeSlipRatio(obj.RR, state.speed);
            
            obj.updateAllCorners( ...
                cornerLoads.FL, cornerLoads.FR, cornerLoads.RL, cornerLoads.RR, ...
                slipAngles.FL, slipAngles.FR, slipAngles.RL, slipAngles.RR, ...
                kappa_FL, kappa_FR, kappa_RL, kappa_RR, mu);
        end
    end
    
    methods (Access = private)
        
        function steeringAngle = computeSteeringAngle(obj, steerInput)
            % COMPUTESTEERINGANGLE Convert driver steering input to wheel angle
            %   steeringAngle = computeSteeringAngle(steerInput)
            %
            %   TODO: Implement proper steering geometry:
            %     - Steering rack ratio
            %     - Ackermann correction (inside vs outside wheel)
            %     - Compliance effects
            %   Currently trivialized to a direct pass-through.
            
            steeringAngle = steerInput;
        end
        
        function peakMu = computePeakMuInternal(obj, Fz, gamma, P, params)
            % COMPUTEPEAKMUINTERNAL Scan lateral curve to find peak mu
            %   Vectorized: builds a matrix of 50 input rows, single mfeval call
            
            alphaScan = linspace(-0.21, 0.21, 50);  % ±12 deg in rad
            V = obj.tireConstants.refVelocity;
            nScan = numel(alphaScan);
            
            % Build inputs matrix: each row = [Fz, kappa, alpha, gamma, phit, Vx, P]
            inputsMF = [repmat(Fz, nScan, 1), ...    % Fz
                        zeros(nScan, 1), ...          % kappa = 0 (pure lateral)
                        alphaScan(:), ...             % alpha scan
                        repmat(gamma, nScan, 1), ...  % gamma
                        zeros(nScan, 1), ...          % phit = 0
                        repmat(V, nScan, 1), ...      % Vx
                        repmat(P, nScan, 1)];         % P
            
            outputs = mfeval(params, inputsMF, 111);
            peakMu = max(abs(outputs(:,2))) / Fz;
        end
    end
end