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

        % Minimum normal load passed into MFeval [N]. Returned forces are
        % scaled back to the actual normal load for lightly loaded tires.
        minEvaluationLoad = 100

        % Tire relaxation lengths. These make slip angle/ratio build over
        % distance instead of appearing instantly at the contact patch.
        enableRelaxation = true
        lateralRelaxationLength = 2.5      % [m]
        longitudinalRelaxationLength = 0.8 % [m]
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
            %   In this codebase, mu is treated as an absolute surface grip cap.
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
                cornerState.frictionLimit = 0;
                cornerState.frictionUsage = 0;
                return;
            end
            
            % Unpack for MFeval call
            kappa = slipRatio;
            alpha = slipAngle;
            Fz    = max(normalLoad, obj.minEvaluationLoad);
            gamma = camberAngle;
            V     = obj.tireConstants.refVelocity;
            P     = obj.tireConstants.nomPressure;
            params = obj.tireConstants.params;
            loadScale = normalLoad / Fz;
            
            % Build MFeval inputs row: [Fz, kappa, alpha, gamma, phit, Vx, P]
            inputsMF = [Fz, kappa, alpha, gamma, 0, V, P];
            
            % Evaluate Pacejka Magic Formula via MFeval (useMode=111: combined)
            outputs = mfeval(params, inputsMF, 111);
            
            rawPeakMu = obj.computePeakMuInternal(Fz, gamma, P, params);
            surfaceScale = obj.computeSurfaceScale(rawPeakMu, mu);

            % Store outputs capped by the current surface friction coefficient.
            cornerState.Fy = -outputs(:,2) * surfaceScale * loadScale;
            cornerState.Fx = outputs(:,1) * surfaceScale * loadScale;
            cornerState.Mx = outputs(:,4) * surfaceScale * loadScale;
            cornerState.My = outputs(:,5) * surfaceScale * loadScale;
            cornerState.Mz = outputs(:,6) * surfaceScale * loadScale;
            cornerState.peakMu = rawPeakMu * surfaceScale;
            cornerState.frictionLimit = cornerState.peakMu * normalLoad;
            forceMagnitude = hypot(cornerState.Fx, cornerState.Fy);
            if cornerState.frictionLimit > 0 && forceMagnitude > cornerState.frictionLimit
                limitScale = cornerState.frictionLimit / forceMagnitude;
                cornerState.Fx = cornerState.Fx * limitScale;
                cornerState.Fy = cornerState.Fy * limitScale;
                cornerState.Mx = cornerState.Mx * limitScale;
                cornerState.My = cornerState.My * limitScale;
                cornerState.Mz = cornerState.Mz * limitScale;
                forceMagnitude = cornerState.frictionLimit;
            end
            if cornerState.frictionLimit > 0
                cornerState.frictionUsage = forceMagnitude / cornerState.frictionLimit;
            else
                cornerState.frictionUsage = 0;
            end
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
            
            evalLoad = max(normalLoad, obj.minEvaluationLoad);
            loadScale = normalLoad / evalLoad;
            inputsMF = [evalLoad, 0, slipAngle, 0, 0, ...
                obj.tireConstants.refVelocity, obj.tireConstants.nomPressure];
            outputs = mfeval(obj.tireConstants.params, inputsMF, 111);
            
            rawPeakMu = obj.computePeakMuInternal(evalLoad, 0, ...
                obj.tireConstants.nomPressure, obj.tireConstants.params);
            surfaceScale = obj.computeSurfaceScale(rawPeakMu, mu);
            Fy = -outputs(:,2) * surfaceScale * loadScale;
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
            
            evalLoad = max(normalLoad, obj.minEvaluationLoad);
            loadScale = normalLoad / evalLoad;
            inputsMF = [evalLoad, slipRatio, 0, 0, 0, ...
                obj.tireConstants.refVelocity, obj.tireConstants.nomPressure];
            outputs = mfeval(obj.tireConstants.params, inputsMF, 111);
            
            rawPeakMu = obj.computePeakMuInternal(evalLoad, 0, ...
                obj.tireConstants.nomPressure, obj.tireConstants.params);
            surfaceScale = obj.computeSurfaceScale(rawPeakMu, mu);
            Fx = outputs(:,1) * surfaceScale * loadScale;
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
            
            evalLoad = max(normalLoad, obj.minEvaluationLoad);
            peakMu = obj.computePeakMuInternal(evalLoad, 0, ...
                obj.tireConstants.nomPressure, obj.tireConstants.params);
        end
        
        %% ---- Slip angle computation ----
        
        function slipAngles = computeSlipAngles(obj, vx, vy, yawRate, steerInput, wheelbase, frontWeightFrac)
            % COMPUTESLIPANGLES Compute per-corner tire slip angles [rad]
            %   slipAngles = computeSlipAngles(vx, vy, yawRate, steerInput, wheelbase, frontWeightFrac)
            %
            %   Uses per-corner wheel-plane kinematics. When track width is
            %   supplied through updateAllFromState, the front wheels use
            %   simple Ackermann steering angles.
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

            wheelKinematics = obj.computeWheelKinematics( ...
                vx, vy, yawRate, steerInput, wheelbase, frontWeightFrac, 0);
            slipAngles = struct('FL', wheelKinematics.FL.slipAngle, ...
                'FR', wheelKinematics.FR.slipAngle, ...
                'RL', wheelKinematics.RL.slipAngle, ...
                'RR', wheelKinematics.RR.slipAngle);
            
            % At very low speed, slip angles are undefined → return zeros
            if vx < 0.5
                return;
            end
            
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
        
        function updateAllFromState(obj, state, vehicleManager, cornerLoads, mu, dt)
            % UPDATEALLFROMSTATE Compute slip angles/ratios and update all corners
            %   updateAllFromState(state, vehicleManager, cornerLoads, mu, dt)
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
            %                      Treated as an absolute surface grip cap here.
            %     dt             - Timestep [s] for tire relaxation
            if nargin < 6
                dt = 0;
            end
            
            % Compute per-corner slip angles and local wheel-plane speeds
            wheelKinematics = obj.computeWheelKinematics( ...
                state.speed, state.vy, state.yawRate, state.steer, ...
                vehicleManager.wheelbase, vehicleManager.staticFrontWeight, ...
                vehicleManager.trackWidth);
            
            % Compute per-corner slip ratios from wheel rotational state
            kappa_FL = obj.computeSlipRatio(obj.FL, wheelKinematics.FL.longitudinalSpeed);
            kappa_FR = obj.computeSlipRatio(obj.FR, wheelKinematics.FR.longitudinalSpeed);
            kappa_RL = obj.computeSlipRatio(obj.RL, wheelKinematics.RL.longitudinalSpeed);
            kappa_RR = obj.computeSlipRatio(obj.RR, wheelKinematics.RR.longitudinalSpeed);

            [alpha_FL, kappa_FL] = obj.relaxSlipInputs( ...
                obj.FL, wheelKinematics.FL.slipAngle, kappa_FL, state.speed, dt);
            [alpha_FR, kappa_FR] = obj.relaxSlipInputs( ...
                obj.FR, wheelKinematics.FR.slipAngle, kappa_FR, state.speed, dt);
            [alpha_RL, kappa_RL] = obj.relaxSlipInputs( ...
                obj.RL, wheelKinematics.RL.slipAngle, kappa_RL, state.speed, dt);
            [alpha_RR, kappa_RR] = obj.relaxSlipInputs( ...
                obj.RR, wheelKinematics.RR.slipAngle, kappa_RR, state.speed, dt);
            
            obj.updateAllCorners( ...
                cornerLoads.FL, cornerLoads.FR, cornerLoads.RL, cornerLoads.RR, ...
                alpha_FL, alpha_FR, alpha_RL, alpha_RR, ...
                kappa_FL, kappa_FR, kappa_RL, kappa_RR, mu);
        end
    end
    
    methods (Access = private)
        
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

        function surfaceScale = computeSurfaceScale(obj, rawPeakMu, surfaceMu)
            % COMPUTESURFACESCALE Scale tire forces so surface mu is an absolute cap.
            surfaceMu = max(surfaceMu, 0);
            if rawPeakMu <= 0
                surfaceScale = 0;
            else
                surfaceScale = min(1, surfaceMu / rawPeakMu);
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
