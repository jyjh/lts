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

        % Cache peak-mu scans by rounded load/camber.
        peakMuCache
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
            obj.peakMuCache = containers.Map('KeyType', 'char', 'ValueType', 'double');
            
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
            slipAngle = max(-0.3, min(0.3, slipAngle));
            slipRatio = max(-1, min(1, slipRatio));

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
            
            % Evaluate Pacejka Magic Formula via MFeval (useMode=111: combined)
            outputs = mfeval(params, inputsMF, 111);
            
            rawPeakMu = obj.getCachedPeakMu(Fz, gamma, P, params);
            surfaceScale = obj.computeSurfaceScale(rawPeakMu, mu);

            % Store outputs capped by the current surface friction coefficient.
            cornerState.Fy = -outputs(:,2) * surfaceScale;
            cornerState.Fx = outputs(:,1) * surfaceScale;
            cornerState.Mx = outputs(:,4) * surfaceScale;
            cornerState.My = outputs(:,5) * surfaceScale;
            cornerState.Mz = outputs(:,6) * surfaceScale;
            cornerState.peakMu = rawPeakMu * surfaceScale;
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
            
            rawPeakMu = obj.computePeakMuInternal(normalLoad, 0, ...
                obj.tireConstants.nomPressure, obj.tireConstants.params);
            surfaceScale = obj.computeSurfaceScale(rawPeakMu, mu);
            Fy = -outputs(:,2) * surfaceScale;
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
            outputs = mfeval(obj.tireConstants.params, inputsMF, 111);
            
            rawPeakMu = obj.computePeakMuInternal(normalLoad, 0, ...
                obj.tireConstants.nomPressure, obj.tireConstants.params);
            surfaceScale = obj.computeSurfaceScale(rawPeakMu, mu);
            Fx = outputs(:,1) * surfaceScale;
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
            
            peakMu = obj.getCachedPeakMu(normalLoad, 0, ...
                obj.tireConstants.nomPressure, obj.tireConstants.params);
        end
        
        %% ---- Slip angle computation ----
        
        function slipAngles = computeSlipAngles(obj, vx, vy, yawRate, steerInput, vehicleManager)
            % COMPUTESLIPANGLES Compute per-corner tire slip angles [rad]
            %   slipAngles = computeSlipAngles(vx, vy, yawRate, steerInput, vehicleManager)
            %
            %   Uses per-corner wheel kinematics:
            %     alpha_i = steer_i + toe_i - atan2(vy_i, vx_i)
            %
            %   steer_i and toe_i come from the suspension geometry model,
            %   allowing Ackermann, bump steer, rear steer, and toe curves.
            %
            %   Inputs:
            %     vx              - forward velocity [m/s]
            %     vy              - lateral velocity at CG [m/s]
            %     yawRate         - yaw rate [rad/s]
            %     steerInput      - driver steering input [rad]
            %     vehicleManager  - vehicle/component manager with geometry
            %
            %   Returns struct with:
            %     slipAngles.FL, .FR, .RL, .RR  [rad]

            slipAngles = struct('FL', 0, 'FR', 0, 'RL', 0, 'RR', 0);
            
            % At very low speed, slip angles are undefined → return zeros
            if vx < 0.5
                return;
            end
            
            suspensionKinematics = obj.getSuspensionKinematics(vehicleManager, steerInput);
            [xFL, yFL] = obj.getWheelPosition(vehicleManager, 'FL');
            [xFR, yFR] = obj.getWheelPosition(vehicleManager, 'FR');
            [xRL, yRL] = obj.getWheelPosition(vehicleManager, 'RL');
            [xRR, yRR] = obj.getWheelPosition(vehicleManager, 'RR');

            slipAngles.FL = obj.computeCornerSlipAngle(vx, vy, yawRate, ...
                xFL, yFL, suspensionKinematics.FL);
            slipAngles.FR = obj.computeCornerSlipAngle(vx, vy, yawRate, ...
                xFR, yFR, suspensionKinematics.FR);
            slipAngles.RL = obj.computeCornerSlipAngle(vx, vy, yawRate, ...
                xRL, yRL, suspensionKinematics.RL);
            slipAngles.RR = obj.computeCornerSlipAngle(vx, vy, yawRate, ...
                xRR, yRR, suspensionKinematics.RR);
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

            slipSpeedFloor = 1.0;
            rawKappa = (wheelSpeed - V) / max(denom, slipSpeedFloor);
            if denom < slipSpeedFloor
                previousKappa = cornerState.slipRatio;
                if ~isfinite(previousKappa)
                    previousKappa = rawKappa;
                end
                blend = denom / slipSpeedFloor;
                kappa = (1 - blend) * previousKappa + blend * rawKappa;
            else
                kappa = rawKappa;
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
                kappa_FL, kappa_FR, kappa_RL, kappa_RR, mu, ...
                camber_FL, camber_FR, camber_RL, camber_RR)
            % UPDATEALLCORNERS Evaluate all four corners at once
            %   updateAllCorners(Fz_FL, Fz_FR, Fz_RL, Fz_RR, ...
            %       slipAngle_FL, slipAngle_FR, slipAngle_RL, slipAngle_RR, ...
            %       kappa_FL, kappa_FR, kappa_RL, kappa_RR, mu)
            %
            %   Updates all four corner states with per-corner slip ratios.
            %   Camber defaults to 0 for all corners.
            
            if nargin < 15
                camber_FL = 0;
                camber_FR = 0;
                camber_RL = 0;
                camber_RR = 0;
            end

            Fz = [Fz_FL; Fz_FR; Fz_RL; Fz_RR];
            alpha = max(-0.3, min(0.3, ...
                [slipAngle_FL; slipAngle_FR; slipAngle_RL; slipAngle_RR]));
            kappa = max(-1, min(1, [kappa_FL; kappa_FR; kappa_RL; kappa_RR]));
            gamma = [camber_FL; camber_FR; camber_RL; camber_RR];
            states = {obj.FL, obj.FR, obj.RL, obj.RR};

            for i = 1:4
                states{i}.normalForce = Fz(i);
                states{i}.slipAngle = alpha(i);
                states{i}.slipRatio = kappa(i);
                states{i}.camberAngle = gamma(i);
            end

            active = Fz > 0;
            if any(active)
                P = obj.tireConstants.nomPressure;
                V = obj.tireConstants.refVelocity;
                params = obj.tireConstants.params;
                nActive = nnz(active);
                inputsMF = [Fz(active), kappa(active), alpha(active), ...
                    gamma(active), zeros(nActive, 1), ...
                    repmat(V, nActive, 1), repmat(P, nActive, 1)];
                outputs = mfeval(params, inputsMF, 111);

                activeIdx = find(active);
                for j = 1:numel(activeIdx)
                    i = activeIdx(j);
                    rawPeakMu = obj.getCachedPeakMu(Fz(i), gamma(i), P, params);
                    surfaceScale = obj.computeSurfaceScale(rawPeakMu, mu);
                    states{i}.Fx = outputs(j,1) * surfaceScale;
                    states{i}.Fy = -outputs(j,2) * surfaceScale;
                    states{i}.Mx = outputs(j,4) * surfaceScale;
                    states{i}.My = outputs(j,5) * surfaceScale;
                    states{i}.Mz = outputs(j,6) * surfaceScale;
                    states{i}.peakMu = rawPeakMu * surfaceScale;
                end
            end

            inactiveIdx = find(~active);
            for j = 1:numel(inactiveIdx)
                i = inactiveIdx(j);
                states{i}.Fx = 0;
                states{i}.Fy = 0;
                states{i}.Mx = 0;
                states{i}.My = 0;
                states{i}.Mz = 0;
                states{i}.peakMu = 0;
            end
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
            %                      Treated as an absolute surface grip cap here.
            
            % Compute per-corner slip angles and suspension geometry
            slipAngles = obj.computeSlipAngles( ...
                state.speed, state.vy, state.yawRate, state.steer, ...
                vehicleManager);
            suspensionKinematics = obj.getSuspensionKinematics(vehicleManager, state.steer);
            
            % Compute per-corner slip ratios from wheel rotational state
            kappa_FL = obj.computeSlipRatio(obj.FL, state.speed);
            kappa_FR = obj.computeSlipRatio(obj.FR, state.speed);
            kappa_RL = obj.computeSlipRatio(obj.RL, state.speed);
            kappa_RR = obj.computeSlipRatio(obj.RR, state.speed);
            
            obj.updateAllCorners( ...
                cornerLoads.FL, cornerLoads.FR, cornerLoads.RL, cornerLoads.RR, ...
                slipAngles.FL, slipAngles.FR, slipAngles.RL, slipAngles.RR, ...
                kappa_FL, kappa_FR, kappa_RL, kappa_RR, mu, ...
                suspensionKinematics.FL.camberAngle, ...
                suspensionKinematics.FR.camberAngle, ...
                suspensionKinematics.RL.camberAngle, ...
                suspensionKinematics.RR.camberAngle);
        end
    end
    
    methods (Access = private)
        function suspensionKinematics = getSuspensionKinematics(~, vehicleManager, steerInput)
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

        function [x, y] = getWheelPosition(~, vehicleManager, corner)
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

        function alpha = computeCornerSlipAngle(~, vx, vy, yawRate, x, y, kin)
            vxCorner = vx - yawRate * y;
            vyCorner = vy + yawRate * x;
            wheelHeading = kin.steerAngle + kin.toeAngle;
            longSpeed = vxCorner * cos(wheelHeading) + vyCorner * sin(wheelHeading);
            latSpeed = -vxCorner * sin(wheelHeading) + vyCorner * cos(wheelHeading);
            alpha = atan2(-latSpeed, max(abs(longSpeed), 0.1));
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

        function peakMu = getCachedPeakMu(obj, Fz, gamma, P, params)
            FzKey = round(Fz / 10) * 10;
            gammaKey = round(gamma * 1000) / 1000;
            key = sprintf('%.0f_%.3f_%.0f', FzKey, gammaKey, P);
            if isKey(obj.peakMuCache, key)
                peakMu = obj.peakMuCache(key);
                return;
            end

            peakMu = obj.computePeakMuInternal(Fz, gamma, P, params);
            obj.peakMuCache(key) = peakMu;
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
