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
            cornerState.Fy = outputs.Fy * mu;
            cornerState.Fx = outputs.Fx * mu;
            cornerState.Mx = outputs.Mx * mu;
            cornerState.My = outputs.My * mu;
            cornerState.Mz = outputs.Mz * mu;
            
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
        
        %% ---- All-corners batch update ----
        
        function updateAllCorners(obj, Fz_FL, Fz_FR, Fz_RL, Fz_RR, ...
                slipAngle_FL, slipAngle_FR, slipAngle_RL, slipAngle_RR, ...
                slipRatio, mu)
            % UPDATEALLCORNERS Evaluate all four corners at once
            %   updateAllCorners(Fz_FL, Fz_FR, Fz_RL, Fz_RR, ...
            %       slipAngle_FL, slipAngle_FR, slipAngle_RL, slipAngle_RR, ...
            %       slipRatio, mu)
            %
            %   Convenience method that updates all four corner states.
            %   Assumes same slip ratio and surface friction for all corners.
            %   Camber defaults to 0 for all corners.
            
            obj.updateCorner(obj.FL, Fz_FL, slipAngle_FL, slipRatio, 0, mu);
            obj.updateCorner(obj.FR, Fz_FR, slipAngle_FR, slipRatio, 0, mu);
            obj.updateCorner(obj.RL, Fz_RL, slipAngle_RL, slipRatio, 0, mu);
            obj.updateCorner(obj.RR, Fz_RR, slipAngle_RR, slipRatio, 0, mu);
        end
    end
    
    methods (Access = private)
        
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