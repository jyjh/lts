classdef SimpleSuspension
    % SIMPLESUSPENSION Per-corner spring-damper suspension model
    % Models each corner independently with:
    %   - Heave spring: F_spring = K * x  (supports vehicle weight + aero)
    %   - Damper:       F_damp  = C * v   (asymmetric compression/rebound)
    %   - Tire spring:  F_tire  = K_tire * x_tire
    %
    % Vehicle-level geometry (trackWidth, wheelbase, cgHeight)
    % is retrieved from VehicleManager at construction time.
    %
    % This is one suspension unit for a SINGLE corner. The SuspensionManager
    % creates four instances (FL, FR, RL, RR), where front corners share
    % parameters and rear corners share parameters.
    %
    % Transient state is stored in a SuspensionState object that persists
    % across timesteps and is mutated in-place.
    
    properties
        % --- Vehicle geometry (from VehicleManager, stored at construction) ---
        trackWidth         = 1.2    % Track width [m]
        wheelbase          = 1.55   % Wheelbase [m]
        cgHeight           = 0.28   % Center of gravity height [m]
        
        % --- Suspension tuning ---
        rollStiffDist      = 0.55   % Roll stiffness distribution for this end [0-1]
        
        % --- Per-corner spring-damper ---
        springRate         = 25000  % Heave spring rate [N/m]
        dampingCoeff       = 3000   % Compression damping coefficient [N·s/m]
        reboundCoeff       = 4500   % Rebound damping coefficient [N·s/m]
        motionRatio        = 0.95   % Installation motion ratio [dimensionless]
        bumpStopLength     = 0.025  % Bump stop engagement length [m]
        bumpStopRate       = 200000 % Bump stop stiffness [N/m]
        
        % --- Tire spring ---
        tireSpringRate     = 200000 % Vertical tire stiffness [N/m]
        
        % --- Unsprung mass ---
        unsprungMass       = 25     % Per-corner unsprung mass [kg]
        
        % --- Transient state ---
        state                       % SuspensionState handle object
    end
    
    methods
        function obj = SimpleSuspension(vehicleManager, rollStiffDist, ...
                springRate, dampingCoeff, reboundCoeff, ...
                motionRatio, bumpStopLength, bumpStopRate, ...
                tireSpringRate, unsprungMass)
            % SIMPLESUSPENSION Construct a per-corner suspension unit
            %   SimpleSuspension(vehicleManager, rollStiffDist, ...
            %       springRate, dampingCoeff, reboundCoeff, ...
            %       motionRatio, bumpStopLength, bumpStopRate, ...
            %       tireSpringRate, unsprungMass)
            %
            %   vehicleManager  - VehicleManager handle (geometry pulled at construction)
            %   rollStiffDist   - Roll stiffness distribution for this end [0-1]
            %   springRate      - Heave spring rate [N/m]
            %   dampingCoeff    - Compression damping [N·s/m]
            %   reboundCoeff    - Rebound damping [N·s/m]
            %   motionRatio     - Installation motion ratio
            %   bumpStopLength  - Bump stop travel before engagement [m]
            %   bumpStopRate    - Bump stop stiffness [N/m]
            %   tireSpringRate  - Vertical tire stiffness [N/m]
            %   unsprungMass    - Per-corner unsprung mass [kg]
            
            % Pull vehicle-level geometry from VehicleManager
            obj.trackWidth   = vehicleManager.trackWidth;
            obj.wheelbase    = vehicleManager.wheelbase;
            obj.cgHeight     = vehicleManager.cgHeight;
            
            % Store suspension-specific parameters
            obj.rollStiffDist     = rollStiffDist;
            obj.springRate        = springRate;
            obj.dampingCoeff      = dampingCoeff;
            obj.reboundCoeff      = reboundCoeff;
            obj.motionRatio       = motionRatio;
            obj.bumpStopLength    = bumpStopLength;
            obj.bumpStopRate      = bumpStopRate;
            obj.tireSpringRate    = tireSpringRate;
            obj.unsprungMass      = unsprungMass;
            
            % Initialize transient state
            obj.state = components.Suspension.SuspensionState();
            obj.state.motionRatioEffective = obj.motionRatio;
        end
        
        function updateCorner(obj, cornerState, demandedLoad, dt)
            % UPDATECORNER Update one corner's transient state
            %   updateCorner(cornerState, demandedLoad, dt)
            %
            %   cornerState  - SuspensionState handle for this corner
            %   demandedLoad - Total static + aero + load-transfer force [N]
            %   dt           - Timestep [s]
            %
            %   Mutates cornerState in-place, updating:
            %     .damperPosition, .damperVelocity, .tireDeflection,
            %     .tireNormalForce, .suspensionForce, .demandedLoad
            
            cornerState.demandedLoad = demandedLoad;
            
            % Previous state
            x_prev = cornerState.damperPosition;
            v_prev = cornerState.damperVelocity;
            
            % Effective spring and damping (through motion ratio)
            MR_eff = obj.motionRatio;
            if isprop(cornerState, 'motionRatioEffective') && ...
                    cornerState.motionRatioEffective > 0
                MR_eff = cornerState.motionRatioEffective;
            end

            K_eff = obj.springRate * MR_eff^2;
            K_tire = obj.tireSpringRate;
            
            % Asymmetric damping: compression vs rebound
            if v_prev >= 0
                C_eff = obj.dampingCoeff * MR_eff^2;
            else
                C_eff = obj.reboundCoeff * MR_eff^2;
            end
            
            % --- Forces on the sprung mass ---
            % Spring restoring force (positive = resists compression)
            F_spring = K_eff * x_prev;
            
            % Damper force
            F_damper = C_eff * v_prev;
            
            % Bump stop force (engages when compression > bumpStopLength)
            F_bumpstop = 0;
            if x_prev > obj.bumpStopLength
                F_bumpstop = obj.bumpStopRate * (x_prev - obj.bumpStopLength);
            end
            
            % --- Unsprung mass equation of motion ---
            % F_net = demandedLoad - spring - damper - bumpstop
            F_net = demandedLoad - F_spring - F_damper - F_bumpstop;
            
            % Acceleration of unsprung mass
            x_ddot = F_net / obj.unsprungMass;
            
            % Semi-implicit Euler integration
            v_new = v_prev + x_ddot * dt;
            x_new = x_prev + v_new * dt;
            
            % --- Update state ---
            cornerState.damperPosition = x_new;
            cornerState.damperVelocity = v_new;
            
            % Contact-patch normal load comes from the vehicle load-transfer
            % balance. Damper position is suspension travel, not tire
            % deflection; using it as tire deflection inflates Fz by the
            % tire/spring-rate ratio and breaks the tire model.
            cornerState.tireNormalForce = max(demandedLoad, 0);
            cornerState.tireDeflection = cornerState.tireNormalForce / K_tire;
            
            % Store total suspension force for logging
            cornerState.suspensionForce = F_spring + F_damper + F_bumpstop;
        end
    end
end
