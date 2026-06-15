classdef SimpleSuspension < handle
    % SIMPLESUSPENSION Per-corner spring-damper suspension model
    % Models each corner independently with:
    %   - Heave spring: F_spring = K * x  (incremental force from static ride)
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
            
            % Store suspension-specific parameters. Reject malformed setup
            % values instead of letting negative rates or singular masses
            % invert the corner load response.
            obj.rollStiffDist = utils.unitScalarOrDefault( ...
                rollStiffDist, obj.rollStiffDist);
            obj.springRate = utils.nonnegativeScalarOrDefault( ...
                springRate, obj.springRate);
            obj.dampingCoeff = utils.nonnegativeScalarOrDefault( ...
                dampingCoeff, obj.dampingCoeff);
            obj.reboundCoeff = utils.nonnegativeScalarOrDefault( ...
                reboundCoeff, obj.reboundCoeff);
            obj.motionRatio = utils.nonnegativeScalarOrDefault( ...
                motionRatio, obj.motionRatio);
            obj.bumpStopLength = utils.nonnegativeScalarOrDefault( ...
                bumpStopLength, obj.bumpStopLength);
            obj.bumpStopRate = utils.nonnegativeScalarOrDefault( ...
                bumpStopRate, obj.bumpStopRate);
            obj.tireSpringRate = utils.positiveScalarOrDefault( ...
                tireSpringRate, obj.tireSpringRate);
            obj.unsprungMass = utils.positiveScalarOrDefault( ...
                unsprungMass, obj.unsprungMass);
            
            % Initialize transient state
            obj.state = components.Suspension.SuspensionState();
        end

        function syncVehicleGeometry(obj, vehicleManager)
            % SYNCVEHICLEGEOMETRY Refresh copied geometry from VehicleManager.
            %
            % Each corner stores wheelbase, track width, and CG height locally
            % for load-transfer and roll-angle calculations. Keeping those
            % copies synchronized avoids using stale lever arms after a setup
            % change or an invariant test mutates VehicleManager geometry.
            if nargin < 2 || isempty(vehicleManager)
                return;
            end

            obj.trackWidth = vehicleManager.trackWidth;
            obj.wheelbase = vehicleManager.wheelbase;
            obj.cgHeight = vehicleManager.cgHeight;
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
            
	            demandedLoad = utils.scalarOrDefault(demandedLoad, 0);
	            dt = utils.nonnegativeScalarOrDefault(dt, 0);
	            springRate = utils.nonnegativeScalarOrDefault( ...
	                obj.springRate, 25000);
	            dampingCoeff = utils.nonnegativeScalarOrDefault( ...
	                obj.dampingCoeff, 3000);
	            reboundCoeff = utils.nonnegativeScalarOrDefault( ...
	                obj.reboundCoeff, 4500);
	            motionRatio = utils.nonnegativeScalarOrDefault( ...
	                obj.motionRatio, 0.95);
	            bumpStopLength = utils.nonnegativeScalarOrDefault( ...
	                obj.bumpStopLength, 0.025);
	            bumpStopRate = utils.nonnegativeScalarOrDefault( ...
	                obj.bumpStopRate, 200000);
	            tireSpringRate = utils.positiveScalarOrDefault( ...
	                obj.tireSpringRate, 200000);
	            unsprungMass = utils.positiveScalarOrDefault( ...
	                obj.unsprungMass, 25);

	            cornerState.demandedLoad = demandedLoad;

	            % Previous state
	            x_prev = cornerState.damperPosition;
	            v_prev = cornerState.damperVelocity;

	            % Effective spring and damping (through motion ratio)
	            K_eff = springRate * motionRatio^2;
	            K_tire = tireSpringRate;

	            % Asymmetric damping: compression vs rebound
	            if v_prev >= 0
	                C_eff = dampingCoeff * motionRatio^2;
	            else
	                C_eff = reboundCoeff * motionRatio^2;
	            end
            
            % --- Forces on the sprung mass ---
            % Spring restoring force (positive = resists compression)
            F_spring = K_eff * x_prev;
            
            % Damper force
            F_damper = C_eff * v_prev;
            
	            % Bump stop force (engages when compression > bumpStopLength)
	            F_bumpstop = 0;
	            if x_prev > bumpStopLength
	                F_bumpstop = bumpStopRate * (x_prev - bumpStopLength);
	            end
            
            % --- Unsprung mass equation of motion ---
            % F_net = demandedLoad - spring - damper - bumpstop
            F_net = demandedLoad - F_spring - F_damper - F_bumpstop;
            
	            % Acceleration of unsprung mass
	            x_ddot = F_net / unsprungMass;
            
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
            cornerState.antiRollForce = 0;
        end

	        function updateCornerFromKinematics(obj, cornerState, baseLoad, chassisDisplacement, chassisVelocity)
            % UPDATECORNERFROMKINEMATICS Update tire load from chassis motion
            %   baseLoad              - Static corner load [N]
            %   chassisDisplacement   - Corner body motion from static [m]
            %                           positive = compression-producing
            %   chassisVelocity       - Corner body velocity [m/s]
            %                           positive = compressing
            %
            % This path is used when the chassis owns heave/pitch/roll
            % dynamics. The suspension no longer invents load transfer from
            % ax/ay directly; spring and damper forces emerge from the
            % chassis corner motion.

	            baseLoad = utils.nonnegativeScalarOrDefault(baseLoad, 0);
	            x = utils.scalarOrDefault(chassisDisplacement, 0);
	            v = utils.scalarOrDefault(chassisVelocity, 0);
	            springRate = utils.nonnegativeScalarOrDefault( ...
	                obj.springRate, 25000);
	            dampingCoeff = utils.nonnegativeScalarOrDefault( ...
	                obj.dampingCoeff, 3000);
	            reboundCoeff = utils.nonnegativeScalarOrDefault( ...
	                obj.reboundCoeff, 4500);
	            motionRatio = utils.nonnegativeScalarOrDefault( ...
	                obj.motionRatio, 0.95);
	            bumpStopLength = utils.nonnegativeScalarOrDefault( ...
	                obj.bumpStopLength, 0.025);
	            bumpStopRate = utils.nonnegativeScalarOrDefault( ...
	                obj.bumpStopRate, 200000);
	            tireSpringRate = utils.positiveScalarOrDefault( ...
	                obj.tireSpringRate, 200000);

	            K_eff = springRate * motionRatio^2;
	            if v >= 0
	                C_eff = dampingCoeff * motionRatio^2;
	            else
	                C_eff = reboundCoeff * motionRatio^2;
	            end

            F_spring = K_eff * x;
            F_damper = C_eff * v;

	            F_bumpstop = 0;
	            if x > bumpStopLength
	                F_bumpstop = bumpStopRate * (x - bumpStopLength);
	            end

            dynamicLoad = F_spring + F_damper + F_bumpstop;
            tireLoad = max(baseLoad + dynamicLoad, 0);

	            cornerState.damperPosition = x;
	            cornerState.damperVelocity = v;
	            cornerState.tireDeflection = tireLoad / max(tireSpringRate, eps);
            cornerState.tireNormalForce = tireLoad;
            cornerState.suspensionForce = dynamicLoad;
            cornerState.antiRollForce = 0;
            cornerState.demandedLoad = baseLoad + dynamicLoad;
        end
    end
end
