classdef SuspensionState < handle
    % SUSPENSIONSTATE Mutable per-corner suspension transient state
    % Holds the dynamic state variables for one corner of the car.
    % Uses handle inheritance so that SimpleSuspension can mutate state
    % in-place across timesteps.
    %
    % All displacements are measured from static equilibrium.
    % Positive = compression (bump). Static weight is carried separately as
    % baseline normal load; this state adds only transient spring/damper force.
    
    properties
        % Damper compression from static equilibrium [m]
        % Positive = suspension compressed (bump)
        damperPosition   = 0
        
        % Damper compression velocity [m/s]
        % Positive = compressing
        damperVelocity   = 0
        
        % Tire vertical deflection [m]
        tireDeflection   = 0
        
        % Normal force at tire contact patch [N]
        tireNormalForce  = 0
        
        % Net spring + damper force acting on sprung mass [N]
        suspensionForce  = 0

        % Anti-roll-bar vertical force contribution at this corner [N]
        % Positive increases tire normal force, negative unloads the tire.
        antiRollForce    = 0
        
        % Total demanded load on this corner (before suspension filtering) [N]
        demandedLoad     = 0
    end
    
	    methods
	        function obj = SuspensionState()
	            % SUSPENSIONSTATE Construct with zero initial conditions
	            obj.damperPosition  = 0;
            obj.damperVelocity  = 0;
            obj.tireDeflection  = 0;
            obj.tireNormalForce = 0;
            obj.suspensionForce = 0;
            obj.antiRollForce   = 0;
	            obj.demandedLoad    = 0;
	        end

	        function set.damperPosition(obj, value)
	            obj.damperPosition = utils.scalarOrDefault(value, 0);
	        end

	        function set.damperVelocity(obj, value)
	            obj.damperVelocity = utils.scalarOrDefault(value, 0);
	        end

	        function set.tireDeflection(obj, value)
	            obj.tireDeflection = utils.nonnegativeScalarOrDefault(value, 0);
	        end

	        function set.tireNormalForce(obj, value)
	            obj.tireNormalForce = utils.nonnegativeScalarOrDefault(value, 0);
	        end

	        function set.suspensionForce(obj, value)
	            obj.suspensionForce = utils.scalarOrDefault(value, 0);
	        end

	        function set.antiRollForce(obj, value)
	            obj.antiRollForce = utils.scalarOrDefault(value, 0);
	        end

	        function set.demandedLoad(obj, value)
	            obj.demandedLoad = utils.scalarOrDefault(value, 0);
	        end

	        function reset(obj)
	            % RESET Reset state to zero (static equilibrium)
	            obj.damperPosition  = 0;
            obj.damperVelocity  = 0;
            obj.tireDeflection  = 0;
            obj.tireNormalForce = 0;
            obj.suspensionForce = 0;
            obj.antiRollForce   = 0;
	            obj.demandedLoad    = 0;
	        end
	    end
	end
