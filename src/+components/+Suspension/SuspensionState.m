classdef SuspensionState < handle
    % SUSPENSIONSTATE Mutable per-corner suspension transient state
    % Holds the dynamic state variables for one corner of the car.
    % Uses handle inheritance so that SimpleSuspension can mutate state
    % in-place across timesteps.
    %
    % All displacements are measured from static equilibrium.
    % Positive = compression (bump).
    
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

        % Wheel travel from static equilibrium [m]
        % Positive = bump/compression
        wheelTravel      = 0

        % Tire inclination angle from suspension geometry [rad]
        camberAngle      = 0

        % Static/compliance toe angle from suspension geometry [rad]
        toeAngle         = 0

        % Road-wheel steering angle from steering geometry [rad]
        steerAngle       = 0

        % Effective installation motion ratio from geometry [-]
        motionRatioEffective = 1
        
        % Net spring + damper force acting on sprung mass [N]
        suspensionForce  = 0
        
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
            obj.wheelTravel     = 0;
            obj.camberAngle     = 0;
            obj.toeAngle        = 0;
            obj.steerAngle      = 0;
            obj.motionRatioEffective = 1;
            obj.suspensionForce = 0;
            obj.demandedLoad    = 0;
        end
        
        function reset(obj)
            % RESET Reset state to zero (static equilibrium)
            obj.damperPosition  = 0;
            obj.damperVelocity  = 0;
            obj.tireDeflection  = 0;
            obj.tireNormalForce = 0;
            obj.wheelTravel     = 0;
            obj.camberAngle     = 0;
            obj.toeAngle        = 0;
            obj.steerAngle      = 0;
            obj.motionRatioEffective = 1;
            obj.suspensionForce = 0;
            obj.demandedLoad    = 0;
        end
    end
end
