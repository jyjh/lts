classdef (Abstract) SuspensionComponent
    % SUSPENSIONCOMPONENT Abstract interface for suspension models
    % Provides per-corner transient load computation with state tracking.
    %
    % Each implementation manages four corner units with independent
    % SuspensionState objects for transient tracking.

    properties (Abstract)
        frontLeft
        frontRight
        rearLeft
        rearRight
    end
    
    methods (Abstract)
        % Settle suspension state to static equilibrium before simulation
        warmup(obj, totalMass, dt)

        % Compute per-corner tire normal forces and update transient state
        % Returns struct with .FL, .FR, .RL, .RR  [N]
        loads = computeCornerLoads(obj, state, Fz_aero_front, Fz_aero_rear, totalMass, dt)

        % Compute body pitch angle from current suspension state [rad]
        pitchAngle = computePitchAngle(obj)
    end
end
