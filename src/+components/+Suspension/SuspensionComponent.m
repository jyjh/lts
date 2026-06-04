classdef (Abstract) SuspensionComponent
    % SUSPENSIONCOMPONENT Abstract interface for suspension models
    % Provides per-corner transient load computation with state tracking.
    %
    % Each implementation manages four corner units with independent
    % SuspensionState objects for transient tracking.
    
    methods (Abstract)
        % Compute per-corner tire normal forces and update transient state
        % Returns struct with .FL, .FR, .RL, .RR  [N]
        loads = computeCornerLoads(obj, state, Fz_aero_front, Fz_aero_rear, totalMass, dt)
    end
end