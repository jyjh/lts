classdef (Abstract) SuspensionComponent
    % SUSPENSIONCOMPONENT Abstract interface for suspension models
    % Provides load transfer computation and roll/pitch characteristics
    
    methods (Abstract)
        % Lateral load transfer [N] given lateral acceleration
        % Returns struct with .front and .rear load transfer
        latTransfer = computeLatLoadTransfer(obj, ay, totalMass)
        
        % Longitudinal load transfer [N] given longitudinal acceleration
        % Returns struct with .front and .rear load transfer (front is negative under braking)
        longTransfer = computeLongLoadTransfer(obj, ax, totalMass)
        
        % Roll stiffness distribution [0-1], fraction on front
        rollStiffDist = getRollStiffnessDistribution(obj)
        
        % Static weight distribution [0-1], fraction on front
        staticFrontWeight = getStaticWeightDistribution(obj)
    end
end