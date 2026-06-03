classdef (Abstract) PowertrainComponent
    % POWERTRAINCOMPONENT Abstract interface for powertrain models
    % Provides drive force computation based on speed and throttle
    
    methods (Abstract)
        % Drive force at wheels [N] given vehicle speed and throttle [0-1]
        F_drive = computeDriveForce(obj, speed, throttle)
        
        % Maximum engine torque [Nm] at given engine speed [rpm]
        maxTorque = getMaxTorque(obj, engineSpeed)
        
        % Total gear ratio (including final drive)
        totalRatio = getTotalGearRatio(obj)
        
        % Drivetrain efficiency [0-1]
        efficiency = getDrivetrainEfficiency(obj)
    end
end