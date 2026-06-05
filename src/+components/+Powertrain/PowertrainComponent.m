classdef (Abstract) PowertrainComponent
    % POWERTRAINCOMPONENT Abstract interface for powertrain models
    % Provides drive force computation and motor speed state tracking.

    properties (Abstract)
        state  % components.Powertrain.PowertrainState
    end
    
    methods (Abstract)
        % Drive force at wheels [N] given vehicle speed and throttle [0-1]
        F_drive = computeDriveForce(obj, speed, throttle)

        % Update motor speed from driven-wheel angular velocity [rad/s]
        updateStateFromDrivenWheels(obj, drivenWheelAngularVelocity)

        % Fallback update for callers that only have vehicle speed [m/s]
        updateStateFromVehicleSpeed(obj, vehicleSpeed)

        % Maximum driven-wheel angular velocity allowed by the motor [rad/s]
        maxOmega = getMaxDrivenWheelAngularVelocity(obj)

        % Clamp driven-wheel angular velocity to the powertrain speed limit
        drivenWheelAngularVelocity = limitDrivenWheelAngularVelocity(obj, drivenWheelAngularVelocity)
        
        % Maximum engine torque [Nm] at given engine speed [rpm]
        maxTorque = getMaxTorque(obj, engineSpeed)
        
        % Total gear ratio (including final drive)
        totalRatio = getTotalGearRatio(obj)
        
        % Drivetrain efficiency [0-1]
        efficiency = getDrivetrainEfficiency(obj)
    end
end
