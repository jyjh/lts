classdef (Abstract) PowertrainComponent
    % POWERTRAINCOMPONENT Abstract interface for powertrain models
    % Provides wheel torque computation and motor speed state tracking.

    properties (Abstract)
        state  % components.Powertrain.PowertrainState
    end
    
    methods (Abstract)
        % Total driven-axle wheel torque [Nm] given vehicle speed and throttle [0-1]
        wheelTorque = computeDriveTorque(obj, speed, throttle)

        % Update motor speed from driven-wheel angular velocity [rad/s]
        updateStateFromDrivenWheels(obj, drivenWheelAngularVelocity)

        % Fallback update for callers that only have vehicle speed [m/s]
        updateStateFromVehicleSpeed(obj, vehicleSpeed)

        % Driven-wheel angular velocity corresponding to the motor RPM limit [rad/s]
        maxOmega = getMaxDrivenWheelAngularVelocity(obj)
        
        % Maximum engine torque [Nm] at given engine speed [rpm]
        maxTorque = getMaxTorque(obj, engineSpeed)
        
        % Total gear ratio (including final drive)
        totalRatio = getTotalGearRatio(obj)
        
        % Drivetrain efficiency [0-1]
        efficiency = getDrivetrainEfficiency(obj)
    end
end
