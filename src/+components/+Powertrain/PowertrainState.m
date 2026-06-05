classdef PowertrainState < handle
    % POWERTRAINSTATE Mutable powertrain transient state
    % Tracks motor speed and commanded output for the current simulation step.
    % Uses handle inheritance so powertrain components can mutate state in-place
    % across timesteps, mirroring TireState and SuspensionState.
    
    properties
        % --- Rotational state ---
        
        % Average driven-wheel angular velocity [rad/s]
        drivenWheelAngularVelocity = 0
        
        % Average driven-wheel speed [rpm]
        drivenWheelRPM = 0
        
        % Motor angular velocity [rad/s]
        motorAngularVelocity = 0
        
        % Motor speed [rpm]
        motorRPM = 0
        
        % True after motor speed has been updated from wheels or fallback speed
        motorSpeedInitialized = false
        
        % True when the powertrain is cutting positive torque at the RPM cap
        rpmLimitActive = false
        
        % --- Command/output state ---
        
        % Throttle position [0-1]
        throttle = 0
        
        % Motor torque command/output [Nm]
        motorTorque = 0
        
        % Wheel torque after gear ratio and efficiency [Nm]
        wheelTorque = 0
        
        % Longitudinal drive force at the contact patches [N]
        driveForce = 0
        
        % Gear/final drive ratio used for this state update [-]
        gearRatio = 0
        
        % Drivetrain efficiency used for this state update [0-1]
        drivetrainEfficiency = 1
    end
    
    methods
        function obj = PowertrainState()
            % POWERTRAINSTATE Construct with zero initial conditions
            obj.reset();
        end
        
        function updateFromDrivenWheels(obj, drivenWheelAngularVelocity, gearRatio)
            % UPDATEFROMDRIVENWHEELS Update motor speed from driven wheels.
            %   drivenWheelAngularVelocity may be a scalar or vector [rad/s].
            drivenWheelAngularVelocity = drivenWheelAngularVelocity(:);
            drivenWheelAngularVelocity = drivenWheelAngularVelocity( ...
                isfinite(drivenWheelAngularVelocity));
            
            if isempty(drivenWheelAngularVelocity)
                avgWheelOmega = 0;
            else
                avgWheelOmega = mean(max(0, drivenWheelAngularVelocity));
            end
            
            obj.drivenWheelAngularVelocity = avgWheelOmega;
            obj.drivenWheelRPM = avgWheelOmega * 60 / (2 * pi);
            obj.gearRatio = gearRatio;
            obj.motorAngularVelocity = avgWheelOmega * gearRatio;
            obj.motorRPM = obj.motorAngularVelocity * 60 / (2 * pi);
            obj.motorSpeedInitialized = true;
        end
        
        function updateFromVehicleSpeed(obj, vehicleSpeed, wheelRadius, gearRatio)
            % UPDATEFROMVEHICLESPEED Fallback for standalone/non-wheel tests.
            vehicleSpeed = max(0, vehicleSpeed);
            wheelRadius = max(wheelRadius, eps);
            wheelOmega = vehicleSpeed / wheelRadius;
            obj.updateFromDrivenWheels(wheelOmega, gearRatio);
        end
        
        function updateOutputs(obj, throttle, motorTorque, wheelTorque, driveForce, drivetrainEfficiency, rpmLimitActive)
            % UPDATEOUTPUTS Store the current powertrain command/output.
            if nargin < 7
                rpmLimitActive = false;
            end
            obj.throttle = max(0, min(1, throttle));
            obj.motorTorque = motorTorque;
            obj.wheelTorque = wheelTorque;
            obj.driveForce = driveForce;
            obj.drivetrainEfficiency = drivetrainEfficiency;
            obj.rpmLimitActive = rpmLimitActive;
        end
        
        function reset(obj)
            % RESET Reset all dynamic state to zero
            obj.drivenWheelAngularVelocity = 0;
            obj.drivenWheelRPM = 0;
            obj.motorAngularVelocity = 0;
            obj.motorRPM = 0;
            obj.motorSpeedInitialized = false;
            obj.rpmLimitActive = false;
            obj.throttle = 0;
            obj.motorTorque = 0;
            obj.wheelTorque = 0;
            obj.driveForce = 0;
            obj.gearRatio = 0;
            obj.drivetrainEfficiency = 1;
        end
    end
end
