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

        % Requested motor torque before any simulator-side replay limiting [Nm]
        requestedMotorTorque = 0

        % Signed motor torque bound implied by measured DC pack power [Nm]
        motorTorquePowerLimitNm = NaN

        % True when measured pack power reduced the requested motor torque
        motorTorquePowerLimitActive = false

        % Measured tractive pack voltage/current/power used for direct replay
        packVoltageV = NaN
        packCurrentA = NaN
        packPowerW = NaN
        
        % Wheel torque after gear ratio and efficiency [Nm]
        wheelTorque = 0
        
        % Equivalent requested drive force for telemetry/backward compatibility [N]
        driveForce = 0
        
        % Gear/final drive ratio used for this state update [-]
        gearRatio = 0
        
        % Drivetrain efficiency used for this state update [0-1]
        drivetrainEfficiency = 1

        % Allow negative motor/wheel angular velocity (reverse rotation). When
        % false (default) omega is clamped >= 0 for stable forward-only sim.
        % The powertrain sets this true when regen/coastdown is enabled.
        allowReverseRotation = false
    end
    
    methods
        function obj = PowertrainState()
            % POWERTRAINSTATE Construct with zero initial conditions
            obj.reset();
        end
        
        function updateFromDrivenWheels(obj, drivenWheelAngularVelocity, gearRatio)
            % UPDATEFROMDRIVENWHEELS Update motor speed from driven wheels.
            %   drivenWheelAngularVelocity may be a scalar or vector [rad/s].
            if isscalar(drivenWheelAngularVelocity)
                avgWheelOmega = drivenWheelAngularVelocity;
                if ~isfinite(avgWheelOmega)
                    avgWheelOmega = 0;
                elseif ~obj.allowReverseRotation
                    avgWheelOmega = max(0, avgWheelOmega);
                end
            elseif obj.allowReverseRotation
                drivenWheelAngularVelocity = drivenWheelAngularVelocity(:);
                drivenWheelAngularVelocity = drivenWheelAngularVelocity( ...
                    isfinite(drivenWheelAngularVelocity));
                if isempty(drivenWheelAngularVelocity)
                    avgWheelOmega = 0;
                else
                    avgWheelOmega = mean(drivenWheelAngularVelocity);
                end
            else
                drivenWheelAngularVelocity = drivenWheelAngularVelocity(:);
                drivenWheelAngularVelocity = drivenWheelAngularVelocity( ...
                    isfinite(drivenWheelAngularVelocity));
                if isempty(drivenWheelAngularVelocity)
                    avgWheelOmega = 0;
                else
                    avgWheelOmega = mean(max(0, drivenWheelAngularVelocity));
                end
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
            if ~obj.allowReverseRotation
                vehicleSpeed = max(0, vehicleSpeed);
            end
            wheelRadius = max(wheelRadius, eps);
            wheelOmega = vehicleSpeed / wheelRadius;
            obj.updateFromDrivenWheels(wheelOmega, gearRatio);
        end
        
        function updateOutputs(obj, throttle, motorTorque, wheelTorque, driveForce, drivetrainEfficiency, rpmLimitActive)
            % UPDATEOUTPUTS Store the current powertrain command/output.
            if nargin < 7
                rpmLimitActive = false;
            end
            obj.throttle = lts.util.saturate(throttle);
            obj.motorTorque = motorTorque;
            obj.requestedMotorTorque = motorTorque;
            obj.motorTorquePowerLimitNm = NaN;
            obj.motorTorquePowerLimitActive = false;
            obj.packVoltageV = NaN;
            obj.packCurrentA = NaN;
            obj.packPowerW = NaN;
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
            obj.requestedMotorTorque = 0;
            obj.motorTorquePowerLimitNm = NaN;
            obj.motorTorquePowerLimitActive = false;
            obj.packVoltageV = NaN;
            obj.packCurrentA = NaN;
            obj.packPowerW = NaN;
            obj.wheelTorque = 0;
            obj.driveForce = 0;
            obj.gearRatio = 0;
            obj.drivetrainEfficiency = 1;
        end
    end
end
