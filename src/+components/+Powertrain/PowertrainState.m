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

        % Driven-wheel angular velocity used for timestep power [rad/s]
        powerDrivenWheelAngularVelocity = 0

        % Motor angular velocity used for timestep power [rad/s]
        powerMotorAngularVelocity = 0

        % Motor speed used for timestep power [rpm]
        powerMotorRPM = 0
        
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

        % Mechanical motor power before drivetrain losses [W]
        motorPower = 0

        % Mechanical power delivered to the driven axle [W]
        wheelPower = 0

        % Positive drivetrain loss power [W]
        drivetrainLossPower = 0
        
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

        function set.drivenWheelAngularVelocity(obj, value)
            obj.drivenWheelAngularVelocity = utils.nonnegativeScalarOrDefault(value, 0);
        end

        function set.drivenWheelRPM(obj, value)
            obj.drivenWheelRPM = utils.nonnegativeScalarOrDefault(value, 0);
        end

        function set.motorAngularVelocity(obj, value)
            obj.motorAngularVelocity = utils.nonnegativeScalarOrDefault(value, 0);
        end

        function set.motorRPM(obj, value)
            obj.motorRPM = utils.nonnegativeScalarOrDefault(value, 0);
        end

        function set.powerDrivenWheelAngularVelocity(obj, value)
            obj.powerDrivenWheelAngularVelocity = utils.nonnegativeScalarOrDefault(value, 0);
        end

        function set.powerMotorAngularVelocity(obj, value)
            obj.powerMotorAngularVelocity = utils.nonnegativeScalarOrDefault(value, 0);
        end

        function set.powerMotorRPM(obj, value)
            obj.powerMotorRPM = utils.nonnegativeScalarOrDefault(value, 0);
        end

        function set.motorSpeedInitialized(obj, value)
            obj.motorSpeedInitialized = utils.logicalScalarOrDefault(value, false);
        end

        function set.rpmLimitActive(obj, value)
            obj.rpmLimitActive = utils.logicalScalarOrDefault(value, false);
        end

        function set.throttle(obj, value)
            obj.throttle = utils.unitScalarOrDefault(value, 0);
        end

        function set.motorTorque(obj, value)
            obj.motorTorque = utils.scalarOrDefault(value, 0);
        end

        function set.wheelTorque(obj, value)
            obj.wheelTorque = utils.scalarOrDefault(value, 0);
        end

        function set.driveForce(obj, value)
            obj.driveForce = utils.scalarOrDefault(value, 0);
        end

        function set.motorPower(obj, value)
            obj.motorPower = utils.scalarOrDefault(value, 0);
        end

        function set.wheelPower(obj, value)
            obj.wheelPower = utils.scalarOrDefault(value, 0);
        end

        function set.drivetrainLossPower(obj, value)
            obj.drivetrainLossPower = utils.nonnegativeScalarOrDefault(value, 0);
        end

        function set.gearRatio(obj, value)
            obj.gearRatio = utils.nonnegativeScalarOrDefault(value, 0);
        end

        function set.drivetrainEfficiency(obj, value)
            obj.drivetrainEfficiency = utils.unitScalarOrDefault(value, 1);
        end
        
        function updateFromDrivenWheels(obj, drivenWheelAngularVelocity, gearRatio)
            % UPDATEFROMDRIVENWHEELS Update motor speed from driven wheels.
            %   drivenWheelAngularVelocity may be a scalar or vector [rad/s].
            avgWheelOmega = obj.meanNonnegativeFinite( ...
                drivenWheelAngularVelocity, 0);
            if nargin < 3 || isempty(gearRatio)
                gearRatio = obj.gearRatio;
            end
            gearRatio = utils.nonnegativeScalarOrDefault(gearRatio, 0);
            
            obj.drivenWheelAngularVelocity = avgWheelOmega;
            obj.drivenWheelRPM = avgWheelOmega * 60 / (2 * pi);
            obj.gearRatio = gearRatio;
            obj.motorAngularVelocity = avgWheelOmega * gearRatio;
            obj.motorRPM = obj.motorAngularVelocity * 60 / (2 * pi);
            obj.motorSpeedInitialized = true;
            obj.updatePowerTelemetry();
        end
        
        function updateFromVehicleSpeed(obj, vehicleSpeed, wheelRadius, gearRatio)
            % UPDATEFROMVEHICLESPEED Fallback for standalone/non-wheel tests.
            vehicleSpeed = utils.nonnegativeScalarOrDefault(vehicleSpeed, 0);
            wheelRadius = utils.positiveScalarOrDefault(wheelRadius, 1);
            if nargin < 4 || isempty(gearRatio)
                gearRatio = obj.gearRatio;
            end
            gearRatio = utils.nonnegativeScalarOrDefault(gearRatio, 0);
            wheelOmega = vehicleSpeed / wheelRadius;
            obj.updateFromDrivenWheels(wheelOmega, gearRatio);
        end
        
        function updateOutputs(obj, throttle, motorTorque, wheelTorque, driveForce, drivetrainEfficiency, rpmLimitActive)
            % UPDATEOUTPUTS Store the current powertrain command/output.
            if nargin < 7
                rpmLimitActive = false;
            end
            obj.throttle = throttle;
            obj.motorTorque = motorTorque;
            obj.wheelTorque = wheelTorque;
            obj.driveForce = driveForce;
            obj.drivetrainEfficiency = drivetrainEfficiency;
            obj.rpmLimitActive = rpmLimitActive;
            obj.updatePowerTelemetry();
        end

        function updatePowerTelemetry(obj, powerDrivenWheelAngularVelocity)
            % UPDATEPOWERTELEMETRY Keep torque, speed, and power consistent.
            %
            % drivenWheelAngularVelocity is the post-update rotational state.
            % Power over a finite timestep should use the mean driven-wheel
            % speed while torque was applied. When no explicit mean is supplied,
            % fall back to the current state for instantaneous estimates.
            if nargin < 2 || isempty(powerDrivenWheelAngularVelocity)
                powerOmega = obj.drivenWheelAngularVelocity;
            else
                powerOmega = obj.meanNonnegativeFinite( ...
                    powerDrivenWheelAngularVelocity, 0);
            end

            obj.powerDrivenWheelAngularVelocity = powerOmega;
            obj.powerMotorAngularVelocity = powerOmega * obj.gearRatio;
            obj.powerMotorRPM = obj.powerMotorAngularVelocity * 60 / (2 * pi);

            % wheelTorque is the total driven-axle torque. Multiplying by mean
            % driven-wheel speed gives left + right wheel power for an
            % equal-torque differential:
            %   P_axle = (T/2)*omega_L + (T/2)*omega_R = T*mean(omega)
            obj.motorPower = obj.motorTorque * obj.powerMotorAngularVelocity;
            obj.wheelPower = obj.wheelTorque * obj.powerDrivenWheelAngularVelocity;
            obj.drivetrainLossPower = max(0, obj.motorPower - obj.wheelPower);
        end
        
        function reset(obj)
            % RESET Reset all dynamic state to zero
            obj.drivenWheelAngularVelocity = 0;
            obj.drivenWheelRPM = 0;
            obj.motorAngularVelocity = 0;
            obj.motorRPM = 0;
            obj.powerDrivenWheelAngularVelocity = 0;
            obj.powerMotorAngularVelocity = 0;
            obj.powerMotorRPM = 0;
            obj.motorSpeedInitialized = false;
            obj.rpmLimitActive = false;
            obj.throttle = 0;
            obj.motorTorque = 0;
            obj.wheelTorque = 0;
            obj.driveForce = 0;
            obj.motorPower = 0;
            obj.wheelPower = 0;
            obj.drivetrainLossPower = 0;
            obj.gearRatio = 0;
            obj.drivetrainEfficiency = 1;
        end
    end

    methods (Static, Access = private)





        function value = meanNonnegativeFinite(candidate, defaultValue)
            value = defaultValue;
            if ~isnumeric(candidate) || ~isreal(candidate)
                return;
            end
            candidate = candidate(:);
            candidate = candidate(isfinite(candidate));
            if ~isempty(candidate)
                value = mean(max(0, candidate));
            end
        end
    end
end
