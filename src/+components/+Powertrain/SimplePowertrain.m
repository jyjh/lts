classdef SimplePowertrain < components.Powertrain.PowertrainComponent
    % SIMPLEPOWERTRAIN Single-gear powertrain with basic torque curve
    % Uses a lookup table for engine torque and fixed gear ratio
    
    properties
        maxEngineTorque   = 55       % Peak engine torque [Nm] (typical FSAE single cylinder)
        torqueCurveRPM    = []       % RPM breakpoints for torque curve
        torqueCurveNm     = []       % Torque values [Nm] at breakpoints
        totalGearRatio    = 12.0     % Overall gear ratio (gear * final drive)
        wheelRadius       = 0.2286   % Tire rolling radius [m] (13" wheel with tire)
        drivetrainEfficiency = 0.90  % Drivetrain efficiency [0-1]
        maxEngineRPM      = 12000    % Redline [rpm]
        rpmLimitHysteresisRPM = 50   % Rev-limiter release band [rpm]
        idleRPM           = 2000     % Idle speed [rpm]
        state                       % components.Powertrain.PowertrainState
    end
    
    methods
        function obj = SimplePowertrain(maxEngineTorque, totalGearRatio, wheelRadius, drivetrainEfficiency)
            % SIMPLEPOWERTRAIN Construct with fixed parameters
            %   SimplePowertrain(maxEngineTorque, totalGearRatio, wheelRadius, drivetrainEfficiency)
            obj.maxEngineTorque = maxEngineTorque;
            obj.totalGearRatio = totalGearRatio;
            obj.wheelRadius = wheelRadius;
            obj.drivetrainEfficiency = drivetrainEfficiency;
            obj.state = components.Powertrain.PowertrainState();
            
            % Default torque curve (flat-ish with peak around 8000 rpm)
            obj.torqueCurveRPM = [0 2000 4000 6000 8000 10000 12000];
            obj.torqueCurveNm  = [0  35   45   52   55    50     40] * (obj.maxEngineTorque / 55);
        end
        
        function wheelTorque = computeDriveTorque(obj, speed, throttle)
            % Compute total driven-axle wheel torque.
            throttle = max(0, min(1, throttle));  % Clamp [0,1]
            
            if ~obj.state.motorSpeedInitialized
                obj.state.updateFromVehicleSpeed( ...
                    speed, obj.wheelRadius, obj.totalGearRatio);
            end
            engineRPM = obj.state.motorRPM;
            
            if throttle == 0
                wheelTorque = 0;
                obj.state.updateOutputs(throttle, 0, 0, 0, obj.drivetrainEfficiency);
                return;
            end
            
            if obj.isRPMLimitActive(engineRPM)
                wheelTorque = 0;
                obj.state.updateOutputs(throttle, 0, 0, 0, ...
                    obj.drivetrainEfficiency, true);
                return;
            end
            
            % Interpolate torque from curve
            lookupRPM = max(obj.idleRPM, engineRPM);
            lookupRPM = min(lookupRPM, obj.torqueCurveRPM(end));
            torque = interp1(obj.torqueCurveRPM, obj.torqueCurveNm, lookupRPM, 'linear', 0);
            
            % Total torque delivered to the driven axle.
            wheelTorque = torque * obj.totalGearRatio * obj.drivetrainEfficiency * throttle;
            equivalentDriveForce = wheelTorque / max(obj.wheelRadius, eps);
            motorTorque = torque * throttle;
            obj.state.updateOutputs( ...
                throttle, motorTorque, wheelTorque, equivalentDriveForce, ...
                obj.drivetrainEfficiency);
        end

        function F_drive = computeDriveForce(obj, speed, throttle)
            % Compatibility helper: requested wheel torque as equivalent force.
            wheelTorque = obj.computeDriveTorque(speed, throttle);
            F_drive = wheelTorque / max(obj.wheelRadius, eps);
        end
        
        function updateStateFromDrivenWheels(obj, drivenWheelAngularVelocity)
            obj.state.updateFromDrivenWheels( ...
                drivenWheelAngularVelocity, obj.totalGearRatio);
        end
        
        function updateStateFromVehicleSpeed(obj, vehicleSpeed)
            obj.state.updateFromVehicleSpeed( ...
                vehicleSpeed, obj.wheelRadius, obj.totalGearRatio);
        end
        
        function maxOmega = getMaxDrivenWheelAngularVelocity(obj)
            maxOmega = obj.maxEngineRPM / obj.totalGearRatio * 2 * pi / 60;
        end
        
        function torque = getMaxTorque(obj, engineSpeed)
            % Interpolate max torque from curve
            if engineSpeed < obj.idleRPM || engineSpeed > obj.maxEngineRPM
                torque = 0;
            else
                torque = interp1(obj.torqueCurveRPM, obj.torqueCurveNm, engineSpeed, 'linear', 0);
            end
        end
        
        function ratio = getTotalGearRatio(obj)
            ratio = obj.totalGearRatio;
        end
        
        function eff = getDrivetrainEfficiency(obj)
            eff = obj.drivetrainEfficiency;
        end
    end

    methods (Access = private)
        function active = isRPMLimitActive(obj, engineRPM)
            if obj.state.rpmLimitActive
                active = engineRPM >= obj.maxEngineRPM - obj.rpmLimitHysteresisRPM;
            else
                active = engineRPM >= obj.maxEngineRPM;
            end
        end
    end
end
