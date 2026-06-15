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
        idleRPM           = 2000     % Idle speed [rpm]
        state                       % components.Powertrain.PowertrainState
    end
    
    methods
        function obj = SimplePowertrain(maxEngineTorque, totalGearRatio, wheelRadius, drivetrainEfficiency)
            % SIMPLEPOWERTRAIN Construct with fixed parameters
            %   SimplePowertrain(maxEngineTorque, totalGearRatio, wheelRadius, drivetrainEfficiency)
            if nargin >= 1 && ~isempty(maxEngineTorque)
                if isnumeric(maxEngineTorque) && isreal(maxEngineTorque) ...
                        && isscalar(maxEngineTorque) && isfinite(maxEngineTorque)
                    obj.maxEngineTorque = max(0, maxEngineTorque);
                end
            end
            if nargin >= 2 && ~isempty(totalGearRatio)
                if isnumeric(totalGearRatio) && isreal(totalGearRatio) ...
                        && isscalar(totalGearRatio) && isfinite(totalGearRatio)
                    obj.totalGearRatio = max(0, totalGearRatio);
                end
            end
            if nargin >= 3 && ~isempty(wheelRadius)
                if isnumeric(wheelRadius) && isreal(wheelRadius) ...
                        && isscalar(wheelRadius) && isfinite(wheelRadius) ...
                        && wheelRadius > 0
                    obj.wheelRadius = wheelRadius;
                end
            end
            if nargin >= 4 && ~isempty(drivetrainEfficiency)
                if isnumeric(drivetrainEfficiency) && isreal(drivetrainEfficiency) ...
                        && isscalar(drivetrainEfficiency) && isfinite(drivetrainEfficiency)
                    obj.drivetrainEfficiency = max(0, ...
                        min(1, drivetrainEfficiency));
                end
            end
            obj.state = components.Powertrain.PowertrainState();
            
            % Default torque curve (flat-ish with peak around 8000 rpm)
            obj.torqueCurveRPM = [0 2000 4000 6000 8000 10000 12000];
            obj.torqueCurveNm  = [0  35   45   52   55    50     40] * (obj.maxEngineTorque / 55);
        end
        
	        function F_drive = computeDriveForce(obj, speed, throttle)
	            % Compute drive force at wheels
	            speed = utils.nonnegativeScalarOrDefault(speed, 0);
	            throttle = utils.unitScalarOrDefault(throttle, 0);
	            totalGearRatio = utils.nonnegativeScalarOrDefault( ...
	                obj.totalGearRatio, 0);
	            wheelRadius = utils.positiveScalarOrDefault( ...
	                obj.wheelRadius, 0.2286);
	            drivetrainEfficiency = utils.unitScalarOrDefault( ...
	                obj.drivetrainEfficiency, 0.90);
	            idleRPM = utils.nonnegativeScalarOrDefault(obj.idleRPM, 0);
	            maxEngineRPM = utils.positiveScalarOrDefault( ...
	                obj.maxEngineRPM, 12000);
	            maxEngineRPM = max(maxEngineRPM, idleRPM);

	            if ~obj.state.motorSpeedInitialized
	                obj.state.updateFromVehicleSpeed( ...
	                    speed, wheelRadius, totalGearRatio);
	            end
	            engineRPM = obj.state.motorRPM;

	            % Engine braking if below idle (no drive force)
	            if engineRPM < idleRPM && throttle == 0
	                F_drive = 0;
	                obj.state.updateOutputs( ...
	                    throttle, 0, 0, F_drive, drivetrainEfficiency);
	                return;
	            end

	            % Clamp engine RPM
	            engineRPM = max(idleRPM, min(engineRPM, maxEngineRPM));

	            % Interpolate torque from curve
	            torque = interp1(obj.torqueCurveRPM, obj.torqueCurveNm, engineRPM, 'linear', 0);

	            % Drive force at wheels
	            F_drive = torque * totalGearRatio * drivetrainEfficiency ...
	                * throttle / wheelRadius;
	            wheelTorque = F_drive * wheelRadius;
	            motorTorque = torque * throttle;
	            obj.state.updateOutputs( ...
	                throttle, motorTorque, wheelTorque, F_drive, drivetrainEfficiency);
	        end

	        function updateStateFromDrivenWheels(obj, drivenWheelAngularVelocity)
	            obj.state.updateFromDrivenWheels( ...
	                drivenWheelAngularVelocity, ...
	                utils.nonnegativeScalarOrDefault(obj.totalGearRatio, 0));
	        end

	        function updateStateFromVehicleSpeed(obj, vehicleSpeed)
	            obj.state.updateFromVehicleSpeed( ...
	                vehicleSpeed, ...
	                utils.positiveScalarOrDefault(obj.wheelRadius, 0.2286), ...
	                utils.nonnegativeScalarOrDefault(obj.totalGearRatio, 0));
	        end

	        function maxOmega = getMaxDrivenWheelAngularVelocity(obj)
	            totalGearRatio = utils.nonnegativeScalarOrDefault( ...
	                obj.totalGearRatio, 0);
	            maxEngineRPM = utils.positiveScalarOrDefault( ...
	                obj.maxEngineRPM, 12000);
	            if totalGearRatio <= eps
	                maxOmega = 0;
	            else
	                maxOmega = maxEngineRPM / totalGearRatio * 2 * pi / 60;
	            end
	        end

	        function drivenWheelAngularVelocity = limitDrivenWheelAngularVelocity(obj, drivenWheelAngularVelocity)
	            maxOmega = obj.getMaxDrivenWheelAngularVelocity();
	            drivenWheelAngularVelocity = utils.nonnegativeScalarOrDefault( ...
	                drivenWheelAngularVelocity, 0);
	            drivenWheelAngularVelocity = min(drivenWheelAngularVelocity, maxOmega);
	        end

	        function torque = getMaxTorque(obj, engineSpeed)
	            % Interpolate max torque from curve
	            engineSpeed = utils.nonnegativeScalarOrDefault(engineSpeed, 0);
	            idleRPM = utils.nonnegativeScalarOrDefault(obj.idleRPM, 0);
	            maxEngineRPM = utils.positiveScalarOrDefault( ...
	                obj.maxEngineRPM, 12000);
	            if engineSpeed < idleRPM || engineSpeed > maxEngineRPM
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
	end
