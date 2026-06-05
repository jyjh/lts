classdef EMRAX228Powertrain < components.Powertrain.PowertrainComponent
    % EMRAX228POWERTRAIN EMRAX 228 electric powertrain from a MAT map
    % Uses the provided EMRAX228CC Single_4.5.mat data for motor torque and
    % tractive force with a fixed 4.5:1 final drive.
    
    properties
        matFilePath = ""
        torqueSpeedCurve = []       % Vehicle-speed breakpoints for torque curve [m/s]
        speedCurve = []             % Vehicle speed breakpoints [m/s]
        motorRPMCurve = []          % Motor speed breakpoints [rpm]
        torqueCurveNm = []          % Motor torque breakpoints [Nm]
        tractiveForceCurveN = []    % Wheel tractive-force breakpoints [N]
        state                       % components.Powertrain.PowertrainState
        totalGearRatio = 4.5        % Final drive ratio [-]
        wheelRadius = 0.228        % Effective tire radius [m]
        drivetrainEfficiency = 1.0  % Additional drivetrain efficiency [0-1]
        maxVehicleSpeed = 0         % Highest speed in the MAT tractive map [m/s]
        maxEngineTorque = 0         % Compatibility alias for existing scripts [Nm]
        maxEngineRPM = 0            % Compatibility alias for existing scripts [rpm]
        rpmFalloffStartRPM = 0      % RPM where torque falloff starts [rpm]
        rpmFalloffFactor = 1.0      % Falloff exponent: 1=linear, >1=steeper
        rpmLimitRPM = 6500          % Hard motor RPM cap [rpm]
        rpmLimitHysteresisRPM = 50  % Rev-limiter release band [rpm]
    end
    
    methods
        function obj = EMRAX228Powertrain(matFilePath, drivetrainEfficiency)
            % EMRAX228POWERTRAIN Construct from EMRAX228CC Single_4.5.mat
            %   EMRAX228Powertrain()
            %   EMRAX228Powertrain(matFilePath)
            %   EMRAX228Powertrain(matFilePath, drivetrainEfficiency)
            
            if nargin < 1 || isempty(matFilePath)
                classDir = fileparts(mfilename('fullpath'));
                matFilePath = fullfile(classDir, 'EMRAX228CC Single_4.5.mat');
            end
            if nargin >= 2
                obj.drivetrainEfficiency = max(0, min(1, drivetrainEfficiency));
            end
            obj.state = components.Powertrain.PowertrainState();
            
            data = load(matFilePath);
            obj.matFilePath = string(matFilePath);
            
            requiredFields = {'FDR', 'Speed', 'Torque', 'Tractive_force', 'Gearing_Map'};
            for i = 1:numel(requiredFields)
                if ~isfield(data, requiredFields{i})
                    error('EMRAX228Powertrain:MissingField', ...
                        'MAT file is missing required field "%s".', requiredFields{i});
                end
            end
            
            obj.totalGearRatio = data.FDR;
            
            rawSpeed = data.Speed(:);
            rawTorque = data.Torque(:);
            rawForce = data.Tractive_force(:);
            obj.validateVectorSet(rawSpeed, rawTorque, rawForce, 'raw EMRAX vectors');
            
            [obj.torqueSpeedCurve, sortIdx] = sort(rawSpeed);
            obj.torqueCurveNm = rawTorque(sortIdx);
            rawForce = rawForce(sortIdx);
            
            validRadius = obj.torqueCurveNm > 0 & rawForce > 0;
            if any(validRadius)
                obj.wheelRadius = median(obj.torqueCurveNm(validRadius) .* ...
                    obj.totalGearRatio ./ rawForce(validRadius));
            end
            
            gm = data.Gearing_Map;
            if isfield(gm, 'Speed') && isfield(gm, 'RPM') && isfield(gm, 'Traction')
                mapSpeed = gm.Speed(:);
                mapRPM = gm.RPM(:);
                mapForce = gm.Traction(:);
                obj.validateVectorSet(mapSpeed, mapRPM, mapForce, 'EMRAX gearing map');
                
                [obj.motorRPMCurve, sortIdx] = sort(mapRPM);
                obj.speedCurve = mapSpeed(sortIdx);
                obj.tractiveForceCurveN = mapForce(sortIdx);
            else
                obj.speedCurve = obj.torqueSpeedCurve;
                obj.motorRPMCurve = obj.vehicleSpeedToMotorRPM(obj.speedCurve);
                obj.tractiveForceCurveN = rawForce;
            end
            
            obj.maxEngineTorque = max(obj.torqueCurveNm);
            obj.rpmFalloffStartRPM = max(obj.motorRPMCurve);
            obj.maxEngineRPM = obj.rpmLimitRPM;
            obj.maxVehicleSpeed = max(obj.speedCurve);
        end
        
        function F_drive = computeDriveForce(obj, speed, throttle)
            % Compute wheel tractive force from current motor RPM.
            throttle = max(0, min(1, throttle));
            
            if throttle == 0
                F_drive = 0;
                obj.state.updateOutputs(throttle, 0, 0, F_drive, obj.drivetrainEfficiency);
                return;
            end
            
            if ~obj.state.motorSpeedInitialized && nargin >= 2
                obj.state.updateFromVehicleSpeed( ...
                    speed, obj.wheelRadius, obj.totalGearRatio);
            end
            
            motorRPM = obj.state.motorRPM;
            rpmLimitActive = obj.isRPMLimitActive(motorRPM);
            if rpmLimitActive
                F_drive = 0;
                obj.state.updateOutputs(throttle, 0, 0, F_drive, ...
                    obj.drivetrainEfficiency, true);
                return;
            end
            
            fullThrottleForce = obj.lookupTractiveForceByRPM(motorRPM);
            
            F_drive = fullThrottleForce * throttle * obj.drivetrainEfficiency;
            wheelTorque = F_drive * obj.wheelRadius;
            if obj.totalGearRatio > 0 && obj.drivetrainEfficiency > 0
                motorTorque = wheelTorque / ...
                    (obj.totalGearRatio * obj.drivetrainEfficiency);
            else
                motorTorque = 0;
            end
            obj.state.updateOutputs( ...
                throttle, motorTorque, wheelTorque, F_drive, obj.drivetrainEfficiency, false);
        end
        
        function updateStateFromDrivenWheels(obj, drivenWheelAngularVelocity)
            % Update motor RPM from driven-wheel angular velocity [rad/s].
            obj.state.updateFromDrivenWheels( ...
                drivenWheelAngularVelocity, obj.totalGearRatio);
        end
        
        function updateStateFromVehicleSpeed(obj, vehicleSpeed)
            % Fallback update for callers without wheel rotational state.
            obj.state.updateFromVehicleSpeed( ...
                vehicleSpeed, obj.wheelRadius, obj.totalGearRatio);
        end
        
        function maxOmega = getMaxDrivenWheelAngularVelocity(obj)
            % Maximum driven-wheel angular velocity from the motor RPM cap.
            maxOmega = obj.rpmLimitRPM / obj.totalGearRatio * 2 * pi / 60;
        end
        
        function drivenWheelAngularVelocity = limitDrivenWheelAngularVelocity(obj, drivenWheelAngularVelocity)
            % Clamp driven-wheel speed so the direct-coupled motor stays capped.
            maxOmega = obj.getMaxDrivenWheelAngularVelocity();
            drivenWheelAngularVelocity = min(max(0, drivenWheelAngularVelocity), maxOmega);
        end
        
        function fullThrottleForce = lookupTractiveForceByRPM(obj, motorRPM)
            % Interpolate full-throttle tractive force by motor speed [rpm].
            motorRPM = max(0, motorRPM);
            
            if motorRPM >= obj.rpmLimitRPM
                fullThrottleForce = 0;
            elseif motorRPM <= obj.motorRPMCurve(1)
                fullThrottleForce = obj.tractiveForceCurveN(1);
            elseif motorRPM <= obj.rpmFalloffStartRPM
                fullThrottleForce = obj.lookupMappedTractiveForce(motorRPM);
            else
                fullThrottleForce = obj.lookupMappedTractiveForce( ...
                    obj.rpmFalloffStartRPM) * obj.computeRPMFalloffMultiplier(motorRPM);
            end
        end
        
        function torque = getMaxTorque(obj, engineSpeed)
            % Interpolate max EMRAX motor torque at motor speed [rpm].
            engineSpeed = max(0, engineSpeed);

            if engineSpeed >= obj.rpmLimitRPM
                torque = 0;
            elseif engineSpeed <= obj.rpmFalloffStartRPM
                torque = obj.lookupMappedMotorTorque(engineSpeed);
            else
                torque = obj.lookupMappedMotorTorque(obj.rpmFalloffStartRPM) * ...
                    obj.computeRPMFalloffMultiplier(engineSpeed);
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
        function rpm = vehicleSpeedToMotorRPM(obj, speed)
            rpm = speed ./ (2 * pi * obj.wheelRadius) * 60 * obj.totalGearRatio;
        end
        
        function force = lookupMappedTractiveForce(obj, motorRPM)
            motorRPM = max(obj.motorRPMCurve(1), ...
                min(obj.motorRPMCurve(end), motorRPM));
            force = interp1(obj.motorRPMCurve, obj.tractiveForceCurveN, ...
                motorRPM, 'linear');
        end
        
        function torque = lookupMappedMotorTorque(obj, motorRPM)
            rawRPM = obj.vehicleSpeedToMotorRPM(obj.torqueSpeedCurve);
            motorRPM = max(rawRPM(1), min(rawRPM(end), motorRPM));
            torque = interp1(rawRPM, obj.torqueCurveNm, motorRPM, 'linear');
        end
        
        function multiplier = computeRPMFalloffMultiplier(obj, motorRPM)
            if motorRPM <= obj.rpmFalloffStartRPM
                multiplier = 1;
                return;
            end
            if motorRPM >= obj.rpmLimitRPM
                multiplier = 0;
                return;
            end
            
            falloffRange = max(obj.rpmLimitRPM - obj.rpmFalloffStartRPM, eps);
            remainingFraction = (obj.rpmLimitRPM - motorRPM) / falloffRange;
            multiplier = remainingFraction ^ max(obj.rpmFalloffFactor, eps);
            multiplier = max(0, min(1, multiplier));
        end
        
        function active = isRPMLimitActive(obj, motorRPM)
            if obj.state.rpmLimitActive
                active = motorRPM >= obj.rpmLimitRPM - obj.rpmLimitHysteresisRPM;
            else
                active = motorRPM >= obj.rpmLimitRPM;
            end
        end
        
    end
    
    methods (Static, Access = private)
        function validateVectorSet(a, b, c, label)
            if isempty(a) || isempty(b) || isempty(c) || ...
                    numel(a) ~= numel(b) || numel(a) ~= numel(c)
                error('EMRAX228Powertrain:InvalidMap', ...
                    'Invalid %s: vectors must be non-empty and equal length.', label);
            end
        end
    end
end
