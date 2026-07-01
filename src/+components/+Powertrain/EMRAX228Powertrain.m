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
        totalGearRatio = 3        % Final drive ratio [-]
        wheelRadius = 0.228        % Effective tire radius [m]
        drivetrainEfficiency = 0.92  % Additional drivetrain efficiency [0-1]
        motorRotorInertia = 0.07    % Motor rotor inertia [kg*m^2], reflected as I*ratio^2 to wheels
        % --- Coastdown / regen (off-throttle motoring) ---
        % Both default OFF to preserve baseline behavior. When regenEnabled is
        % true the motor also lifts the omega>=0 / speed>=0 clamps so it can
        % track reverse rotation and apply a coastdown drag torque.
        regenEnabled = false            % Apply regenerative braking at off-throttle
        motoringDragTorque = 0          % Motor coastdown drag torque [Nm] (motor-side), 0 = off
        regenTorqueLimitNm = 30         % Max regen torque (motor-side) at off-throttle [Nm]
        regenEnabledSpeedFloor = 1.0    % Vehicle speed below which regen tapers to 0 [m/s]
        maxVehicleSpeed = 0         % Highest speed in the MAT tractive map [m/s]
        maxEngineTorque = 0         % Compatibility alias for existing scripts [Nm]
        maxEngineRPM = 0            % Compatibility alias for existing scripts [rpm]
        baseSpeedRPM = 5000         % Motor physical base speed (field-weakening onset) [rpm]
        rpmFalloffStartRPM = 0      % RPM above which constant-power rolloff is applied [rpm]
        rpmFalloffFactor = 1.0      % Deprecated: ignored. Kept for config back-compat.
        rpmLimitRPM = 6500          % Hard motor RPM cap [rpm]
        rpmLimitHysteresisRPM = 50  % Rev-limiter release band [rpm]
    end

    properties (Dependent)
        % True when the powertrain may produce reverse rotation / coastdown
        % drag, so callers (tire/differential) can lift omega>=0 clamps.
        reverseCapable
    end

    methods
        function obj = EMRAX228Powertrain(matFilePath, drivetrainEfficiency, motorRotorInertia)
            % EMRAX228POWERTRAIN Construct from EMRAX228CC Single_4.5.mat
            %   EMRAX228Powertrain()
            %   EMRAX228Powertrain(matFilePath)
            %   EMRAX228Powertrain(matFilePath, drivetrainEfficiency)
            %   EMRAX228Powertrain(matFilePath, drivetrainEfficiency, motorRotorInertia)

            if nargin < 1 || isempty(matFilePath)
                classDir = fileparts(mfilename('fullpath'));
                matFilePath = fullfile(classDir, 'EMRAX228CC Single_4.5.mat');
            end
            if nargin >= 2
                obj.drivetrainEfficiency = max(0, min(1, drivetrainEfficiency));
            end
            if nargin >= 3 && ~isempty(motorRotorInertia)
                obj.motorRotorInertia = max(0, motorRotorInertia);
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
            % The constant-power rolloff is anchored at the top of the measured
            % map. The EMRAX .mat already encodes partial field-weakening
            % through its full RPM range, so we trust the measured force up to
            % the table's end and apply T proportional to 1/rpm only for the
            % extrapolation beyond it (up to rpmLimitRPM). baseSpeedRPM remains
            % the documented physical base speed for reference/telemetry.
            obj.rpmFalloffStartRPM = max(obj.motorRPMCurve);
            obj.maxEngineRPM = obj.rpmLimitRPM;
            obj.maxVehicleSpeed = max(obj.speedCurve);
        end
        
        function wheelTorque = computeDriveTorque(obj, speed, throttle)
            % Compute total driven-axle wheel torque from current motor RPM.
            throttle = max(0, min(1, throttle));
            
            if throttle == 0
                wheelTorque = 0;
                obj.state.updateOutputs(throttle, 0, 0, 0, obj.drivetrainEfficiency);
                return;
            end
            
            if ~obj.state.motorSpeedInitialized && nargin >= 2
                obj.state.updateFromVehicleSpeed( ...
                    speed, obj.wheelRadius, obj.totalGearRatio);
            end
            
            motorRPM = obj.state.motorRPM;
            rpmLimitActive = obj.isRPMLimitActive(motorRPM);
            if rpmLimitActive
                wheelTorque = 0;
                obj.state.updateOutputs(throttle, 0, 0, 0, ...
                    obj.drivetrainEfficiency, true);
                return;
            end
            
            fullThrottleForce = obj.lookupTractiveForceByRPM(motorRPM);
            
            equivalentDriveForce = fullThrottleForce * throttle * obj.drivetrainEfficiency;
            wheelTorque = equivalentDriveForce * obj.wheelRadius;
            if obj.totalGearRatio > 0 && obj.drivetrainEfficiency > 0
                motorTorque = wheelTorque / ...
                    (obj.totalGearRatio * obj.drivetrainEfficiency);
            else
                motorTorque = 0;
            end
            obj.state.updateOutputs( ...
                throttle, motorTorque, wheelTorque, equivalentDriveForce, ...
                obj.drivetrainEfficiency, false);
        end

        function F_drive = computeDriveForce(obj, speed, throttle)
            % Compatibility helper: requested wheel torque as equivalent force.
            wheelTorque = obj.computeDriveTorque(speed, throttle);
            F_drive = wheelTorque / max(obj.wheelRadius, eps);
        end
        
        function updateStateFromDrivenWheels(obj, drivenWheelAngularVelocity)
            % Update motor RPM from driven-wheel angular velocity [rad/s].
            obj.state.allowReverseRotation = obj.reverseCapable;
            obj.state.updateFromDrivenWheels( ...
                drivenWheelAngularVelocity, obj.totalGearRatio);
        end

        function updateStateFromVehicleSpeed(obj, vehicleSpeed)
            % Fallback update for callers without wheel rotational state.
            obj.state.allowReverseRotation = obj.reverseCapable;
            obj.state.updateFromVehicleSpeed( ...
                vehicleSpeed, obj.wheelRadius, obj.totalGearRatio);
        end
        
        function maxOmega = getMaxDrivenWheelAngularVelocity(obj)
            % Driven-wheel angular velocity corresponding to the motor RPM cap.
            maxOmega = obj.rpmLimitRPM / obj.totalGearRatio * 2 * pi / 60;
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
            % Derived from the same wheel-force path as computeDriveTorque so
            % telemetry (GraphPlotter) and the sim agree on a single source of
            % truth: T_motor = F_wheel * R / (ratio * efficiency).
            engineSpeed = max(0, engineSpeed);
            fullThrottleForce = obj.lookupTractiveForceByRPM(engineSpeed);
            if obj.totalGearRatio > 0 && obj.drivetrainEfficiency > 0
                torque = fullThrottleForce * obj.wheelRadius / ...
                    (obj.totalGearRatio * obj.drivetrainEfficiency);
            else
                torque = 0;
            end
        end
        
        function ratio = getTotalGearRatio(obj)
            ratio = obj.totalGearRatio;
        end

        function I = getReflectedRotorInertia(obj)
            % GETREFLECTEDROTORINERTIA Motor rotor inertia reflected to the
            %   driven wheels [kg*m^2]. A gear ratio couples the rotor to the
            %   wheels so the effective rotational inertia seen at each wheel
            %   is I_motor * ratio^2 (per half-shaft). This is added to the
            %   bare wheel+tire inertia on the driven axle only.
            I = obj.motorRotorInertia * obj.totalGearRatio^2;
        end

        function tf = get.reverseCapable(obj)
            % True when the powertrain may rotate backward / apply coastdown
            % drag. Callers use this to gate omega>=0 clamps.
            tf = obj.regenEnabled || obj.motoringDragTorque > 0;
        end

        function T = computeCoastdownTorque(obj, vehicleSpeed, throttle)
            % COMPUTECOASTDOWNTORQUE Off-throttle motoring/regen torque
            %   reflected to the driven axle [Nm] (signed, opposing rotation).
            %   Returns 0 when both regen and motoring drag are disabled, so
            %   baseline (flags off) is unaffected. Negative = braking.
            %
            %   - Motoring drag: always opposes motor spin (motor-side torque
            %     reflected through the ratio), applied whenever > 0.
            %   - Regen: at off-throttle (throttle == 0) and forward speed, a
            %     braking torque up to regenTorqueLimitNm, tapered to 0 below
            %     regenEnabledSpeedFloor and never strong enough to reverse the
            %     wheels.
            T = 0;
            motorSign = sign(obj.state.motorAngularVelocity);
            if motorSign == 0 && vehicleSpeed > 0
                motorSign = 1;  % forward coastdown from rest-forward
            end
            if obj.motoringDragTorque > 0
                T = T - motorSign * obj.motoringDragTorque * obj.totalGearRatio;
            end
            if obj.regenEnabled && throttle == 0 && vehicleSpeed > 0
                % Taper regen to zero near rest so it cannot reverse the car.
                taper = min(1, vehicleSpeed / max(obj.regenEnabledSpeedFloor, eps));
                T_regen = obj.regenTorqueLimitNm * obj.totalGearRatio * ...
                    obj.drivetrainEfficiency * taper;
                T = T - motorSign * T_regen;
            end
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

        function multiplier = computeRPMFalloffMultiplier(obj, motorRPM)
            % Constant-power field-weakening rolloff, anchored at the top of
            % the measured map (rpmFalloffStartRPM = max(motorRPMCurve)). The
            % EMRAX .mat already encodes the constant-torque/early field-
            % weakening region through its full RPM range, so we trust the
            % measured force up to the table end and apply T proportional to
            % 1/rpm only for the extrapolation beyond it. The previous linear
            % falloff drove torque to 0 at the rev limit (e.g. 1194 N at
            % 6000 rpm vs the correct ~2240 N) — a large over-declaration.
            if motorRPM <= obj.rpmFalloffStartRPM
                multiplier = 1;
                return;
            end
            if motorRPM >= obj.rpmLimitRPM
                multiplier = 0;
                return;
            end
            % Constant power: T(rpm) = T_anchor * rpmFalloffStartRPM / rpm.
            multiplier = obj.rpmFalloffStartRPM / motorRPM;
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
