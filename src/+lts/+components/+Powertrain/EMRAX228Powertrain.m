classdef EMRAX228Powertrain < lts.components.Powertrain.PowertrainComponent
    % EMRAX228POWERTRAIN EMRAX 228 electric powertrain from a MAT map
    % Uses the provided EMRAX228LC Single_3.36.mat data for motor torque and
    % tractive force with the final drive ratio stored in the map.
    %
    % The simulator consumes wheel torque, but the physical limiter is motor
    % speed. Each step updates PowertrainState from driven-wheel omega, then
    % this model reads the full-throttle tractive-force map by motor RPM,
    % scales by throttle request and efficiency, and returns total rear-axle
    % wheel torque for the differential to split.
    
    properties
        matFilePath = ""
        torqueSpeedCurve = []       % Vehicle-speed breakpoints for torque curve [m/s]
        speedCurve = []             % Vehicle speed breakpoints [m/s]
        motorRPMCurve = []          % Motor speed breakpoints [rpm]
        torqueCurveNm = []          % Motor torque breakpoints [Nm]
        tractiveForceCurveN = []    % Wheel tractive-force breakpoints [N]
        baseTractiveForceCurveN = [] % Map tractive force at mapGearRatio [N]
        state                       % lts.components.Powertrain.PowertrainState
        totalGearRatio = 3        % Final drive ratio [-]
        mapGearRatio = NaN        % Final drive ratio embedded in the MAT map [-]
        wheelRadius = 0.228        % Configured driven-tire rolling radius [m]
        mapWheelRadius = 0.228     % Radius used to encode MAT tractive force [m]
        drivetrainEfficiency = 0.92  % Motoring drivetrain efficiency [0-1]
        regenEfficiency = NaN        % Optional direct-mode regen drivetrain efficiency [0-1]
        motorRotorInertia = 0.07    % Motor rotor inertia [kg*m^2], reflected as I*ratio^2 to wheels
        % --- Coastdown / regen (off-throttle motoring) ---
        % Both default OFF to preserve baseline behavior. When regenEnabled is
        % true the motor also lifts the omega>=0 / speed>=0 clamps so it can
        % track reverse rotation and apply a coastdown drag torque.
        regenEnabled = false            % Apply regenerative braking at off-throttle
        motoringDragTorque = 0          % Motor coastdown drag torque [Nm] (motor-side), 0 = off
        motoringDragThrottleThreshold = Inf % Apply motoring drag at/below this throttle command [-]
        regenTorqueLimitNm = 30         % Max regen torque (motor-side) at off-throttle [Nm]
        regenEnabledSpeedFloor = 1.0    % Vehicle speed below which regen tapers to 0 [m/s]
        throttleDeadband = 0            % Pedal command below this produces zero drive torque [-]
        throttleMapInput = [0.00 0.15 0.35 0.60 0.80 1.00] % Post-deadband pedal breakpoints [-]
        throttleMapOutput = [0.00 0.02 0.10 0.28 0.58 1.00] % Motor torque/current request fraction [-]
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
            % EMRAX228POWERTRAIN Construct from EMRAX228LC Single_3.36.mat
            %   EMRAX228Powertrain()
            %   EMRAX228Powertrain(matFilePath)
            %   EMRAX228Powertrain(matFilePath, drivetrainEfficiency)
            %   EMRAX228Powertrain(matFilePath, drivetrainEfficiency, motorRotorInertia)

            if nargin < 1 || isempty(matFilePath)
                classDir = fileparts(mfilename('fullpath'));
                matFilePath = fullfile(classDir, 'EMRAX228LC Single_3.36.mat');
            end
            if nargin >= 2
                obj.drivetrainEfficiency = lts.util.saturate(drivetrainEfficiency);
            end
            if nargin >= 3 && ~isempty(motorRotorInertia)
                obj.motorRotorInertia = max(0, motorRotorInertia);
            end
            obj.state = lts.components.Powertrain.PowertrainState();
            
            data = load(matFilePath);
            obj.matFilePath = string(matFilePath);
            
            requiredFields = {'FDR', 'Speed', 'Tractive_force', 'Gearing_Map'};
            for i = 1:numel(requiredFields)
                if ~isfield(data, requiredFields{i})
                    error('EMRAX228Powertrain:MissingField', ...
                        'MAT file is missing required field "%s".', requiredFields{i});
                end
            end
            if isfield(data, 'Torque')
                rawTorque = data.Torque(:);
            elseif isfield(data, 'torque')
                rawTorque = data.torque(:);
            else
                error('EMRAX228Powertrain:MissingField', ...
                    'MAT file is missing required field "Torque" or "torque".');
            end
            
            obj.mapGearRatio = data.FDR;
            obj.totalGearRatio = obj.mapGearRatio;
            
            rawSpeed = data.Speed(:);
            rawForce = data.Tractive_force(:);
            obj.validateVectorSet(rawSpeed, rawTorque, rawForce, 'raw EMRAX vectors');
            
            [obj.torqueSpeedCurve, sortIdx] = sort(rawSpeed);
            obj.torqueCurveNm = rawTorque(sortIdx);
            rawForce = rawForce(sortIdx);
            
            validRadius = obj.torqueCurveNm > 0 & rawForce > 0;
            if any(validRadius)
                obj.mapWheelRadius = median(obj.torqueCurveNm(validRadius) .* ...
                    obj.totalGearRatio ./ rawForce(validRadius));
                obj.wheelRadius = obj.mapWheelRadius;
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
            obj.baseTractiveForceCurveN = obj.tractiveForceCurveN;
            
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

        function obj = setFinalDriveRatio(obj, ratio)
            % SETFINALDRIVERATIO Override the final drive ratio.
            %
            % The MAT gearing map stores wheel force for its original FDR.
            % When a tuning overlay changes FDR, preserve the same motor torque
            % curve by scaling wheel tractive force in proportion to the ratio.
            if isempty(ratio) || (isnumeric(ratio) && isscalar(ratio) && isnan(ratio))
                return;
            end
            ratio = double(ratio);
            if ~isscalar(ratio) || ~isfinite(ratio) || ratio <= 0
                error('EMRAX228Powertrain:InvalidFinalDriveRatio', ...
                    'Final drive ratio must be a positive finite scalar.');
            end

            if isempty(obj.baseTractiveForceCurveN)
                obj.baseTractiveForceCurveN = obj.tractiveForceCurveN;
            end
            if ~isfinite(obj.mapGearRatio) || obj.mapGearRatio <= 0
                obj.mapGearRatio = obj.totalGearRatio;
            end

            obj.totalGearRatio = ratio;
            obj.tractiveForceCurveN = obj.baseTractiveForceCurveN .* ...
                (obj.totalGearRatio / obj.mapGearRatio);
            if ~isempty(obj.motorRPMCurve)
                obj.speedCurve = obj.motorRPMCurve ./ obj.totalGearRatio .* ...
                    (2 * pi * obj.wheelRadius / 60);
                obj.maxVehicleSpeed = max(obj.speedCurve);
            end
        end

        function obj = setDrivenWheelRadius(obj, radius)
            % SETDRIVENWHEELRADIUS Synchronize the configured tire radius.
            %
            % The MAT file's tractive-force curve was generated with
            % mapWheelRadius. Keep that radius immutable for force-to-torque
            % conversion, while using the vehicle's actual rolling radius for
            % speed/RPM conversion and wheel-force reporting.
            radius = double(radius);
            if ~isscalar(radius) || ~isfinite(radius) || radius <= 0
                error('EMRAX228Powertrain:InvalidWheelRadius', ...
                    'Driven-wheel radius must be a positive finite scalar.');
            end
            obj.wheelRadius = radius;
            if ~isempty(obj.motorRPMCurve)
                obj.speedCurve = obj.motorRPMCurve ./ obj.totalGearRatio .* ...
                    (2 * pi * obj.wheelRadius / 60);
                obj.maxVehicleSpeed = max(obj.speedCurve);
            end
        end
        
        function wheelTorque = computeDriveTorque(obj, speed, throttle)
            % Compute total driven-axle wheel torque from current motor RPM.
            % The speed argument is only a fallback for initializing state.
            % Once wheel dynamics are active, motorRPM comes from the rear
            % wheels through updateStateFromDrivenWheels, so wheelspin and the
            % differential affect the rev limiter.
            throttle = lts.util.saturate(throttle);
            effectiveThrottle = obj.applyThrottleDeadband(throttle);
            torqueRequest = obj.mapThrottleToTorqueRequest(effectiveThrottle);
            
            if torqueRequest == 0
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
            
            % lookupTractiveForceByRPM is expressed at the radius embedded in
            % the MAT map. Convert it back to physical axle torque before
            % applying the configured tire radius anywhere.
            wheelTorque = fullThrottleForce * obj.mapWheelRadius * ...
                torqueRequest * obj.drivetrainEfficiency;
            equivalentDriveForce = wheelTorque / max(obj.wheelRadius, eps);
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

        function F_drive = computeMaxDriveForce(obj, speed)
            % COMPUTEMAXDRIVEFORCE Full-throttle wheel-equivalent drive force [N]
            %
            %   F_drive = computeMaxDriveForce(obj, speed)
            %
            %   Returns the full-throttle tractive force available at the given
            %   vehicle speed [m/s], WITHOUT mutating the live powertrain state.
            %   This is the planner-safe probe: it converts vehicle speed to a
            %   motor speed directly (independent of obj.state) and reads the
            %   tractive-force map, so the lap-time planner can sample capability
            %   across the speed profile without disturbing the simulator's
            %   per-step motor-state bookkeeping (which computeDriveTorque
            %   performs via obj.state.updateOutputs).
            speed = max(0, speed);
            motorRPM = obj.vehicleSpeedToMotorRPM(speed);
            fullThrottleForce = obj.lookupTractiveForceByRPM(motorRPM);
            wheelTorque = fullThrottleForce * obj.mapWheelRadius * ...
                obj.drivetrainEfficiency;
            F_drive = max(0, wheelTorque / max(obj.wheelRadius, eps));
        end

        function pedal = pedalForTorqueFraction(obj, fraction)
            % PEDALFORTORQUEFRACTION Invert the configured controller map.
            %   Returns the pedal command whose post-deadband nonlinear map
            %   produces the requested motor torque fraction. Zero request
            %   returns zero pedal; positive requests include the configured
            %   deadband offset. Flat portions of a valid monotonic map are
            %   supported and use the lowest pedal that produces an exact
            %   plateau value.
            fraction = double(fraction);
            if ~isscalar(fraction) || ~isfinite(fraction)
                error('EMRAX228Powertrain:InvalidTorqueFraction', ...
                    'Requested torque fraction must be a finite scalar.');
            end
            fraction = lts.util.saturate(fraction);
            if fraction <= 0
                pedal = 0;
                return;
            end

            [x, y] = obj.validatedThrottleMap();
            exactIdx = find(abs(y - fraction) <= 1e-12, 1, 'first');
            if ~isempty(exactIdx)
                effectiveThrottle = x(exactIdx);
            else
                segment = find(y(1:end-1) < fraction & ...
                    y(2:end) > fraction, 1, 'first');
                if isempty(segment)
                    effectiveThrottle = double(fraction >= y(end));
                else
                    blend = (fraction - y(segment)) / ...
                        (y(segment + 1) - y(segment));
                    effectiveThrottle = x(segment) + blend * ...
                        (x(segment + 1) - x(segment));
                end
            end

            deadband = obj.validThrottleDeadband();
            pedal = deadband + (1 - deadband) * effectiveThrottle;
            pedal = lts.util.saturate(pedal);
        end
        
        function updateStateFromDrivenWheels(obj, drivenWheelAngularVelocity)
            % Update motor RPM from driven-wheel angular velocity [rad/s].
            obj.state.allowReverseRotation = ...
                obj.regenEnabled || obj.motoringDragTorque > 0;
            obj.state.updateFromDrivenWheels( ...
                drivenWheelAngularVelocity, obj.totalGearRatio);
        end

        function updateStateFromVehicleSpeed(obj, vehicleSpeed)
            % Fallback update for callers without wheel rotational state.
            obj.state.allowReverseRotation = ...
                obj.regenEnabled || obj.motoringDragTorque > 0;
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
            % telemetry (lts.telemetry.GraphPlotter) and the sim agree on a single source of
            % truth. The MAT force is upstream of the separately configured
            % drivetrain efficiency, so T_motor = F_map*R_map/ratio.
            engineSpeed = max(0, engineSpeed);
            fullThrottleForce = obj.lookupTractiveForceByRPM(engineSpeed);
            if obj.totalGearRatio > 0
                torque = fullThrottleForce * obj.mapWheelRadius / ...
                    obj.totalGearRatio;
            else
                torque = 0;
            end
        end
        
        function ratio = getTotalGearRatio(obj)
            ratio = obj.totalGearRatio;
        end

        function I = getReflectedRotorInertia(obj)
            % GETREFLECTEDROTORINERTIA Motor rotor inertia reflected to the
            %   differential carrier [kg*m^2]. A gear ratio couples the rotor
            %   to carrier speed, so I_reflected = I_motor * ratio^2. The
            %   axle solver applies it to common-mode wheel acceleration.
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
                threshold = obj.motoringDragThrottleThreshold;
                if ~isfinite(threshold)
                    threshold = Inf;
                end
                if throttle <= lts.util.saturate(threshold)
                    T = T - motorSign * obj.motoringDragTorque * obj.totalGearRatio;
                end
            end
            if obj.regenEnabled && throttle == 0 && vehicleSpeed > 0
                % Taper regen to zero near rest so it cannot reverse the car.
                taper = min(1, vehicleSpeed / max(obj.regenEnabledSpeedFloor, eps));
                % Regen power flows from wheel to motor. Reflecting a
                % motor-side braking request to the wheel therefore reverses
                % the loss direction, matching Simulator's direct-command
                % convention: T_wheel = T_motor*ratio/eta_regen.
                effectiveRegenEfficiency = obj.getRegenDrivetrainEfficiency();
                T_regen = obj.regenTorqueLimitNm * obj.totalGearRatio / ...
                    max(effectiveRegenEfficiency, eps) * taper;
                T = T - motorSign * T_regen;
            end
        end
        
        function eff = getDrivetrainEfficiency(obj)
            eff = obj.drivetrainEfficiency;
        end

        function eff = getRegenDrivetrainEfficiency(obj)
            eff = obj.regenEfficiency;
            if ~isfinite(eff)
                eff = obj.drivetrainEfficiency;
            end
            eff = lts.util.saturate(eff);
        end
    end
    
    methods (Access = private)
        function effectiveThrottle = applyThrottleDeadband(obj, throttle)
            deadband = obj.validThrottleDeadband();
            if throttle <= deadband
                effectiveThrottle = 0;
            else
                effectiveThrottle = (throttle - deadband) / (1 - deadband);
            end
        end

        function torqueRequest = mapThrottleToTorqueRequest(obj, effectiveThrottle)
            % MAPTHROTTLETOTORQUEREQUEST Convert pedal to controller request.
            % EV inverters such as the BAMOCAR command motor current/torque
            % rather than output power directly. The RPM map remains the
            % full-throttle capability envelope; this curve shapes how much
            % of that envelope a partial pedal command requests.
            effectiveThrottle = lts.util.saturate(effectiveThrottle);
            [x, y] = obj.validatedThrottleMap();

            torqueRequest = interp1(x, y, effectiveThrottle, 'linear');
            torqueRequest = lts.util.saturate(torqueRequest);
        end

        function deadband = validThrottleDeadband(obj)
            deadband = obj.throttleDeadband;
            if ~isfinite(deadband)
                deadband = 0;
            end
            deadband = lts.util.clamp(deadband, 0, 0.99);
        end

        function [x, y] = validatedThrottleMap(obj)
            x = obj.throttleMapInput(:);
            y = obj.throttleMapOutput(:);

            if isempty(x) || isempty(y) || numel(x) ~= numel(y) || numel(x) < 2 || ...
                    any(~isfinite(x)) || any(~isfinite(y)) || ...
                    any(x < 0) || any(x > 1) || any(y < 0) || any(y > 1) || ...
                    any(diff(x) <= 0) || any(diff(y) < 0) || ...
                    abs(x(1)) > 1e-12 || abs(x(end) - 1) > 1e-12 || ...
                    abs(y(1)) > 1e-12 || abs(y(end) - 1) > 1e-12
                error('EMRAX228Powertrain:InvalidThrottleMap', ...
                    ['Throttle map input/output must be equal-length finite vectors; ' ...
                    'input must increase from 0 to 1 and output must be ' ...
                    'nondecreasing from 0 to 1.']);
            end
        end

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
            multiplier = lts.util.saturate(multiplier);
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
