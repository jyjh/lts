classdef CorrelationReplayProfile
    % CORRELATIONREPLAYPROFILE Normalized real-lap controls for replay runs.
    %
    % The normalized CSV contract is intentionally small and stable:
    % time_s, distance_m, throttle_ratio, brake_ratio,
    % brake_pressure_front_bar, brake_pressure_rear_bar, regen_torque_nm,
    % motor_torque_command_nm, motor_torque_delivered_nm, motor_rpm,
    % pack_voltage_v, pack_current_a,
    % steer_rad, speed_mps,
    % with optional yaw_rad, yaw_rate_radps, x_m, y_m, GPS/course,
    % measured acceleration channels, and per-corner wheel speeds.

    properties
        sourceFile = ''
        time = []
        distance = []
        throttle = []
        brake = []
        brakePressureFrontBar = []
        brakePressureRearBar = []
        regenTorqueNm = []
        motorTorqueCommandNm = []
        motorTorqueDeliveredNm = []
        motorRpm = []
        packVoltageV = []
        packCurrentA = []
        steer = []
        speed = []
        wheelSpeedFL = []
        wheelSpeedFR = []
        wheelSpeedRL = []
        wheelSpeedRR = []
        vx = []
        vy = []
        bodySlip = []
        yaw = []
        yawRate = []
        x = []
        y = []
        gpsLat = []
        gpsLon = []
        gpsCourse = []
        latAccelG = []
        frontLatAccelG = []
        rearLatAccelG = []
        frontLongAccelG = []
        rearLongAccelG = []
        longAccelG = []
    end

    properties (Access = private)
        timeSampleCache = struct()
        distanceSampleCache = struct()
    end

    methods
        function obj = CorrelationReplayProfile(varargin)
            if nargin == 0
                return;
            end

            parser = inputParser;
            parser.addParameter('SourceFile', '', @(x) ischar(x) || isstring(x));
            channelNames = { ...
                'Time','Distance','Throttle','Brake','BrakePressureFrontBar', ...
                'BrakePressureRearBar','RegenTorqueNm','MotorTorqueCommandNm', ...
                'MotorTorqueDeliveredNm','MotorRpm','PackVoltageV','PackCurrentA', ...
                'Steer','Speed','WheelSpeedFL','WheelSpeedFR','WheelSpeedRL', ...
                'WheelSpeedRR','Vx','Vy','BodySlip','Yaw','YawRate','X','Y', ...
                'GpsLat','GpsLon','GpsCourse','LatAccelG','FrontLatAccelG', ...
                'RearLatAccelG','FrontLongAccelG','RearLongAccelG','LongAccelG'};
            for i = 1:numel(channelNames)
                parser.addParameter(channelNames{i}, [], @isnumeric);
            end
            parser.parse(varargin{:});

            obj.sourceFile = char(parser.Results.SourceFile);
            for i = 1:numel(channelNames)
                property = channelNames{i};
                property(1) = lower(property(1));
                obj.(property) = parser.Results.(channelNames{i})(:);
            end
            obj = obj.validateAndComplete();
        end

        function input = sampleByTime(obj, time)
            input = obj.sampleAt(obj.time, obj.timeSampleCache, time);
        end

        function input = sampleByDistance(obj, distance)
            if isempty(obj.distance) || all(~isfinite(obj.distance))
                error('lts_correlation_CorrelationReplayProfile:MissingDistance', ...
                    'Distance-domain replay requires a distance_m channel or speed-derived distance.');
            end
            input = obj.sampleAt(obj.distance, obj.distanceSampleCache, distance);
        end

        function value = initialSpeed(obj)
            value = obj.speed(1);
        end

        function tf = hasInitialWheelSpeeds(obj)
            speeds = obj.initialWheelSpeeds();
            tf = any(isfinite([speeds.FL, speeds.FR, speeds.RL, speeds.RR]));
        end

        function speeds = initialWheelSpeeds(obj)
            speeds = struct( ...
                'FL', obj.wheelSpeedFL(1), ...
                'FR', obj.wheelSpeedFR(1), ...
                'RL', obj.wheelSpeedRL(1), ...
                'RR', obj.wheelSpeedRR(1));
        end

        function tf = hasWheelSpeeds(obj)
            tf = any(isfinite([ ...
                obj.wheelSpeedFL(:); ...
                obj.wheelSpeedFR(:); ...
                obj.wheelSpeedRL(:); ...
                obj.wheelSpeedRR(:)]));
        end

        function tf = hasYaw(obj)
            tf = ~isempty(obj.yaw) && isfinite(obj.yaw(1));
        end

        function tf = hasVelocity(obj)
            tf = ~isempty(obj.vx) && ~isempty(obj.vy) && ...
                isfinite(obj.vx(1)) && isfinite(obj.vy(1));
        end

        function tf = hasBodySlip(obj)
            tf = ~isempty(obj.bodySlip) && isfinite(obj.bodySlip(1));
        end

        function tf = hasBrakePressure(obj)
            tf = (~isempty(obj.brakePressureFrontBar) && ...
                any(isfinite(obj.brakePressureFrontBar))) || ...
                (~isempty(obj.brakePressureRearBar) && ...
                any(isfinite(obj.brakePressureRearBar)));
        end

        function tf = hasRegenTorque(obj)
            tf = ~isempty(obj.regenTorqueNm) && any(isfinite(obj.regenTorqueNm));
        end

        function tf = hasMotorTorqueCommand(obj)
            tf = ~isempty(obj.motorTorqueCommandNm) && ...
                any(isfinite(obj.motorTorqueCommandNm));
        end

        function tf = hasMotorRpm(obj)
            tf = ~isempty(obj.motorRpm) && any(isfinite(obj.motorRpm));
        end

        function tf = hasPackPower(obj)
            tf = ~isempty(obj.packVoltageV) && ~isempty(obj.packCurrentA) && ...
                any(isfinite(obj.packVoltageV) & isfinite(obj.packCurrentA));
        end

        function tf = hasPosition(obj)
            tf = ~isempty(obj.x) && ~isempty(obj.y) && ...
                isfinite(obj.x(1)) && isfinite(obj.y(1));
        end

        function tf = hasGpsCourse(obj)
            tf = ~isempty(obj.gpsCourse) && any(isfinite(obj.gpsCourse));
        end

        function tf = hasLatAccel(obj)
            tf = (~isempty(obj.latAccelG) && any(isfinite(obj.latAccelG))) || ...
                (~isempty(obj.frontLatAccelG) && any(isfinite(obj.frontLatAccelG))) || ...
                (~isempty(obj.rearLatAccelG) && any(isfinite(obj.rearLatAccelG)));
        end

        function tf = hasLongAccel(obj)
            tf = (~isempty(obj.longAccelG) && any(isfinite(obj.longAccelG))) || ...
                (~isempty(obj.frontLongAccelG) && any(isfinite(obj.frontLongAccelG))) || ...
                (~isempty(obj.rearLongAccelG) && any(isfinite(obj.rearLongAccelG)));
        end

        function duration = duration(obj)
            duration = obj.time(end) - obj.time(1);
        end

        function distance = totalDistance(obj)
            distance = obj.distance(end) - obj.distance(1);
        end

        function out = window(obj, startTimeS, horizonS)
            %WINDOW Extract an exact, time-rebased replay segment.
            %   All public per-sample channels are interpolated onto a common
            %   axis containing the requested boundaries. Distance is rebased
            %   alongside time so the segment can be passed directly to the
            %   existing replay driver and state initializer.
            if ~isnumeric(startTimeS) || ~isscalar(startTimeS) || ...
                    ~isfinite(startTimeS)
                error('lts_correlation_CorrelationReplayProfile:InvalidWindowStart', ...
                    'Window start must be a finite scalar in seconds.');
            end
            if ~isnumeric(horizonS) || ~isscalar(horizonS) || ...
                    ~isfinite(horizonS) || horizonS <= 0
                error('lts_correlation_CorrelationReplayProfile:InvalidWindowHorizon', ...
                    'Window horizon must be a positive finite scalar in seconds.');
            end

            sourceStart = obj.time(1);
            sourceEnd = obj.time(end);
            startTimeS = double(startTimeS);
            endTimeS = startTimeS + double(horizonS);
            tolerance = 32 * eps(max(1, max(abs([sourceStart, sourceEnd]))));
            if startTimeS < sourceStart - tolerance || ...
                    endTimeS > sourceEnd + tolerance
                error('lts_correlation_CorrelationReplayProfile:WindowOutsideProfile', ...
                    ['Requested window [%.6g, %.6g] s is outside replay ' ...
                    'range [%.6g, %.6g] s.'], ...
                    startTimeS, endTimeS, sourceStart, sourceEnd);
            end
            startTimeS = max(sourceStart, startTimeS);
            endTimeS = min(sourceEnd, endTimeS);

            interior = obj.time(obj.time > startTimeS & obj.time < endTimeS);
            queryTime = unique([startTimeS; interior(:); endTimeS], 'stable');
            if numel(queryTime) < 2
                error('lts_correlation_CorrelationReplayProfile:WindowTooShort', ...
                    'Replay window must contain at least two samples.');
            end

            out = obj;
            n = numel(obj.time);
            channelNames = properties(obj);
            for i = 1:numel(channelNames)
                name = channelNames{i};
                values = obj.(name);
                if isnumeric(values) && isvector(values) && numel(values) == n
                    out.(name) = obj.interpolateWindowChannel(values(:), queryTime);
                end
            end
            out.time = queryTime - queryTime(1);
            if ~isempty(out.distance)
                firstDistance = out.distance(1);
                if isfinite(firstDistance)
                    out.distance = out.distance - firstDistance;
                end
            end
            out = out.buildSampleCaches();
        end

        function [obj, report] = withGpsKinematics(obj, varargin)
            %WITHGPSKINEMATICS Make GPS position the body-motion authority.
            % Position is converted to a local east/north frame. Smoothed
            % derivatives provide vehicle speed and body-frame acceleration;
            % wheel-speed and axle accelerometer channels remain untouched.
            parser = inputParser;
            parser.addParameter('SmoothingWindowS', 0.35, ...
                @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0);
            parser.addParameter('MinimumSpeedMps', 1.0, ...
                @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0);
            parser.parse(varargin{:});
            report = struct('status', "unavailable", 'sampleCount', 0, ...
                'smoothingWindowS', double(parser.Results.SmoothingWindowS));

            valid = isfinite(obj.time) & isfinite(obj.gpsLat) & isfinite(obj.gpsLon);
            if nnz(valid) < 5
                return;
            end
            latitude = obj.interpolateFinite(obj.gpsLat);
            longitude = obj.interpolateFinite(obj.gpsLon);
            origin = find(isfinite(latitude) & isfinite(longitude), 1, 'first');
            if isempty(origin)
                return;
            end
            earthRadiusM = 6371008.8;
            lat0 = deg2rad(latitude(origin));
            xEast = earthRadiusM .* deg2rad(longitude - longitude(origin)) .* cos(lat0);
            yNorth = earthRadiusM .* deg2rad(latitude - latitude(origin));

            dt = median(diff(obj.time(isfinite(obj.time) & [true; diff(obj.time) > 0])));
            if ~isfinite(dt) || dt <= 0
                return;
            end
            samples = max(3, round(parser.Results.SmoothingWindowS / dt));
            if mod(samples, 2) == 0
                samples = samples + 1;
            end
            % Preserve the measured GPS trace itself. Differentiate first,
            % then smooth velocity/acceleration; smoothing position with a
            % truncated endpoint window halves the initial speed.
            dxdt = gradient(xEast) ./ gradient(obj.time);
            dydt = gradient(yNorth) ./ gradient(obj.time);
            dxdt = movmean(dxdt, samples, 'Endpoints', 'shrink');
            dydt = movmean(dydt, samples, 'Endpoints', 'shrink');
            speedGps = hypot(dxdt, dydt);
            axEast = gradient(dxdt) ./ gradient(obj.time);
            ayNorth = gradient(dydt) ./ gradient(obj.time);
            axEast = movmean(axEast, samples, 'Endpoints', 'shrink');
            ayNorth = movmean(ayNorth, samples, 'Endpoints', 'shrink');

            moving = speedGps >= parser.Results.MinimumSpeedMps;
            longAccel = nan(size(speedGps));
            latAccel = nan(size(speedGps));
            longAccel(moving) = (dxdt(moving) .* axEast(moving) + ...
                dydt(moving) .* ayNorth(moving)) ./ speedGps(moving);
            latAccel(moving) = (dxdt(moving) .* ayNorth(moving) - ...
                dydt(moving) .* axEast(moving)) ./ speedGps(moving);

            obj.x = xEast(:);
            obj.y = yNorth(:);
            obj.speed = max(0, speedGps(:));
            obj.longAccelG = longAccel(:) ./ 9.80665;
            obj.latAccelG = latAccel(:) ./ 9.80665;
            obj.distance = cumtrapz(obj.time, obj.speed);
            obj = obj.buildSampleCaches();
            report.status = "applied";
            report.sampleCount = nnz(isfinite(obj.speed));
        end

        function obj = withPackPowerAdvance(obj, advanceS)
            if nargin < 2 || isempty(advanceS)
                advanceS = 0;
            end
            if ~isnumeric(advanceS) || ~isscalar(advanceS) || ~isfinite(advanceS)
                error('lts_correlation_CorrelationReplayProfile:InvalidPackPowerAdvance', ...
                    'Pack power channel advance must be a finite scalar in seconds.');
            end

            advanceS = double(advanceS);
            if advanceS == 0 || ~obj.hasPackPower()
                return;
            end

            queryTime = obj.time + advanceS;
            obj.packVoltageV = obj.shiftTimeChannel(obj.packVoltageV, queryTime);
            obj.packCurrentA = obj.shiftTimeChannel(obj.packCurrentA, queryTime);
            obj = obj.buildSampleCaches();
        end

        function [obj, report] = withPowerConservingRegenRepair(obj, varargin)
            % Replace delivered-torque samples that cannot produce measured
            % pack charging power. Unity efficiency gives the smallest
            % physically admissible shaft-regeneration magnitude.
            parser = inputParser;
            parser.addParameter('PackPowerAdvanceS', -0.015, ...
                @(x) isnumeric(x) && isscalar(x) && isfinite(x));
            parser.addParameter('MinimumChargingPowerW', 1000, ...
                @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0);
            parser.addParameter('MinimumMotorSpeedRpm', 300, ...
                @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0);
            parser.addParameter('RegenRequestThresholdNm', 5, ...
                @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0);
            parser.addParameter('PowerToleranceW', 500, ...
                @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0);
            parser.addParameter('MaximumRegenEfficiency', 1, ...
                @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0 && x <= 1);
            parser.addParameter('MaximumReconstructedTorqueNm', inf, ...
                @(x) isnumeric(x) && isscalar(x) && x > 0);
            parser.parse(varargin{:});
            opts = parser.Results;

            report = struct( ...
                'status', "unavailable", ...
                'repairedSampleCount', 0, ...
                'signContradictionSampleCount', 0, ...
                'powerDeficitSampleCount', 0, ...
                'remainingPowerDeficitSampleCount', 0, ...
                'maximumTorqueCorrectionNm', 0);
            if ~obj.hasMotorTorqueDelivered() || ~obj.hasPackPower() || ...
                    ~obj.hasMotorRpm() || ...
                    (~obj.hasMotorTorqueCommand() && ~obj.hasRegenTorque())
                return;
            end

            queryTime = obj.time + double(opts.PackPowerAdvanceS);
            alignedVoltage = obj.shiftTimeChannel(obj.packVoltageV, queryTime);
            alignedCurrent = obj.shiftTimeChannel(obj.packCurrentA, queryTime);
            chargingPowerW = max(0, -alignedVoltage .* alignedCurrent);
            rpmMagnitude = abs(obj.motorRpm);
            motorOmega = rpmMagnitude * (2 * pi / 60);
            measuredTorqueNm = obj.motorTorqueDeliveredNm;
            measuredRegenPowerW = max(0, -measuredTorqueNm) .* motorOmega;
            % The R25 regen channel is the available negative-torque limit,
            % not the torque currently requested. Taking min(limit, command)
            % falsely labels positive-torque transitions as regen. Prefer the
            % signed motor command and use the regen channel only when that
            % command is unavailable.
            regenRequestNm = obj.motorTorqueCommandNm;
            missingRequest = ~isfinite(regenRequestNm);
            regenRequestNm(missingRequest) = ...
                obj.regenTorqueNm(missingRequest);

            active = isfinite(measuredTorqueNm) & isfinite(regenRequestNm) & ...
                isfinite(chargingPowerW) & isfinite(motorOmega) & ...
                regenRequestNm <= -double(opts.RegenRequestThresholdNm) & ...
                chargingPowerW >= double(opts.MinimumChargingPowerW) & ...
                rpmMagnitude >= double(opts.MinimumMotorSpeedRpm);
            signContradiction = active & measuredTorqueNm >= 0;
            powerDeficit = active & chargingPowerW > ...
                double(opts.MaximumRegenEfficiency) .* measuredRegenPowerW + ...
                double(opts.PowerToleranceW);
            repairMask = signContradiction | powerDeficit;

            minimumTorqueNm = nan(size(measuredTorqueNm));
            positiveSpeed = motorOmega > eps;
            minimumTorqueNm(positiveSpeed) = ...
                -chargingPowerW(positiveSpeed) ./ ...
                (double(opts.MaximumRegenEfficiency) .* motorOmega(positiveSpeed));
            if isfinite(opts.MaximumReconstructedTorqueNm)
                minimumTorqueNm = max( ...
                    minimumTorqueNm, -abs(double(opts.MaximumReconstructedTorqueNm)));
            end

            repairedTorqueNm = measuredTorqueNm;
            repairedTorqueNm(repairMask) = min( ...
                measuredTorqueNm(repairMask), minimumTorqueNm(repairMask));
            obj.motorTorqueDeliveredNm = repairedTorqueNm;
            obj = obj.buildSampleCaches();

            remainingPowerDeficit = repairMask & chargingPowerW > ...
                double(opts.MaximumRegenEfficiency) .* ...
                max(0, -repairedTorqueNm) .* motorOmega + ...
                double(opts.PowerToleranceW);
            correctionNm = repairedTorqueNm - measuredTorqueNm;
            report.status = "ok";
            report.repairedSampleCount = nnz(repairMask);
            report.signContradictionSampleCount = nnz(signContradiction);
            report.powerDeficitSampleCount = nnz(powerDeficit);
            report.remainingPowerDeficitSampleCount = nnz(remainingPowerDeficit);
            if any(repairMask)
                report.maximumTorqueCorrectionNm = ...
                    max(abs(correctionNm(repairMask)));
            end
            report.packPowerAdvanceS = double(opts.PackPowerAdvanceS);
            report.maximumRegenEfficiency = double(opts.MaximumRegenEfficiency);
            report.powerToleranceW = double(opts.PowerToleranceW);
        end

        function obj = withMotorTorqueCommandDelay(obj, delayS)
            if nargin < 2 || isempty(delayS)
                delayS = 0;
            end
            if ~isnumeric(delayS) || ~isscalar(delayS) || ~isfinite(delayS)
                error('lts_correlation_CorrelationReplayProfile:InvalidMotorTorqueCommandDelay', ...
                    'Motor torque command delay must be a finite scalar in seconds.');
            end

            delayS = double(delayS);
            if delayS == 0 || ~obj.hasMotorTorqueCommand()
                return;
            end

            queryTime = obj.time - delayS;
            obj.motorTorqueCommandNm = obj.shiftTimeChannel( ...
                obj.motorTorqueCommandNm, queryTime);
            if obj.hasRegenTorque()
                obj.regenTorqueNm = obj.shiftTimeChannel(obj.regenTorqueNm, queryTime);
            end
            obj = obj.buildSampleCaches();
        end

        function tf = hasMotorTorqueDelivered(obj)
            tf = ~isempty(obj.motorTorqueDeliveredNm) && ...
                any(isfinite(obj.motorTorqueDeliveredNm));
        end

        function obj = withSteeringCalibration( ...
                obj, centerGain, endAngleRad, delayS, centerOffsetRad)
            if nargin < 2 || isempty(centerGain)
                centerGain = 1;
            end
            if nargin < 3 || isempty(endAngleRad)
                endAngleRad = pi;
            end
            if nargin < 4 || isempty(delayS)
                delayS = 0;
            end
            if nargin < 5 || isempty(centerOffsetRad)
                centerOffsetRad = 0;
            end
            if ~isnumeric(centerGain) || ~isscalar(centerGain) || ...
                    ~isfinite(centerGain) || centerGain < 0 || centerGain > 2
                error('lts_correlation_CorrelationReplayProfile:InvalidSteeringCenterGain', ...
                    'Steering center gain must be a finite scalar from 0 to 2.');
            end
            if ~isnumeric(endAngleRad) || ~isscalar(endAngleRad) || ...
                    ~isfinite(endAngleRad) || endAngleRad <= 0
                error('lts_correlation_CorrelationReplayProfile:InvalidSteeringCalibrationEndAngle', ...
                    'Steering calibration end angle must be a positive finite scalar in radians.');
            end
            if ~isnumeric(delayS) || ~isscalar(delayS) || ...
                    ~isfinite(delayS) || delayS < 0
                error('lts_correlation_CorrelationReplayProfile:InvalidSteeringDelay', ...
                    'Steering delay must be a nonnegative finite scalar in seconds.');
            end
            if ~isnumeric(centerOffsetRad) || ~isscalar(centerOffsetRad) || ...
                    ~isfinite(centerOffsetRad)
                error('lts_correlation_CorrelationReplayProfile:InvalidSteeringCenterOffset', ...
                    'Steering center offset must be a finite scalar in radians.');
            end

            centerGain = double(centerGain);
            endAngleRad = double(endAngleRad);
            delayS = double(delayS);
            centerOffsetRad = double(centerOffsetRad);
            if centerGain ~= 1 || centerOffsetRad ~= 0
                magnitude = abs(obj.steer);
                fraction = min(magnitude ./ endAngleRad, 1);
                correctedMagnitude = endAngleRad .* ( ...
                    centerGain .* fraction + (1 - centerGain) .* fraction.^2);
                corrected = sign(obj.steer) .* correctedMagnitude + ...
                    centerOffsetRad .* (1 - fraction).^2;
                corrected(magnitude > endAngleRad) = obj.steer(magnitude > endAngleRad);
                obj.steer = corrected;
            end
            if delayS > 0
                obj.steer = obj.shiftTimeChannel(obj.steer, obj.time - delayS);
            end
            obj = obj.buildSampleCaches();
        end
    end

    methods (Access = private)
        function obj = validateAndComplete(obj)
            required = {'time', 'throttle', 'brake', 'steer', 'speed'};
            for i = 1:numel(required)
                field = required{i};
                if isempty(obj.(field))
                    error('lts_correlation_CorrelationReplayProfile:MissingField', ...
                        'Replay profile is missing required field "%s".', field);
                end
            end

            n = numel(obj.time);
            if n < 2
                error('lts_correlation_CorrelationReplayProfile:TooShort', ...
                    'Replay profile must contain at least two samples.');
            end

            obj.time = obj.time - obj.time(1);
            obj.time = obj.requireColumnLength(obj.time, n, 'time');
            if any(~isfinite(obj.time)) || any(diff(obj.time) <= 0)
                error('lts_correlation_CorrelationReplayProfile:InvalidTime', ...
                    'time_s must be finite and strictly increasing.');
            end

            obj.throttle = obj.clamp01(obj.requireColumnLength(obj.throttle, n, 'throttle'));
            obj.brake = obj.clamp01(obj.requireColumnLength(obj.brake, n, 'brake'));
            optionalFields = { ...
                'brakePressureFrontBar','brakePressureRearBar','regenTorqueNm', ...
                'motorTorqueCommandNm','motorTorqueDeliveredNm','motorRpm', ...
                'packVoltageV','packCurrentA'};
            for i = 1:numel(optionalFields)
                field = optionalFields{i};
                obj.(field) = obj.optionalColumn(obj.(field), n, NaN, field);
            end
            obj.steer = obj.requireColumnLength(obj.steer, n, 'steer');
            obj.speed = max(0, obj.requireColumnLength(obj.speed, n, 'speed'));
            wheelFields = {'wheelSpeedFL','wheelSpeedFR','wheelSpeedRL','wheelSpeedRR'};
            for i = 1:numel(wheelFields)
                field = wheelFields{i};
                obj.(field) = obj.nonnegativeOptionalColumn(obj.(field), n, field);
            end

            if isempty(obj.distance) || all(~isfinite(obj.distance))
                obj.distance = obj.integrateDistanceFromSpeed();
            else
                obj.distance = obj.requireColumnLength(obj.distance, n, 'distance');
                obj.distance = obj.distance - obj.distance(1);
            end

            optionalFields = { ...
                'vx','vy','bodySlip','yaw','yawRate','x','y','gpsLat','gpsLon', ...
                'gpsCourse','latAccelG','frontLatAccelG','rearLatAccelG', ...
                'frontLongAccelG','rearLongAccelG','longAccelG'};
            for i = 1:numel(optionalFields)
                field = optionalFields{i};
                obj.(field) = obj.optionalColumn(obj.(field), n, NaN, field);
            end
            if all(~isfinite(obj.frontLatAccelG)) && any(isfinite(obj.latAccelG))
                obj.frontLatAccelG = obj.latAccelG;
            end
            if all(~isfinite(obj.frontLongAccelG)) && any(isfinite(obj.longAccelG))
                obj.frontLongAccelG = obj.longAccelG;
            end
            obj = obj.buildSampleCaches();
        end

        function values = requireColumnLength(~, values, n, fieldName)
            values = double(values(:));
            if numel(values) ~= n
                error('lts_correlation_CorrelationReplayProfile:LengthMismatch', ...
                    'Field "%s" has %d samples, expected %d.', ...
                    fieldName, numel(values), n);
            end
        end

        function values = optionalColumn(~, values, n, defaultValue, fieldName)
            if isempty(values)
                values = repmat(defaultValue, n, 1);
                return;
            end
            values = double(values(:));
            if numel(values) ~= n
                error('lts_correlation_CorrelationReplayProfile:LengthMismatch', ...
                    'Field "%s" has %d samples, expected %d.', ...
                    fieldName, numel(values), n);
            end
        end

        function values = nonnegativeOptionalColumn(obj, values, n, fieldName)
            values = obj.optionalColumn(values, n, NaN, fieldName);
            values(values < 0) = 0;
        end

        function distance = integrateDistanceFromSpeed(obj)
            dt = diff(obj.time);
            v = obj.speed;
            distance = [0; cumsum(0.5 * (v(1:end-1) + v(2:end)) .* dt)];
        end

        function values = clamp01(~, values)
            values(~isfinite(values)) = 0;
            values = lts.util.saturate(values);
        end

        function input = sampleAt(obj, axis, cache, query)
            query = double(query);
            if ~isfinite(query)
                query = axis(1);
            end
            query = max(axis(1), min(axis(end), query));

            % P4-A: Batch path — single interp1 call over all channels stored
            % in a pre-built N×13 matrix. Replaces 12 individual griddedInterpolant
            % calls (each with their own lookup overhead) with one vectorised op.
            if ~isempty(cache.batchAxis)
                q = max(cache.batchAxis(1), min(cache.batchAxis(end), query));
                row = interp1(cache.batchAxis, cache.batchMatrix, q, 'linear');
                input = obj.sampleRowToInput(row);
                return;
            end

            % Fallback: per-channel lookup (used only when batch cache build failed).
            input = struct( ...
                'throttle', obj.lookup(cache.throttle, query), ...
                'brake', obj.lookup(cache.brake, query), ...
                'brakePressureFrontBar', obj.lookup(cache.brakePressureFrontBar, query), ...
                'brakePressureRearBar', obj.lookup(cache.brakePressureRearBar, query), ...
                'regenTorqueNm', obj.lookup(cache.regenTorqueNm, query), ...
                'motorTorqueCommandNm', obj.lookup(cache.motorTorqueCommandNm, query), ...
                'motorTorqueDeliveredNm', obj.lookup(cache.motorTorqueDeliveredNm, query), ...
                'motorRpm', obj.lookup(cache.motorRpm, query), ...
                'packVoltageV', obj.lookup(cache.packVoltageV, query), ...
                'packCurrentA', obj.lookup(cache.packCurrentA, query), ...
                'steer', obj.lookup(cache.steer, query), ...
                'targetSpeed', obj.lookup(cache.speed, query), ...
                'axRef', NaN, ...
                'sourceTime', obj.lookup(cache.time, query), ...
                'sourceDistance', obj.lookup(cache.distance, query));
        end

        function obj = buildSampleCaches(obj)
            obj.timeSampleCache = obj.buildAxisCache(obj.time);
            obj.distanceSampleCache = obj.buildAxisCache(obj.distance);
        end

        function cache = buildAxisCache(obj, axis)
            fields = obj.sampleFieldOrder();
            cache = struct();
            for i = 1:numel(fields)
                field = fields{i};
                cache.(field) = obj.buildInterpCache(axis, obj.(field));
            end

            cache.batchAxis = [];
            cache.batchMatrix = [];
            for i = 1:numel(fields)
                channel = cache.(fields{i});
                if ~channel.isMissing && ~channel.isScalar
                    cache.batchAxis = channel.axis(:);
                    break;
                end
            end
            if isempty(cache.batchAxis)
                return;
            end

            cache.batchMatrix = NaN(numel(cache.batchAxis), numel(fields));
            for i = 1:numel(fields)
                channel = cache.(fields{i});
                if channel.isScalar
                    cache.batchMatrix(:, i) = channel.values(1);
                elseif ~channel.isMissing
                    query = max(channel.axis(1), ...
                        min(channel.axis(end), cache.batchAxis));
                    cache.batchMatrix(:, i) = channel.interpolant(query);
                end
            end
        end

        function cache = buildInterpCache(~, axis, values)
            values = double(values(:));
            if all(~isfinite(values))
                cache = struct('axis', [], 'values', [], ...
                    'interpolant', [], 'isMissing', true, 'isScalar', false);
                return;
            end

            keep = isfinite(axis(:)) & isfinite(values);
            axis = axis(keep);
            values = values(keep);
            [axis, ia] = unique(axis, 'sorted');
            values = values(ia);

            if isempty(axis)
                cache = struct('axis', [], 'values', [], ...
                    'interpolant', [], 'isMissing', true, 'isScalar', false);
            elseif isscalar(axis)
                cache = struct('axis', axis, 'values', values, ...
                    'interpolant', [], 'isMissing', false, 'isScalar', true);
            else
                interpolant = griddedInterpolant(axis, values, 'linear', 'nearest');
                cache = struct('axis', axis, 'values', values, ...
                    'interpolant', interpolant, 'isMissing', false, 'isScalar', false);
            end
        end

        function value = lookup(~, cache, query)
            if cache.isMissing
                value = NaN;
            elseif cache.isScalar
                value = cache.values(1);
            else
                query = max(cache.axis(1), min(cache.axis(end), query));
                value = cache.interpolant(query);
            end
        end

        function input = sampleRowToInput(~, row)
            input = struct( ...
                'throttle', row(1), ...
                'brake', row(2), ...
                'brakePressureFrontBar', row(3), ...
                'brakePressureRearBar', row(4), ...
                'regenTorqueNm', row(5), ...
                'motorTorqueCommandNm', row(6), ...
                'motorTorqueDeliveredNm', row(7), ...
                'motorRpm', row(8), ...
                'packVoltageV', row(9), ...
                'packCurrentA', row(10), ...
                'steer', row(11), ...
                'targetSpeed', row(12), ...
                'axRef', NaN, ...
                'sourceTime', row(13), ...
                'sourceDistance', row(14));
        end

        function fields = sampleFieldOrder(~)
            fields = {'throttle', 'brake', ...
                'brakePressureFrontBar', 'brakePressureRearBar', ...
                'regenTorqueNm', 'motorTorqueCommandNm', ...
                'motorTorqueDeliveredNm', 'motorRpm', ...
                'packVoltageV', 'packCurrentA', 'steer', 'speed', ...
                'time', 'distance'};
        end

        function values = shiftTimeChannel(obj, sourceValues, queryTime)
            cache = obj.buildInterpCache(obj.time, sourceValues);
            if cache.isMissing
                values = NaN(size(obj.time));
            elseif cache.isScalar
                values = repmat(cache.values(1), size(obj.time));
            else
                queryTime = max(cache.axis(1), min(cache.axis(end), queryTime(:)));
                values = cache.interpolant(queryTime);
            end
        end

        function values = interpolateWindowChannel(obj, sourceValues, queryTime)
            valid = isfinite(obj.time) & isfinite(sourceValues);
            if nnz(valid) >= 2
                values = interp1(obj.time(valid), sourceValues(valid), ...
                    queryTime, 'linear', NaN);
            elseif nnz(valid) == 1
                values = repmat(sourceValues(find(valid, 1)), size(queryTime));
            else
                values = nan(size(queryTime));
            end
            values = values(:);
        end

        function values = interpolateFinite(obj, sourceValues)
            valid = isfinite(obj.time) & isfinite(sourceValues);
            if nnz(valid) < 2
                values = nan(size(obj.time));
                return;
            end
            sourceTime = obj.time(valid);
            sourceValues = sourceValues(valid);
            [sourceTime, uniqueIndex] = unique(sourceTime, 'stable');
            sourceValues = sourceValues(uniqueIndex);
            values = interp1(sourceTime, sourceValues, obj.time, 'linear', NaN);
            values(obj.time < sourceTime(1)) = sourceValues(1);
            values(obj.time > sourceTime(end)) = sourceValues(end);
            values = values(:);
        end
    end

    methods (Static)
        function obj = fromCsv(filepath)
            filepath = char(filepath);
            if ~exist(filepath, 'file')
                error('lts_correlation_CorrelationReplayProfile:MissingCsv', ...
                    'Replay CSV "%s" does not exist.', filepath);
            end

            [ok, obj] = lts.correlation.CorrelationReplayProfile.tryReadNumericCsv(filepath);
            if ok
                return;
            end

            opts = detectImportOptions(filepath, 'VariableNamingRule', 'preserve');
            T = readtable(filepath, opts);
            readFcn = @(aliases, required, defaultValue) ...
                lts.correlation.CorrelationReplayProfile.readColumn( ...
                    T, aliases, required, defaultValue);
            obj = lts.correlation.CorrelationReplayProfile.fromColumnReader( ...
                filepath, readFcn);
        end

        function obj = fromColumnReader(filepath, readFcn)
            obj = lts.correlation.CorrelationReplayProfile( ...
                'SourceFile', filepath, ...
                'Time', readFcn({'time_s', 'time'}, true, NaN), ...
                'Distance', readFcn({'distance_m', 'distance', 's_m'}, false, NaN), ...
                'Throttle', readFcn({'throttle_ratio', 'throttle'}, true, NaN), ...
                'Brake', readFcn({'brake_ratio', 'brake'}, true, NaN), ...
                'BrakePressureFrontBar', readFcn( ...
                {'brake_pressure_front_bar', 'brake_pressure_front', ...
                'front_brake_pressure_bar', 'brakepressurefrontbar', ...
                'Brake Pressure Front'}, false, NaN), ...
                'BrakePressureRearBar', readFcn( ...
                {'brake_pressure_rear_bar', 'brake_pressure_rear', ...
                'rear_brake_pressure_bar', 'brakepressurerearbar', ...
                'Brake Pressure Rear'}, false, NaN), ...
                'RegenTorqueNm', readFcn( ...
                {'regen_torque_nm', 'regen_torque', 'regenTorqueNm', ...
                'Throttle Regen Negative Torque Command', ...
                'Throttle Regen Negative Torque C'}, false, NaN), ...
                'MotorTorqueCommandNm', readFcn( ...
                {'motor_torque_command_nm', 'motor_torque_command', ...
                'motorTorqueCommandNm', 'BAMOCAR Channels Calculated Cmd', ...
                'BAMOCAR Calculated Cmd', 'Calculated Cmd'}, false, NaN), ...
                'MotorTorqueDeliveredNm', readFcn( ...
                {'motor_torque_delivered_nm', 'motorTorqueDeliveredNm', ...
                'delivered_motor_torque_nm', 'BAMOCAR Delivered Torque'}, false, NaN), ...
                'MotorRpm', readFcn( ...
                {'motor_rpm', 'motorRpm', 'BAMOCAR Channels RPM', ...
                'BAMOCAR RPM', 'Motor RPM', 'Inverter Motor RPM'}, false, NaN), ...
                'PackVoltageV', readFcn( ...
                {'pack_voltage_v', 'pack_voltage', 'packVoltageV', ...
                'BMS Channels Pack Voltage', 'Battery Pack Voltage', ...
                'HV Pack Voltage', 'Pack Voltage'}, false, NaN), ...
                'PackCurrentA', readFcn( ...
                {'pack_current_a', 'pack_current', 'packCurrentA', ...
                'BMS Channels Pack Current', 'Battery Pack Current', ...
                'HV Pack Current', 'Pack Current'}, false, NaN), ...
                'Steer', readFcn({'steer_rad', 'steer'}, true, NaN), ...
                'Speed', readFcn({'speed_mps', 'speed'}, true, NaN), ...
                'WheelSpeedFL', readFcn( ...
                {'wheel_speed_fl_mps', 'wheel_speed_front_left_mps', ...
                'wheel_speed_fl', 'wheelSpeedFL', 'tireSpeed_FL', ...
                'Wheel Speed Front Left Sensor Linear'}, false, NaN), ...
                'WheelSpeedFR', readFcn( ...
                {'wheel_speed_fr_mps', 'wheel_speed_front_right_mps', ...
                'wheel_speed_fr', 'wheelSpeedFR', 'tireSpeed_FR', ...
                'Wheel Speed Front Right Sensor Linear'}, false, NaN), ...
                'WheelSpeedRL', readFcn( ...
                {'wheel_speed_rl_mps', 'wheel_speed_rear_left_mps', ...
                'wheel_speed_rl', 'wheelSpeedRL', 'tireSpeed_RL', ...
                'Wheel Speed Rear Left Sensor Linear'}, false, NaN), ...
                'WheelSpeedRR', readFcn( ...
                {'wheel_speed_rr_mps', 'wheel_speed_rear_right_mps', ...
                'wheel_speed_rr', 'wheelSpeedRR', 'tireSpeed_RR', ...
                'Wheel Speed Rear Right Sensor Linear'}, false, NaN), ...
                'Vx', readFcn({'vx_mps', 'vx', 'longitudinal_velocity_mps', 'body_vx_mps'}, false, NaN), ...
                'Vy', readFcn({'vy_mps', 'vy', 'lateral_velocity_mps', 'body_vy_mps'}, false, NaN), ...
                'BodySlip', readFcn({'body_slip_rad', 'sideslip_rad', 'body_slip', 'sideslip', 'beta'}, false, NaN), ...
                'Yaw', readFcn({'yaw_rad', 'yaw', 'heading_rad'}, false, NaN), ...
                'YawRate', readFcn({'yaw_rate_radps', 'yaw_rate'}, false, NaN), ...
                'X', readFcn({'x_m', 'x'}, false, NaN), ...
                'Y', readFcn({'y_m', 'y'}, false, NaN), ...
                'GpsLat', readFcn({'gps_lat_deg', 'gps_lat', 'latitude'}, false, NaN), ...
                'GpsLon', readFcn({'gps_lon_deg', 'gps_lon', 'longitude'}, false, NaN), ...
                'GpsCourse', readFcn({'gps_course_rad', 'gps_course', 'true_course_rad'}, false, NaN), ...
                'LatAccelG', readFcn({'lat_accel_g', 'lateral_accel_g'}, false, NaN), ...
                'FrontLatAccelG', readFcn( ...
                {'front_lat_accel_g', 'front_lateral_accel_g', ...
                'G Sensor Front Acceleration Lateral', ...
                'G Sensor Front Acceleration Late', ...
                'Front Axle Lat Accel Raw'}, false, NaN), ...
                'RearLatAccelG', readFcn( ...
                {'rear_lat_accel_g', 'rear_lateral_accel_g', ...
                'G Sensor Rear Acceleration Lateral', ...
                'G Sensor Rear Acceleration Later', ...
                'Rear Axle Lat Accel Raw'}, false, NaN), ...
                'FrontLongAccelG', readFcn( ...
                {'front_long_accel_g', 'front_longitudinal_accel_g', ...
                'G Sensor Front Acceleration Longitudinal', ...
                'G Sensor Front Acceleration Long', ...
                'G Sensor G Front Longitudinal', ...
                'Front Long Accel Raw'}, false, NaN), ...
                'RearLongAccelG', readFcn( ...
                {'rear_long_accel_g', 'rear_longitudinal_accel_g', ...
                'G Sensor Rear Acceleration Longitudinal', ...
                'G Sensor Rear Acceleration Long', ...
                'G Sensor Rear Acceleration Longi', ...
                'G Sensor G Rear Longitudinal', ...
                'Rear Long Accel Raw'}, false, NaN), ...
                'LongAccelG', readFcn({'long_accel_g', 'longitudinal_accel_g'}, false, NaN));
        end

        function [ok, obj] = tryReadNumericCsv(filepath)
            ok = false;
            obj = [];
            fid = fopen(filepath, 'r');
            if fid < 0
                return;
            end
            cleanup = onCleanup(@() fclose(fid));
            header = fgetl(fid);
            if ~ischar(header) && ~isstring(header)
                return;
            end
            names = strtrim(strsplit(char(header), ','));
            if isempty(names)
                return;
            end
            try
                data = readmatrix(filepath, 'NumHeaderLines', 1);
            catch err
                warning('lts_correlation_CorrelationReplayProfile:ReadMatrixFailed', ...
                    'Could not read companion matrix "%s" (%s); skipping.', ...
                    filepath, err.identifier);
                return;
            end
            if isempty(data) || size(data, 2) < numel(names)
                return;
            end
            normalizedNames = cellfun(@lts.correlation.CorrelationReplayProfile.normalizeName, ...
                names, 'UniformOutput', false);
            readFcn = @(aliases, required, defaultValue) ...
                lts.correlation.CorrelationReplayProfile.readNumericColumn( ...
                    data, normalizedNames, aliases, required, defaultValue);
            obj = lts.correlation.CorrelationReplayProfile.fromColumnReader( ...
                filepath, readFcn);
            ok = true;
        end

        function values = readColumn(T, aliases, required, defaultValue)
            names = T.Properties.VariableNames;
            normalizedNames = cellfun(@lts.correlation.CorrelationReplayProfile.normalizeName, ...
                names, 'UniformOutput', false);
            normalizedAliases = cellfun(@lts.correlation.CorrelationReplayProfile.normalizeName, ...
                aliases, 'UniformOutput', false);

            idx = find(ismember(normalizedNames, normalizedAliases), 1, 'first');
            if isempty(idx)
                if required
                    error('lts_correlation_CorrelationReplayProfile:MissingColumn', ...
                        'Replay CSV is missing required column "%s".', aliases{1});
                end
                values = repmat(defaultValue, height(T), 1);
                return;
            end

            values = T.(names{idx});
            values = double(values(:));
        end

        function values = readNumericColumn(data, normalizedNames, aliases, required, defaultValue)
            normalizedAliases = cellfun(@lts.correlation.CorrelationReplayProfile.normalizeName, ...
                aliases, 'UniformOutput', false);
            idx = find(ismember(normalizedNames, normalizedAliases), 1, 'first');
            if isempty(idx) || idx > size(data, 2)
                if required
                    error('lts_correlation_CorrelationReplayProfile:MissingColumn', ...
                        'Replay CSV is missing required column "%s".', aliases{1});
                end
                values = repmat(defaultValue, size(data, 1), 1);
                return;
            end

            values = double(data(:, idx));
        end

        function name = normalizeName(name)
            name = lower(char(name));
            name = regexprep(name, '[^a-z0-9]', '');
        end
    end
end
