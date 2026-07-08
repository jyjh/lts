classdef CorrelationReplayProfile
    % CORRELATIONREPLAYPROFILE Normalized real-lap controls for replay runs.
    %
    % The normalized CSV contract is intentionally small and stable:
    % time_s, distance_m, throttle_ratio, brake_ratio,
    % brake_pressure_front_bar, brake_pressure_rear_bar, regen_torque_nm,
    % motor_torque_command_nm, motor_rpm, pack_voltage_v, pack_current_a,
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
            parser.addParameter('Time', [], @isnumeric);
            parser.addParameter('Distance', [], @isnumeric);
            parser.addParameter('Throttle', [], @isnumeric);
            parser.addParameter('Brake', [], @isnumeric);
            parser.addParameter('BrakePressureFrontBar', [], @isnumeric);
            parser.addParameter('BrakePressureRearBar', [], @isnumeric);
            parser.addParameter('RegenTorqueNm', [], @isnumeric);
            parser.addParameter('MotorTorqueCommandNm', [], @isnumeric);
            parser.addParameter('MotorRpm', [], @isnumeric);
            parser.addParameter('PackVoltageV', [], @isnumeric);
            parser.addParameter('PackCurrentA', [], @isnumeric);
            parser.addParameter('Steer', [], @isnumeric);
            parser.addParameter('Speed', [], @isnumeric);
            parser.addParameter('WheelSpeedFL', [], @isnumeric);
            parser.addParameter('WheelSpeedFR', [], @isnumeric);
            parser.addParameter('WheelSpeedRL', [], @isnumeric);
            parser.addParameter('WheelSpeedRR', [], @isnumeric);
            parser.addParameter('Vx', [], @isnumeric);
            parser.addParameter('Vy', [], @isnumeric);
            parser.addParameter('BodySlip', [], @isnumeric);
            parser.addParameter('Yaw', [], @isnumeric);
            parser.addParameter('YawRate', [], @isnumeric);
            parser.addParameter('X', [], @isnumeric);
            parser.addParameter('Y', [], @isnumeric);
            parser.addParameter('GpsLat', [], @isnumeric);
            parser.addParameter('GpsLon', [], @isnumeric);
            parser.addParameter('GpsCourse', [], @isnumeric);
            parser.addParameter('LatAccelG', [], @isnumeric);
            parser.addParameter('FrontLatAccelG', [], @isnumeric);
            parser.addParameter('RearLatAccelG', [], @isnumeric);
            parser.addParameter('FrontLongAccelG', [], @isnumeric);
            parser.addParameter('RearLongAccelG', [], @isnumeric);
            parser.addParameter('LongAccelG', [], @isnumeric);
            parser.parse(varargin{:});

            obj.sourceFile = char(parser.Results.SourceFile);
            obj.time = parser.Results.Time(:);
            obj.distance = parser.Results.Distance(:);
            obj.throttle = parser.Results.Throttle(:);
            obj.brake = parser.Results.Brake(:);
            obj.brakePressureFrontBar = parser.Results.BrakePressureFrontBar(:);
            obj.brakePressureRearBar = parser.Results.BrakePressureRearBar(:);
            obj.regenTorqueNm = parser.Results.RegenTorqueNm(:);
            obj.motorTorqueCommandNm = parser.Results.MotorTorqueCommandNm(:);
            obj.motorRpm = parser.Results.MotorRpm(:);
            obj.packVoltageV = parser.Results.PackVoltageV(:);
            obj.packCurrentA = parser.Results.PackCurrentA(:);
            obj.steer = parser.Results.Steer(:);
            obj.speed = parser.Results.Speed(:);
            obj.wheelSpeedFL = parser.Results.WheelSpeedFL(:);
            obj.wheelSpeedFR = parser.Results.WheelSpeedFR(:);
            obj.wheelSpeedRL = parser.Results.WheelSpeedRL(:);
            obj.wheelSpeedRR = parser.Results.WheelSpeedRR(:);
            obj.vx = parser.Results.Vx(:);
            obj.vy = parser.Results.Vy(:);
            obj.bodySlip = parser.Results.BodySlip(:);
            obj.yaw = parser.Results.Yaw(:);
            obj.yawRate = parser.Results.YawRate(:);
            obj.x = parser.Results.X(:);
            obj.y = parser.Results.Y(:);
            obj.gpsLat = parser.Results.GpsLat(:);
            obj.gpsLon = parser.Results.GpsLon(:);
            obj.gpsCourse = parser.Results.GpsCourse(:);
            obj.latAccelG = parser.Results.LatAccelG(:);
            obj.frontLatAccelG = parser.Results.FrontLatAccelG(:);
            obj.rearLatAccelG = parser.Results.RearLatAccelG(:);
            obj.frontLongAccelG = parser.Results.FrontLongAccelG(:);
            obj.rearLongAccelG = parser.Results.RearLongAccelG(:);
            obj.longAccelG = parser.Results.LongAccelG(:);
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
            obj.brakePressureFrontBar = obj.optionalColumn( ...
                obj.brakePressureFrontBar, n, NaN, 'brakePressureFrontBar');
            obj.brakePressureRearBar = obj.optionalColumn( ...
                obj.brakePressureRearBar, n, NaN, 'brakePressureRearBar');
            obj.regenTorqueNm = obj.optionalColumn( ...
                obj.regenTorqueNm, n, NaN, 'regenTorqueNm');
            obj.motorTorqueCommandNm = obj.optionalColumn( ...
                obj.motorTorqueCommandNm, n, NaN, 'motorTorqueCommandNm');
            obj.motorRpm = obj.optionalColumn( ...
                obj.motorRpm, n, NaN, 'motorRpm');
            obj.packVoltageV = obj.optionalColumn( ...
                obj.packVoltageV, n, NaN, 'packVoltageV');
            obj.packCurrentA = obj.optionalColumn( ...
                obj.packCurrentA, n, NaN, 'packCurrentA');
            obj.steer = obj.requireColumnLength(obj.steer, n, 'steer');
            obj.speed = max(0, obj.requireColumnLength(obj.speed, n, 'speed'));
            obj.wheelSpeedFL = obj.nonnegativeOptionalColumn( ...
                obj.wheelSpeedFL, n, 'wheelSpeedFL');
            obj.wheelSpeedFR = obj.nonnegativeOptionalColumn( ...
                obj.wheelSpeedFR, n, 'wheelSpeedFR');
            obj.wheelSpeedRL = obj.nonnegativeOptionalColumn( ...
                obj.wheelSpeedRL, n, 'wheelSpeedRL');
            obj.wheelSpeedRR = obj.nonnegativeOptionalColumn( ...
                obj.wheelSpeedRR, n, 'wheelSpeedRR');

            if isempty(obj.distance) || all(~isfinite(obj.distance))
                obj.distance = obj.integrateDistanceFromSpeed();
            else
                obj.distance = obj.requireColumnLength(obj.distance, n, 'distance');
                obj.distance = obj.distance - obj.distance(1);
            end

            obj.vx = obj.optionalColumn(obj.vx, n, NaN, 'vx');
            obj.vy = obj.optionalColumn(obj.vy, n, NaN, 'vy');
            obj.bodySlip = obj.optionalColumn(obj.bodySlip, n, NaN, 'bodySlip');
            obj.yaw = obj.optionalColumn(obj.yaw, n, NaN, 'yaw');
            obj.yawRate = obj.optionalColumn(obj.yawRate, n, NaN, 'yawRate');
            obj.x = obj.optionalColumn(obj.x, n, NaN, 'x');
            obj.y = obj.optionalColumn(obj.y, n, NaN, 'y');
            obj.gpsLat = obj.optionalColumn(obj.gpsLat, n, NaN, 'gpsLat');
            obj.gpsLon = obj.optionalColumn(obj.gpsLon, n, NaN, 'gpsLon');
            obj.gpsCourse = obj.optionalColumn(obj.gpsCourse, n, NaN, 'gpsCourse');
            obj.latAccelG = obj.optionalColumn(obj.latAccelG, n, NaN, 'latAccelG');
            obj.frontLatAccelG = obj.optionalColumn(obj.frontLatAccelG, n, NaN, 'frontLatAccelG');
            obj.rearLatAccelG = obj.optionalColumn(obj.rearLatAccelG, n, NaN, 'rearLatAccelG');
            if all(~isfinite(obj.frontLatAccelG)) && any(isfinite(obj.latAccelG))
                obj.frontLatAccelG = obj.latAccelG;
            end
            obj.frontLongAccelG = obj.optionalColumn(obj.frontLongAccelG, n, NaN, 'frontLongAccelG');
            obj.rearLongAccelG = obj.optionalColumn(obj.rearLongAccelG, n, NaN, 'rearLongAccelG');
            obj.longAccelG = obj.optionalColumn(obj.longAccelG, n, NaN, 'longAccelG');
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
            values = max(0, min(1, values));
        end

        function input = sampleAt(obj, axis, cache, query)
            query = double(query);
            if ~isfinite(query)
                query = axis(1);
            end
            query = max(axis(1), min(axis(end), query));

            input = struct( ...
                'throttle', obj.lookup(cache.throttle, query), ...
                'brake', obj.lookup(cache.brake, query), ...
                'brakePressureFrontBar', obj.lookup(cache.brakePressureFrontBar, query), ...
                'brakePressureRearBar', obj.lookup(cache.brakePressureRearBar, query), ...
                'regenTorqueNm', obj.lookup(cache.regenTorqueNm, query), ...
                'motorTorqueCommandNm', obj.lookup(cache.motorTorqueCommandNm, query), ...
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
            cache = struct( ...
                'throttle', obj.buildInterpCache(axis, obj.throttle), ...
                'brake', obj.buildInterpCache(axis, obj.brake), ...
                'brakePressureFrontBar', obj.buildInterpCache(axis, obj.brakePressureFrontBar), ...
                'brakePressureRearBar', obj.buildInterpCache(axis, obj.brakePressureRearBar), ...
                'regenTorqueNm', obj.buildInterpCache(axis, obj.regenTorqueNm), ...
                'motorTorqueCommandNm', obj.buildInterpCache(axis, obj.motorTorqueCommandNm), ...
                'motorRpm', obj.buildInterpCache(axis, obj.motorRpm), ...
                'packVoltageV', obj.buildInterpCache(axis, obj.packVoltageV), ...
                'packCurrentA', obj.buildInterpCache(axis, obj.packCurrentA), ...
                'steer', obj.buildInterpCache(axis, obj.steer), ...
                'speed', obj.buildInterpCache(axis, obj.speed), ...
                'time', obj.buildInterpCache(axis, obj.time), ...
                'distance', obj.buildInterpCache(axis, obj.distance));
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
            [axis, ia] = unique(axis, 'stable');
            values = values(ia);

            if isempty(axis)
                cache = struct('axis', [], 'values', [], ...
                    'interpolant', [], 'isMissing', true, 'isScalar', false);
            elseif numel(axis) == 1
                cache = struct('axis', axis, 'values', values, ...
                    'interpolant', [], 'isMissing', false, 'isScalar', true);
            else
                [axis, order] = sort(axis);
                values = values(order);
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
    end

    methods (Static)
        function obj = fromCsv(filepath)
            filepath = char(filepath);
            if ~exist(filepath, 'file')
                error('lts_correlation_CorrelationReplayProfile:MissingCsv', ...
                    'Replay CSV "%s" does not exist.', filepath);
            end

            opts = detectImportOptions(filepath, 'VariableNamingRule', 'preserve');
            T = readtable(filepath, opts);

            obj = lts.correlation.CorrelationReplayProfile( ...
                'SourceFile', filepath, ...
                'Time', lts.correlation.CorrelationReplayProfile.readColumn(T, {'time_s', 'time'}, true, NaN), ...
                'Distance', lts.correlation.CorrelationReplayProfile.readColumn(T, {'distance_m', 'distance', 's_m'}, false, NaN), ...
                'Throttle', lts.correlation.CorrelationReplayProfile.readColumn(T, {'throttle_ratio', 'throttle'}, true, NaN), ...
                'Brake', lts.correlation.CorrelationReplayProfile.readColumn(T, {'brake_ratio', 'brake'}, true, NaN), ...
                'BrakePressureFrontBar', lts.correlation.CorrelationReplayProfile.readColumn(T, ...
                {'brake_pressure_front_bar', 'brake_pressure_front', ...
                'front_brake_pressure_bar', 'brakepressurefrontbar', ...
                'Brake Pressure Front'}, false, NaN), ...
                'BrakePressureRearBar', lts.correlation.CorrelationReplayProfile.readColumn(T, ...
                {'brake_pressure_rear_bar', 'brake_pressure_rear', ...
                'rear_brake_pressure_bar', 'brakepressurerearbar', ...
                'Brake Pressure Rear'}, false, NaN), ...
                'RegenTorqueNm', lts.correlation.CorrelationReplayProfile.readColumn(T, ...
                {'regen_torque_nm', 'regen_torque', 'regenTorqueNm', ...
                'Throttle Regen Negative Torque Command', ...
                'Throttle Regen Negative Torque C'}, false, NaN), ...
                'MotorTorqueCommandNm', lts.correlation.CorrelationReplayProfile.readColumn(T, ...
                {'motor_torque_command_nm', 'motor_torque_command', ...
                'motorTorqueCommandNm', 'BAMOCAR Channels Calculated Cmd', ...
                'BAMOCAR Calculated Cmd', 'Calculated Cmd'}, false, NaN), ...
                'MotorRpm', lts.correlation.CorrelationReplayProfile.readColumn(T, ...
                {'motor_rpm', 'motorRpm', 'BAMOCAR Channels RPM', ...
                'BAMOCAR RPM', 'Motor RPM', 'Inverter Motor RPM'}, false, NaN), ...
                'PackVoltageV', lts.correlation.CorrelationReplayProfile.readColumn(T, ...
                {'pack_voltage_v', 'pack_voltage', 'packVoltageV', ...
                'BMS Channels Pack Voltage', 'Battery Pack Voltage', ...
                'HV Pack Voltage', 'Pack Voltage'}, false, NaN), ...
                'PackCurrentA', lts.correlation.CorrelationReplayProfile.readColumn(T, ...
                {'pack_current_a', 'pack_current', 'packCurrentA', ...
                'BMS Channels Pack Current', 'Battery Pack Current', ...
                'HV Pack Current', 'Pack Current'}, false, NaN), ...
                'Steer', lts.correlation.CorrelationReplayProfile.readColumn(T, {'steer_rad', 'steer'}, true, NaN), ...
                'Speed', lts.correlation.CorrelationReplayProfile.readColumn(T, {'speed_mps', 'speed'}, true, NaN), ...
                'WheelSpeedFL', lts.correlation.CorrelationReplayProfile.readColumn(T, ...
                {'wheel_speed_fl_mps', 'wheel_speed_front_left_mps', ...
                'wheel_speed_fl', 'wheelSpeedFL', 'tireSpeed_FL', ...
                'Wheel Speed Front Left Sensor Linear'}, false, NaN), ...
                'WheelSpeedFR', lts.correlation.CorrelationReplayProfile.readColumn(T, ...
                {'wheel_speed_fr_mps', 'wheel_speed_front_right_mps', ...
                'wheel_speed_fr', 'wheelSpeedFR', 'tireSpeed_FR', ...
                'Wheel Speed Front Right Sensor Linear'}, false, NaN), ...
                'WheelSpeedRL', lts.correlation.CorrelationReplayProfile.readColumn(T, ...
                {'wheel_speed_rl_mps', 'wheel_speed_rear_left_mps', ...
                'wheel_speed_rl', 'wheelSpeedRL', 'tireSpeed_RL', ...
                'Wheel Speed Rear Left Sensor Linear'}, false, NaN), ...
                'WheelSpeedRR', lts.correlation.CorrelationReplayProfile.readColumn(T, ...
                {'wheel_speed_rr_mps', 'wheel_speed_rear_right_mps', ...
                'wheel_speed_rr', 'wheelSpeedRR', 'tireSpeed_RR', ...
                'Wheel Speed Rear Right Sensor Linear'}, false, NaN), ...
                'Vx', lts.correlation.CorrelationReplayProfile.readColumn(T, {'vx_mps', 'vx', 'longitudinal_velocity_mps', 'body_vx_mps'}, false, NaN), ...
                'Vy', lts.correlation.CorrelationReplayProfile.readColumn(T, {'vy_mps', 'vy', 'lateral_velocity_mps', 'body_vy_mps'}, false, NaN), ...
                'BodySlip', lts.correlation.CorrelationReplayProfile.readColumn(T, {'body_slip_rad', 'sideslip_rad', 'body_slip', 'sideslip', 'beta'}, false, NaN), ...
                'Yaw', lts.correlation.CorrelationReplayProfile.readColumn(T, {'yaw_rad', 'yaw', 'heading_rad'}, false, NaN), ...
                'YawRate', lts.correlation.CorrelationReplayProfile.readColumn(T, {'yaw_rate_radps', 'yaw_rate'}, false, NaN), ...
                'X', lts.correlation.CorrelationReplayProfile.readColumn(T, {'x_m', 'x'}, false, NaN), ...
                'Y', lts.correlation.CorrelationReplayProfile.readColumn(T, {'y_m', 'y'}, false, NaN), ...
                'GpsLat', lts.correlation.CorrelationReplayProfile.readColumn(T, {'gps_lat_deg', 'gps_lat', 'latitude'}, false, NaN), ...
                'GpsLon', lts.correlation.CorrelationReplayProfile.readColumn(T, {'gps_lon_deg', 'gps_lon', 'longitude'}, false, NaN), ...
                'GpsCourse', lts.correlation.CorrelationReplayProfile.readColumn(T, {'gps_course_rad', 'gps_course', 'true_course_rad'}, false, NaN), ...
                'LatAccelG', lts.correlation.CorrelationReplayProfile.readColumn(T, {'lat_accel_g', 'lateral_accel_g'}, false, NaN), ...
                'FrontLatAccelG', lts.correlation.CorrelationReplayProfile.readColumn(T, ...
                {'front_lat_accel_g', 'front_lateral_accel_g', ...
                'G Sensor Front Acceleration Lateral', ...
                'G Sensor Front Acceleration Late', ...
                'Front Axle Lat Accel Raw'}, false, NaN), ...
                'RearLatAccelG', lts.correlation.CorrelationReplayProfile.readColumn(T, ...
                {'rear_lat_accel_g', 'rear_lateral_accel_g', ...
                'G Sensor Rear Acceleration Lateral', ...
                'G Sensor Rear Acceleration Later', ...
                'Rear Axle Lat Accel Raw'}, false, NaN), ...
                'FrontLongAccelG', lts.correlation.CorrelationReplayProfile.readColumn(T, ...
                {'front_long_accel_g', 'front_longitudinal_accel_g', ...
                'G Sensor Front Acceleration Longitudinal', ...
                'G Sensor Front Acceleration Long', ...
                'G Sensor G Front Longitudinal', ...
                'Front Long Accel Raw'}, false, NaN), ...
                'RearLongAccelG', lts.correlation.CorrelationReplayProfile.readColumn(T, ...
                {'rear_long_accel_g', 'rear_longitudinal_accel_g', ...
                'G Sensor Rear Acceleration Longitudinal', ...
                'G Sensor Rear Acceleration Long', ...
                'G Sensor Rear Acceleration Longi', ...
                'G Sensor G Rear Longitudinal', ...
                'Rear Long Accel Raw'}, false, NaN), ...
                'LongAccelG', lts.correlation.CorrelationReplayProfile.readColumn(T, {'long_accel_g', 'longitudinal_accel_g'}, false, NaN));
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

        function name = normalizeName(name)
            name = lower(char(name));
            name = regexprep(name, '[^a-z0-9]', '');
        end
    end
end
