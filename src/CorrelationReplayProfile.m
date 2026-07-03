classdef CorrelationReplayProfile
    % CORRELATIONREPLAYPROFILE Normalized real-lap controls for replay runs.
    %
    % The normalized CSV contract is intentionally small and stable:
    % time_s, distance_m, throttle_ratio, brake_ratio, steer_rad, speed_mps,
    % with optional yaw_rad, yaw_rate_radps, x_m, y_m, GPS/course, and
    % measured acceleration channels.

    properties
        sourceFile = ''
        time = []
        distance = []
        throttle = []
        brake = []
        steer = []
        speed = []
        yaw = []
        yawRate = []
        x = []
        y = []
        gpsLat = []
        gpsLon = []
        gpsCourse = []
        latAccelG = []
        longAccelG = []
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
            parser.addParameter('Steer', [], @isnumeric);
            parser.addParameter('Speed', [], @isnumeric);
            parser.addParameter('Yaw', [], @isnumeric);
            parser.addParameter('YawRate', [], @isnumeric);
            parser.addParameter('X', [], @isnumeric);
            parser.addParameter('Y', [], @isnumeric);
            parser.addParameter('GpsLat', [], @isnumeric);
            parser.addParameter('GpsLon', [], @isnumeric);
            parser.addParameter('GpsCourse', [], @isnumeric);
            parser.addParameter('LatAccelG', [], @isnumeric);
            parser.addParameter('LongAccelG', [], @isnumeric);
            parser.parse(varargin{:});

            obj.sourceFile = char(parser.Results.SourceFile);
            obj.time = parser.Results.Time(:);
            obj.distance = parser.Results.Distance(:);
            obj.throttle = parser.Results.Throttle(:);
            obj.brake = parser.Results.Brake(:);
            obj.steer = parser.Results.Steer(:);
            obj.speed = parser.Results.Speed(:);
            obj.yaw = parser.Results.Yaw(:);
            obj.yawRate = parser.Results.YawRate(:);
            obj.x = parser.Results.X(:);
            obj.y = parser.Results.Y(:);
            obj.gpsLat = parser.Results.GpsLat(:);
            obj.gpsLon = parser.Results.GpsLon(:);
            obj.gpsCourse = parser.Results.GpsCourse(:);
            obj.latAccelG = parser.Results.LatAccelG(:);
            obj.longAccelG = parser.Results.LongAccelG(:);
            obj = obj.validateAndComplete();
        end

        function input = sampleByTime(obj, time)
            input = obj.sampleAt(obj.time, time);
        end

        function input = sampleByDistance(obj, distance)
            if isempty(obj.distance) || all(~isfinite(obj.distance))
                error('CorrelationReplayProfile:MissingDistance', ...
                    'Distance-domain replay requires a distance_m channel or speed-derived distance.');
            end
            input = obj.sampleAt(obj.distance, distance);
        end

        function value = initialSpeed(obj)
            value = obj.speed(1);
        end

        function tf = hasYaw(obj)
            tf = ~isempty(obj.yaw) && isfinite(obj.yaw(1));
        end

        function tf = hasPosition(obj)
            tf = ~isempty(obj.x) && ~isempty(obj.y) && ...
                isfinite(obj.x(1)) && isfinite(obj.y(1));
        end

        function tf = hasGpsCourse(obj)
            tf = ~isempty(obj.gpsCourse) && any(isfinite(obj.gpsCourse));
        end

        function tf = hasLatAccel(obj)
            tf = ~isempty(obj.latAccelG) && any(isfinite(obj.latAccelG));
        end

        function duration = duration(obj)
            duration = obj.time(end) - obj.time(1);
        end

        function distance = totalDistance(obj)
            distance = obj.distance(end) - obj.distance(1);
        end
    end

    methods (Access = private)
        function obj = validateAndComplete(obj)
            required = {'time', 'throttle', 'brake', 'steer', 'speed'};
            for i = 1:numel(required)
                field = required{i};
                if isempty(obj.(field))
                    error('CorrelationReplayProfile:MissingField', ...
                        'Replay profile is missing required field "%s".', field);
                end
            end

            n = numel(obj.time);
            if n < 2
                error('CorrelationReplayProfile:TooShort', ...
                    'Replay profile must contain at least two samples.');
            end

            obj.time = obj.time - obj.time(1);
            obj.time = obj.requireColumnLength(obj.time, n, 'time');
            if any(~isfinite(obj.time)) || any(diff(obj.time) <= 0)
                error('CorrelationReplayProfile:InvalidTime', ...
                    'time_s must be finite and strictly increasing.');
            end

            obj.throttle = obj.clamp01(obj.requireColumnLength(obj.throttle, n, 'throttle'));
            obj.brake = obj.clamp01(obj.requireColumnLength(obj.brake, n, 'brake'));
            obj.steer = obj.requireColumnLength(obj.steer, n, 'steer');
            obj.speed = max(0, obj.requireColumnLength(obj.speed, n, 'speed'));

            if isempty(obj.distance) || all(~isfinite(obj.distance))
                obj.distance = obj.integrateDistanceFromSpeed();
            else
                obj.distance = obj.requireColumnLength(obj.distance, n, 'distance');
                obj.distance = obj.distance - obj.distance(1);
            end

            obj.yaw = obj.optionalColumn(obj.yaw, n, NaN, 'yaw');
            obj.yawRate = obj.optionalColumn(obj.yawRate, n, NaN, 'yawRate');
            obj.x = obj.optionalColumn(obj.x, n, NaN, 'x');
            obj.y = obj.optionalColumn(obj.y, n, NaN, 'y');
            obj.gpsLat = obj.optionalColumn(obj.gpsLat, n, NaN, 'gpsLat');
            obj.gpsLon = obj.optionalColumn(obj.gpsLon, n, NaN, 'gpsLon');
            obj.gpsCourse = obj.optionalColumn(obj.gpsCourse, n, NaN, 'gpsCourse');
            obj.latAccelG = obj.optionalColumn(obj.latAccelG, n, NaN, 'latAccelG');
            obj.longAccelG = obj.optionalColumn(obj.longAccelG, n, NaN, 'longAccelG');
        end

        function values = requireColumnLength(~, values, n, fieldName)
            values = double(values(:));
            if numel(values) ~= n
                error('CorrelationReplayProfile:LengthMismatch', ...
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
                error('CorrelationReplayProfile:LengthMismatch', ...
                    'Field "%s" has %d samples, expected %d.', ...
                    fieldName, numel(values), n);
            end
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

        function input = sampleAt(obj, axis, query)
            query = double(query);
            if ~isfinite(query)
                query = axis(1);
            end
            query = max(axis(1), min(axis(end), query));

            input = struct( ...
                'throttle', obj.interp(axis, obj.throttle, query), ...
                'brake', obj.interp(axis, obj.brake, query), ...
                'steer', obj.interp(axis, obj.steer, query), ...
                'targetSpeed', obj.interp(axis, obj.speed, query), ...
                'axRef', NaN, ...
                'sourceTime', obj.interp(axis, obj.time, query), ...
                'sourceDistance', obj.interp(axis, obj.distance, query));
        end

        function value = interp(~, axis, values, query)
            values = double(values(:));
            if all(~isfinite(values))
                value = NaN;
                return;
            end

            keep = isfinite(axis(:)) & isfinite(values);
            axis = axis(keep);
            values = values(keep);
            [axis, ia] = unique(axis, 'stable');
            values = values(ia);

            if isempty(axis)
                value = NaN;
            elseif numel(axis) == 1
                value = values(1);
            else
                query = max(axis(1), min(axis(end), query));
                value = interp1(axis, values, query, 'linear');
            end
        end
    end

    methods (Static)
        function obj = fromCsv(filepath)
            filepath = char(filepath);
            if ~exist(filepath, 'file')
                error('CorrelationReplayProfile:MissingCsv', ...
                    'Replay CSV "%s" does not exist.', filepath);
            end

            opts = detectImportOptions(filepath, 'VariableNamingRule', 'preserve');
            T = readtable(filepath, opts);

            obj = CorrelationReplayProfile( ...
                'SourceFile', filepath, ...
                'Time', CorrelationReplayProfile.readColumn(T, {'time_s', 'time'}, true, NaN), ...
                'Distance', CorrelationReplayProfile.readColumn(T, {'distance_m', 'distance', 's_m'}, false, NaN), ...
                'Throttle', CorrelationReplayProfile.readColumn(T, {'throttle_ratio', 'throttle'}, true, NaN), ...
                'Brake', CorrelationReplayProfile.readColumn(T, {'brake_ratio', 'brake'}, true, NaN), ...
                'Steer', CorrelationReplayProfile.readColumn(T, {'steer_rad', 'steer'}, true, NaN), ...
                'Speed', CorrelationReplayProfile.readColumn(T, {'speed_mps', 'speed'}, true, NaN), ...
                'Yaw', CorrelationReplayProfile.readColumn(T, {'yaw_rad', 'yaw', 'heading_rad'}, false, NaN), ...
                'YawRate', CorrelationReplayProfile.readColumn(T, {'yaw_rate_radps', 'yaw_rate'}, false, NaN), ...
                'X', CorrelationReplayProfile.readColumn(T, {'x_m', 'x'}, false, NaN), ...
                'Y', CorrelationReplayProfile.readColumn(T, {'y_m', 'y'}, false, NaN), ...
                'GpsLat', CorrelationReplayProfile.readColumn(T, {'gps_lat_deg', 'gps_lat', 'latitude'}, false, NaN), ...
                'GpsLon', CorrelationReplayProfile.readColumn(T, {'gps_lon_deg', 'gps_lon', 'longitude'}, false, NaN), ...
                'GpsCourse', CorrelationReplayProfile.readColumn(T, {'gps_course_rad', 'gps_course', 'true_course_rad'}, false, NaN), ...
                'LatAccelG', CorrelationReplayProfile.readColumn(T, {'lat_accel_g', 'lateral_accel_g'}, false, NaN), ...
                'LongAccelG', CorrelationReplayProfile.readColumn(T, {'long_accel_g', 'longitudinal_accel_g'}, false, NaN));
        end

        function values = readColumn(T, aliases, required, defaultValue)
            names = T.Properties.VariableNames;
            normalizedNames = cellfun(@CorrelationReplayProfile.normalizeName, ...
                names, 'UniformOutput', false);
            normalizedAliases = cellfun(@CorrelationReplayProfile.normalizeName, ...
                aliases, 'UniformOutput', false);

            idx = find(ismember(normalizedNames, normalizedAliases), 1, 'first');
            if isempty(idx)
                if required
                    error('CorrelationReplayProfile:MissingColumn', ...
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
