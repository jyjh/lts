classdef TelemetryExporter
    % TELEMETRYEXPORTER Export simulation telemetry for external analysis.
    %
    % The MoTeC export is a CSV intended for MoTeC i2's CSV import workflow:
    % row 1 contains channel names, row 2 contains channel units, and the
    % remaining rows contain numeric samples. The first channel is Time [s].

    methods (Static)
        function filepath = writeToMoTeCFormat(stateLog, filepath, varargin)
            % WRITETOMOTECFORMAT Write stateLog to a MoTeC i2 import CSV.
            %   filepath = TelemetryExporter.writeToMoTeCFormat(stateLog, filepath)
            %   writes a channel-name row, unit row, and numeric data rows.
            %
            %   Optional name-value pairs:
            %     Channels       - cell array of stateLog field names to export
            %     IncludeDerived - true/false, include convenience channels

            if nargin < 2 || isempty(filepath)
                filepath = fullfile(pwd, 'motec_export.csv');
            end

            parser = inputParser;
            parser.addParameter('Channels', {}, @(x) iscell(x) || isstring(x) || ischar(x));
            parser.addParameter('IncludeDerived', true, @(x) islogical(x) || isnumeric(x));
            parser.parse(varargin{:});

            channels = cellstr(parser.Results.Channels);
            includeDerived = logical(parser.Results.IncludeDerived);

            TelemetryExporter.validateStateLog(stateLog);
            filepath = TelemetryExporter.ensureCsvPath(filepath);

            outDir = fileparts(filepath);
            if ~isempty(outDir) && ~exist(outDir, 'dir')
                mkdir(outDir);
            end

            tableData = TelemetryExporter.buildMoTeCTable(stateLog, channels, includeDerived);
            TelemetryExporter.writeCsv(filepath, tableData);

            fprintf('MoTeC CSV exported: %s\n', filepath);
        end
    end

    methods (Static, Access = private)
        function validateStateLog(stateLog)
            if ~isstruct(stateLog)
                error('TelemetryExporter:InvalidStateLog', ...
                    'stateLog must be a scalar struct of telemetry channel arrays.');
            end

            if ~isscalar(stateLog)
                error('TelemetryExporter:InvalidStateLog', ...
                    'stateLog must be a scalar struct, not a struct array.');
            end

            if ~isfield(stateLog, 'time') || isempty(stateLog.time)
                error('TelemetryExporter:MissingTime', ...
                    'stateLog must contain a non-empty time channel.');
            end
        end

        function filepath = ensureCsvPath(filepath)
            filepath = char(filepath);
            [folder, name, ext] = fileparts(filepath);
            if isempty(ext)
                ext = '.csv';
            end

            filepath = fullfile(folder, [name ext]);
        end

        function tableData = buildMoTeCTable(stateLog, requestedChannels, includeDerived)
            nSamples = numel(stateLog.time);
            tableData.names = {};
            tableData.units = {};
            tableData.values = zeros(nSamples, 0);

            if includeDerived
                tableData = TelemetryExporter.addDerivedChannels(tableData, stateLog, nSamples);
            else
                tableData = TelemetryExporter.addRawChannel(tableData, stateLog, 'time', 'Time', 's', nSamples);
            end

            if isempty(requestedChannels)
                requestedChannels = TelemetryExporter.defaultChannelOrder(stateLog);
            end

            for i = 1:numel(requestedChannels)
                field = requestedChannels{i};
                if ~isfield(stateLog, field)
                    warning('TelemetryExporter:MissingChannel', ...
                        'Skipping missing telemetry channel "%s".', field);
                    continue;
                end

                if TelemetryExporter.hasChannel(tableData, field)
                    continue;
                end

                [channelName, unit] = TelemetryExporter.rawChannelMetadata(field);
                tableData = TelemetryExporter.addRawChannel( ...
                    tableData, stateLog, field, channelName, unit, nSamples);
            end
        end

        function tableData = addDerivedChannels(tableData, stateLog, nSamples)
            tableData = TelemetryExporter.addRawChannel(tableData, stateLog, 'time', 'Time', 's', nSamples);
            tableData = TelemetryExporter.addRawChannel(tableData, stateLog, 's', 'Distance', 'm', nSamples);

            if isfield(stateLog, 'speedKmh')
                tableData = TelemetryExporter.addRawChannel(tableData, stateLog, 'speedKmh', 'Speed', 'km/h', nSamples);
            elseif isfield(stateLog, 'speed')
                tableData = TelemetryExporter.addComputedChannel(tableData, 'speed', 'Speed', ...
                    stateLog.speed(:) * 3.6, 'km/h', nSamples);
            end

            if isfield(stateLog, 'speed')
                tableData = TelemetryExporter.addRawChannel(tableData, stateLog, 'speed', 'Speed mps', 'm/s', nSamples);
            end

            if isfield(stateLog, 'ax')
                tableData = TelemetryExporter.addComputedChannel(tableData, 'axG', 'Long Accel', ...
                    stateLog.ax(:) / 9.81, 'G', nSamples);
            end

            if isfield(stateLog, 'ay')
                tableData = TelemetryExporter.addComputedChannel(tableData, 'ayG', 'Lat Accel', ...
                    stateLog.ay(:) / 9.81, 'G', nSamples);
            end

            pctChannels = { ...
                'throttle', 'Throttle'; ...
                'brake', 'Brake'; ...
                'brakeRequested', 'Brake Requested'; ...
                'slipRatio_FL', 'Slip Ratio FL'; ...
                'slipRatio_FR', 'Slip Ratio FR'; ...
                'slipRatio_RL', 'Slip Ratio RL'; ...
                'slipRatio_RR', 'Slip Ratio RR'};
            for i = 1:size(pctChannels, 1)
                field = pctChannels{i, 1};
                if isfield(stateLog, field)
                    tableData = TelemetryExporter.addComputedChannel(tableData, field, pctChannels{i, 2}, ...
                        stateLog.(field)(:) * 100, '%', nSamples);
                end
            end

            degChannels = { ...
                'steer', 'Steer'; ...
                'heading', 'Heading'; ...
                'pitchAngle', 'Pitch'; ...
                'camber_FL', 'Camber FL'; ...
                'camber_FR', 'Camber FR'; ...
                'camber_RL', 'Camber RL'; ...
                'camber_RR', 'Camber RR'; ...
                'toe_FL', 'Toe FL'; ...
                'toe_FR', 'Toe FR'; ...
                'toe_RL', 'Toe RL'; ...
                'toe_RR', 'Toe RR'; ...
                'wheelSteer_FL', 'Wheel Steer FL'; ...
                'wheelSteer_FR', 'Wheel Steer FR'; ...
                'wheelSteer_RL', 'Wheel Steer RL'; ...
                'wheelSteer_RR', 'Wheel Steer RR'};
            for i = 1:size(degChannels, 1)
                field = degChannels{i, 1};
                if isfield(stateLog, field)
                    tableData = TelemetryExporter.addComputedChannel(tableData, field, degChannels{i, 2}, ...
                        stateLog.(field)(:) * (180 / pi), 'deg', nSamples);
                end
            end

            mmChannels = { ...
                'damperPos_FL', 'Damper Pos FL'; ...
                'damperPos_FR', 'Damper Pos FR'; ...
                'damperPos_RL', 'Damper Pos RL'; ...
                'damperPos_RR', 'Damper Pos RR'; ...
                'wheelTravel_FL', 'Wheel Travel FL'; ...
                'wheelTravel_FR', 'Wheel Travel FR'; ...
                'wheelTravel_RL', 'Wheel Travel RL'; ...
                'wheelTravel_RR', 'Wheel Travel RR'};
            for i = 1:size(mmChannels, 1)
                field = mmChannels{i, 1};
                if isfield(stateLog, field)
                    tableData = TelemetryExporter.addComputedChannel(tableData, field, mmChannels{i, 2}, ...
                        stateLog.(field)(:) * 1000, 'mm', nSamples);
                end
            end

            mmPerSecChannels = { ...
                'damperVel_FL', 'Damper Vel FL'; ...
                'damperVel_FR', 'Damper Vel FR'; ...
                'damperVel_RL', 'Damper Vel RL'; ...
                'damperVel_RR', 'Damper Vel RR'};
            for i = 1:size(mmPerSecChannels, 1)
                field = mmPerSecChannels{i, 1};
                if isfield(stateLog, field)
                    tableData = TelemetryExporter.addComputedChannel(tableData, field, mmPerSecChannels{i, 2}, ...
                        stateLog.(field)(:) * 1000, 'mm/s', nSamples);
                end
            end

            omegaChannels = { ...
                'omega_FL', 'Wheel Speed FL'; ...
                'omega_FR', 'Wheel Speed FR'; ...
                'omega_RL', 'Wheel Speed RL'; ...
                'omega_RR', 'Wheel Speed RR'};
            for i = 1:size(omegaChannels, 1)
                field = omegaChannels{i, 1};
                if isfield(stateLog, field)
                    tableData = TelemetryExporter.addComputedChannel(tableData, [field 'RPM'], omegaChannels{i, 2}, ...
                        stateLog.(field)(:) * (60 / (2 * pi)), 'rpm', nSamples);
                end
            end
        end

        function tableData = addRawChannel(tableData, stateLog, field, channelName, unit, nSamples)
            values = stateLog.(field)(:);
            tableData = TelemetryExporter.addComputedChannel(tableData, field, channelName, values, unit, nSamples);
        end

        function tableData = addComputedChannel(tableData, field, channelName, values, unit, nSamples)
            if numel(values) ~= nSamples
                warning('TelemetryExporter:ChannelLengthMismatch', ...
                    'Skipping telemetry channel "%s" because it has %d samples, expected %d.', ...
                    field, numel(values), nSamples);
                return;
            end

            values = double(values(:));
            values(~isfinite(values)) = NaN;

            tableData.names{end + 1} = channelName;
            tableData.units{end + 1} = unit;
            tableData.values(:, end + 1) = values;
            tableData.sourceFields{numel(tableData.names)} = field;
        end

        function yes = hasChannel(tableData, field)
            yes = isfield(tableData, 'sourceFields') && any(strcmp(tableData.sourceFields, field));
        end

        function fields = defaultChannelOrder(stateLog)
            preferred = { ...
                'time', 's', 'speedKmh', 'speed', 'ax', 'ay', ...
                'throttle', 'brake', 'brakeRequested', 'steer', ...
                'curvature', 'heading', 'controlS', 'controlTime', ...
                'F_downforce', 'F_drag', 'F_drive', 'F_brake', 'F_tire_long', ...
                'F_brake_front', 'F_brake_rear', 'F_brake_FL', 'F_brake_FR', ...
                'F_brake_RL', 'F_brake_RR', 'brakeGrip_FL', 'brakeGrip_FR', ...
                'brakeGrip_RL', 'brakeGrip_RR', 'driveTorqueTotal', ...
                'driveTorque_RL', 'driveTorque_RR', 'brakeTorque_FL', ...
                'brakeTorque_FR', 'brakeTorque_RL', 'brakeTorque_RR', ...
                'motorRPM', 'motorTorque', 'wheelTorque', 'drivenWheelRPM', ...
                'rpmLimitActive', 'pitchAngle', 'Fz_FL', 'Fz_FR', 'Fz_RL', 'Fz_RR', ...
                'damperPos_FL', 'damperPos_FR', 'damperPos_RL', 'damperPos_RR', ...
                'damperVel_FL', 'damperVel_FR', 'damperVel_RL', 'damperVel_RR', ...
                'wheelTravel_FL', 'wheelTravel_FR', 'wheelTravel_RL', 'wheelTravel_RR', ...
                'camber_FL', 'camber_FR', 'camber_RL', 'camber_RR', ...
                'toe_FL', 'toe_FR', 'toe_RL', 'toe_RR', ...
                'wheelSteer_FL', 'wheelSteer_FR', 'wheelSteer_RL', 'wheelSteer_RR', ...
                'slipRatio_FL', 'slipRatio_FR', 'slipRatio_RL', 'slipRatio_RR', ...
                'omega_FL', 'omega_FR', 'omega_RL', 'omega_RR', ...
                'tireFx_FL', 'tireFx_FR', 'tireFx_RL', 'tireFx_RR', ...
                'tireFy_FL', 'tireFy_FR', 'tireFy_RL', 'tireFy_RR', ...
                'aeroFz_front', 'aeroFz_rear'};

            allFields = fieldnames(stateLog)';
            fields = preferred(ismember(preferred, allFields));
            remaining = setdiff(allFields, fields, 'stable');
            fields = [fields remaining];
        end

        function [name, unit] = rawChannelMetadata(field)
            switch field
                case 'time'
                    name = 'Time'; unit = 's';
                case 's'
                    name = 'Distance'; unit = 'm';
                case 'controlS'
                    name = 'Control Distance'; unit = 'm';
                case 'controlTime'
                    name = 'Control Time'; unit = 's';
                case 'speed'
                    name = 'Speed mps'; unit = 'm/s';
                case 'speedKmh'
                    name = 'Speed'; unit = 'km/h';
                case 'ax'
                    name = 'Long Accel Raw'; unit = 'm/s/s';
                case 'ay'
                    name = 'Lat Accel Raw'; unit = 'm/s/s';
                case 'throttle'
                    name = 'Throttle Raw'; unit = 'ratio';
                case 'brake'
                    name = 'Brake Raw'; unit = 'ratio';
                case 'brakeRequested'
                    name = 'Brake Requested Raw'; unit = 'ratio';
                case 'steer'
                    name = 'Steer Raw'; unit = 'rad';
                case 'heading'
                    name = 'Heading Raw'; unit = 'rad';
                case 'curvature'
                    name = 'Curvature'; unit = '1/m';
                case 'motorRPM'
                    name = 'Motor RPM'; unit = 'rpm';
                case 'drivenWheelRPM'
                    name = 'Driven Wheel RPM'; unit = 'rpm';
                case 'rpmLimitActive'
                    name = 'RPM Limit Active'; unit = 'bool';
                otherwise
                    name = TelemetryExporter.fieldToChannelName(field);
                    unit = TelemetryExporter.inferUnit(field);
            end
        end

        function unit = inferUnit(field)
            unit = '';

            if startsWith(field, 'F_') || startsWith(field, 'Fz_') || ...
                    startsWith(field, 'tireFx_') || startsWith(field, 'tireFy_') || ...
                    startsWith(field, 'aeroFz_') || startsWith(field, 'brakeGrip_')
                unit = 'N';
            elseif contains(field, 'Torque')
                unit = 'Nm';
            elseif startsWith(field, 'damperPos_') || startsWith(field, 'wheelTravel_')
                unit = 'm';
            elseif startsWith(field, 'damperVel_')
                unit = 'm/s';
            elseif startsWith(field, 'camber_') || startsWith(field, 'toe_') || ...
                    startsWith(field, 'wheelSteer_') || strcmp(field, 'pitchAngle')
                unit = 'rad';
            elseif startsWith(field, 'slipRatio_')
                unit = 'ratio';
            elseif startsWith(field, 'omega_')
                unit = 'rad/s';
            end
        end

        function name = fieldToChannelName(field)
            name = regexprep(field, '_', ' ');
            name = regexprep(name, '([a-z])([A-Z])', '$1 $2');
            name = strtrim(name);
            if isempty(name)
                return;
            end

            words = split(name);
            for i = 1:numel(words)
                word = char(words{i});
                if numel(word) > 2 && ~all(isstrprop(word, 'upper'))
                    words{i} = [upper(word(1)) lower(word(2:end))];
                else
                    words{i} = upper(word);
                end
            end
            name = strjoin(words, ' ');
        end

        function writeCsv(filepath, tableData)
            fid = fopen(filepath, 'w');
            if fid < 0
                error('TelemetryExporter:FileOpenFailed', ...
                    'Could not open "%s" for writing.', filepath);
            end
            cleanup = onCleanup(@() fclose(fid));

            fprintf(fid, '%s\n', TelemetryExporter.csvRow(tableData.names));
            fprintf(fid, '%s\n', TelemetryExporter.csvRow(tableData.units));

            for row = 1:size(tableData.values, 1)
                fprintf(fid, '%s\n', TelemetryExporter.numericCsvRow(tableData.values(row, :)));
            end
        end

        function row = numericCsvRow(values)
            cells = cell(1, numel(values));
            for i = 1:numel(values)
                if isnan(values(i))
                    cells{i} = '';
                else
                    cells{i} = sprintf('%.9g', values(i));
                end
            end
            row = strjoin(cells, ',');
        end

        function row = csvRow(values)
            cells = cell(1, numel(values));
            for i = 1:numel(values)
                cells{i} = TelemetryExporter.csvEscape(values{i});
            end
            row = strjoin(cells, ',');
        end

        function value = csvEscape(value)
            value = char(value);
            if contains(value, '"')
                value = strrep(value, '"', '""');
            end

            if contains(value, ',') || contains(value, '"') || contains(value, newline)
                value = ['"' value '"'];
            end
        end
    end
end
