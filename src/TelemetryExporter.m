classdef TelemetryExporter
    % TELEMETRYEXPORTER Export simulation telemetry for external analysis.
    %
    % The MoTeC export is a MotecLogGenerator-compatible CSV:
    % row 1 contains channel names, and remaining rows contain numeric
    % samples. The first channel is Time [s].

    methods (Static)
        function filepath = writeToMoTeCFormat(stateLog, filepath, varargin)
            % WRITETOMOTECFORMAT Write stateLog to a MotecLogGenerator CSV.
            %   filepath = TelemetryExporter.writeToMoTeCFormat(stateLog, filepath)
            %   writes one channel-name row followed by numeric data rows.
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

        function [csvFile, ldFile] = exportToMoTeCLog(stateLog, csvFile, varargin)
            % EXPORTTOMOTECLOG Write CSV and convert it to a MoTeC .ld file.
            %   [csvFile, ldFile] = TelemetryExporter.exportToMoTeCLog(stateLog, csvFile)

            parser = inputParser;
            parser.KeepUnmatched = true;
            parser.addParameter('Channels', {}, @(x) iscell(x) || isstring(x) || ischar(x));
            parser.addParameter('IncludeDerived', true, @(x) islogical(x) || isnumeric(x));
            parser.parse(varargin{:});

            csvFile = TelemetryExporter.writeToMoTeCFormat( ...
                stateLog, csvFile, ...
                'Channels', parser.Results.Channels, ...
                'IncludeDerived', parser.Results.IncludeDerived);

            convertArgs = TelemetryExporter.nameValueStructToCell(parser.Unmatched);
            ldFile = TelemetryExporter.convertCsvToMoTeCLog(csvFile, convertArgs{:});
        end

        function ldFile = convertCsvToMoTeCLog(csvFile, varargin)
            % CONVERTCSVTOMOTECLOG Convert a MotecLogGenerator CSV to .ld.

            parser = inputParser;
            parser.addParameter('OutputFile', '', @(x) ischar(x) || isstring(x));
            parser.addParameter('Frequency', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x) && x > 0));
            parser.addParameter('PythonCommand', 'python', @(x) ischar(x) || isstring(x));
            parser.addParameter('GeneratorPath', '', @(x) ischar(x) || isstring(x));
            parser.addParameter('Driver', '', @(x) ischar(x) || isstring(x));
            parser.addParameter('VehicleId', '', @(x) ischar(x) || isstring(x));
            parser.addParameter('VehicleWeight', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x)));
            parser.addParameter('VehicleType', '', @(x) ischar(x) || isstring(x));
            parser.addParameter('VehicleComment', '', @(x) ischar(x) || isstring(x));
            parser.addParameter('VenueName', '', @(x) ischar(x) || isstring(x));
            parser.addParameter('EventName', '', @(x) ischar(x) || isstring(x));
            parser.addParameter('EventSession', '', @(x) ischar(x) || isstring(x));
            parser.addParameter('LongComment', '', @(x) ischar(x) || isstring(x));
            parser.addParameter('ShortComment', '', @(x) ischar(x) || isstring(x));
            parser.parse(varargin{:});

            csvFile = char(csvFile);
            if ~exist(csvFile, 'file')
                error('TelemetryExporter:MissingCsv', ...
                    'CSV file "%s" does not exist.', csvFile);
            end

            ldFile = char(parser.Results.OutputFile);
            if isempty(ldFile)
                [folder, name] = fileparts(csvFile);
                ldFile = fullfile(folder, [name '.ld']);
            end

            generatorPath = char(parser.Results.GeneratorPath);
            if isempty(generatorPath)
                generatorPath = TelemetryExporter.defaultGeneratorPath();
            end

            if ~exist(generatorPath, 'file')
                error('TelemetryExporter:MissingGenerator', ...
                    ['MoTeC log generator was not found at "%s". ' ...
                    'Run "git submodule update --init --recursive".'], generatorPath);
            end

            args = { ...
                TelemetryExporter.quoteShellArg(generatorPath), ...
                TelemetryExporter.quoteShellArg(csvFile), ...
                'CSV', ...
                '--output', TelemetryExporter.quoteShellArg(ldFile)};

            if ~isempty(parser.Results.Frequency)
                args(end + 1:end + 2) = {'--frequency', sprintf('%.9g', parser.Results.Frequency)};
            end

            args = TelemetryExporter.addOptionalCliArg(args, '--driver', parser.Results.Driver);
            args = TelemetryExporter.addOptionalCliArg(args, '--vehicle_id', parser.Results.VehicleId);
            if ~isempty(parser.Results.VehicleWeight)
                args(end + 1:end + 2) = {'--vehicle_weight', sprintf('%.0f', parser.Results.VehicleWeight)};
            end
            args = TelemetryExporter.addOptionalCliArg(args, '--vehicle_type', parser.Results.VehicleType);
            args = TelemetryExporter.addOptionalCliArg(args, '--vehicle_comment', parser.Results.VehicleComment);
            args = TelemetryExporter.addOptionalCliArg(args, '--venue_name', parser.Results.VenueName);
            args = TelemetryExporter.addOptionalCliArg(args, '--event_name', parser.Results.EventName);
            args = TelemetryExporter.addOptionalCliArg(args, '--event_session', parser.Results.EventSession);
            args = TelemetryExporter.addOptionalCliArg(args, '--long_comment', parser.Results.LongComment);
            args = TelemetryExporter.addOptionalCliArg(args, '--short_comment', parser.Results.ShortComment);

            command = strjoin([{char(parser.Results.PythonCommand)} args], ' ');
            [status, output] = system(command);
            if status ~= 0
                error('TelemetryExporter:MoTeCConversionFailed', ...
                    ['MoTeC log generator failed with status %d.\nCommand: %s\nOutput:\n%s\n' ...
                    'Install dependencies with: python -m pip install cantools numpy'], ...
                    status, command, output);
            end

            fprintf('MoTeC LD exported: %s\n', ldFile);
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
            tableData = TelemetryExporter.addFakeGpsChannels(tableData, stateLog, nSamples);

            if isfield(stateLog, 'speedKmh')
                tableData = TelemetryExporter.addRawChannel(tableData, stateLog, 'speedKmh', 'Vehicle Speed Value', 'km/h', nSamples);
            elseif isfield(stateLog, 'speed')
                tableData = TelemetryExporter.addComputedChannel(tableData, 'speed', 'Vehicle Speed Value', ...
                    stateLog.speed(:) * 3.6, 'km/h', nSamples);
            end

            if isfield(stateLog, 'speed')
                tableData = TelemetryExporter.addRawChannel(tableData, stateLog, 'speed', 'Speed mps', 'm/s', nSamples);
            end

            if isfield(stateLog, 'ax')
                tableData = TelemetryExporter.addComputedChannel(tableData, 'axG', 'G Sensor Front Acceleration Longitudinal', ...
                    stateLog.ax(:) / 9.81, 'G', nSamples);
            end

            if isfield(stateLog, 'ay')
                tableData = TelemetryExporter.addComputedChannel(tableData, 'ayG', 'G Sensor Front Acceleration Lateral', ...
                    stateLog.ay(:) / 9.81, 'G', nSamples);
            end

            pctChannels = { ...
                'throttle', 'Throttle Pedal'; ...
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
                'bodySlipAngle', 'Body Slip Angle'; ...
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
                'damperPos_FL', 'Damper Front Left Linear'; ...
                'damperPos_FR', 'Damper Front Right Linear'; ...
                'damperPos_RL', 'Damper Rear Left Linear'; ...
                'damperPos_RR', 'Damper Rear Right Linear'; ...
                'tireDeflection_FL', 'Tire Deflection FL'; ...
                'tireDeflection_FR', 'Tire Deflection FR'; ...
                'tireDeflection_RL', 'Tire Deflection RL'; ...
                'tireDeflection_RR', 'Tire Deflection RR'; ...
                'sprungPosition_FL', 'Sprung Position FL'; ...
                'sprungPosition_FR', 'Sprung Position FR'; ...
                'sprungPosition_RL', 'Sprung Position RL'; ...
                'sprungPosition_RR', 'Sprung Position RR'; ...
                'unsprungPosition_FL', 'Unsprung Position FL'; ...
                'unsprungPosition_FR', 'Unsprung Position FR'; ...
                'unsprungPosition_RL', 'Unsprung Position RL'; ...
                'unsprungPosition_RR', 'Unsprung Position RR'; ...
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
                'damperVel_RR', 'Damper Vel RR'; ...
                'sprungVelocity_FL', 'Sprung Vel FL'; ...
                'sprungVelocity_FR', 'Sprung Vel FR'; ...
                'sprungVelocity_RL', 'Sprung Vel RL'; ...
                'sprungVelocity_RR', 'Sprung Vel RR'; ...
                'unsprungVelocity_FL', 'Unsprung Vel FL'; ...
                'unsprungVelocity_FR', 'Unsprung Vel FR'; ...
                'unsprungVelocity_RL', 'Unsprung Vel RL'; ...
                'unsprungVelocity_RR', 'Unsprung Vel RR'};
            for i = 1:size(mmPerSecChannels, 1)
                field = mmPerSecChannels{i, 1};
                if isfield(stateLog, field)
                    tableData = TelemetryExporter.addComputedChannel(tableData, field, mmPerSecChannels{i, 2}, ...
                        stateLog.(field)(:) * 1000, 'mm/s', nSamples);
                end
            end

            omegaChannels = { ...
                'omega_FL', 'Wheel Speed Front Left Sensor Rotational'; ...
                'omega_FR', 'Wheel Speed Front Right Sensor Rotational'; ...
                'omega_RL', 'Wheel Speed Rear Left Sensor Rotational'; ...
                'omega_RR', 'Wheel Speed Rear Right Sensor Rotational'};
            for i = 1:size(omegaChannels, 1)
                field = omegaChannels{i, 1};
                if isfield(stateLog, field)
                    tableData = TelemetryExporter.addComputedChannel(tableData, [field 'RPM'], omegaChannels{i, 2}, ...
                        stateLog.(field)(:) * (60 / (2 * pi)), 'rpm', nSamples);
                end
            end
        end

        function tableData = addFakeGpsChannels(tableData, stateLog, nSamples)
            if ~isfield(stateLog, 's')
                return;
            end

            [latitude, longitude] = TelemetryExporter.fakeGpsFromPosition(stateLog, nSamples);
            tableData = TelemetryExporter.addComputedChannel(tableData, 'gpsLatitude', ...
                'GPS Latitude', latitude, 'deg', nSamples);
            tableData = TelemetryExporter.addComputedChannel(tableData, 'gpsLongitude', ...
                'GPS Longitude', longitude, 'deg', nSamples);
        end

        function [latitude, longitude] = fakeGpsFromPosition(stateLog, nSamples)
            originLatitude = 42.0;
            originLongitude = -83.0;
            earthRadiusM = 6378137.0;

            distance = double(stateLog.s(:));
            if numel(distance) ~= nSamples
                latitude = zeros(nSamples, 1);
                longitude = zeros(nSamples, 1);
                return;
            end

            distance(~isfinite(distance)) = 0;
            deltaDistance = [0; diff(distance)];
            deltaDistance(deltaDistance < 0) = 0;

            if isfield(stateLog, 'heading') && numel(stateLog.heading) == nSamples
                heading = double(stateLog.heading(:));
                heading(~isfinite(heading)) = 0;
            else
                heading = zeros(nSamples, 1);
            end

            eastMeters = cumsum(deltaDistance .* cos(heading));
            northMeters = cumsum(deltaDistance .* sin(heading));

            latitude = originLatitude + (northMeters / earthRadiusM) * (180 / pi);
            longitude = originLongitude + ...
                (eastMeters / (earthRadiusM * cos(originLatitude * pi / 180))) * (180 / pi);
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
            values(~isfinite(values)) = 0;

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
                'gpsLatitude', 'gpsLongitude', ...
                'throttle', 'brake', 'brakeRequested', 'steer', ...
                'curvature', 'heading', 'bodySlipAngle', 'controlS', 'controlTime', ...
                'F_downforce', 'F_drag', 'F_drive', 'F_brake', 'F_tire_long', ...
                'F_brake_front', 'F_brake_rear', 'F_brake_FL', 'F_brake_FR', ...
                'F_brake_RL', 'F_brake_RR', 'brakeGrip_FL', 'brakeGrip_FR', ...
                'brakeGrip_RL', 'brakeGrip_RR', 'driveTorqueTotal', ...
                'driveTorque_RL', 'driveTorque_RR', 'brakeTorque_FL', ...
                'brakeTorque_FR', 'brakeTorque_RL', 'brakeTorque_RR', ...
                'motorRPM', 'motorTorque', 'wheelTorque', 'drivenWheelRPM', ...
                'rpmLimitActive', 'pitchAngle', 'rideHeight', 'aeroDragHeight', ...
                'downforcePitchMoment', 'dragPitchMoment', 'aeroPitchMoment', ...
                'Fz_FL', 'Fz_FR', 'Fz_RL', 'Fz_RR', ...
                'suspensionForce_FL', 'suspensionForce_FR', 'suspensionForce_RL', 'suspensionForce_RR', ...
                'antiRollBarForce_FL', 'antiRollBarForce_FR', 'antiRollBarForce_RL', 'antiRollBarForce_RR', ...
                'suspensionDemand_FL', 'suspensionDemand_FR', 'suspensionDemand_RL', 'suspensionDemand_RR', ...
                'tireDeflection_FL', 'tireDeflection_FR', 'tireDeflection_RL', 'tireDeflection_RR', ...
                'damperPos_FL', 'damperPos_FR', 'damperPos_RL', 'damperPos_RR', ...
                'damperVel_FL', 'damperVel_FR', 'damperVel_RL', 'damperVel_RR', ...
                'sprungPosition_FL', 'sprungPosition_FR', 'sprungPosition_RL', 'sprungPosition_RR', ...
                'unsprungPosition_FL', 'unsprungPosition_FR', 'unsprungPosition_RL', 'unsprungPosition_RR', ...
                'sprungVelocity_FL', 'sprungVelocity_FR', 'sprungVelocity_RL', 'sprungVelocity_RR', ...
                'unsprungVelocity_FL', 'unsprungVelocity_FR', 'unsprungVelocity_RL', 'unsprungVelocity_RR', ...
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
                    name = 'Vehicle Speed Value'; unit = 'km/h';
                case 'gpsLatitude'
                    name = 'GPS Latitude'; unit = 'deg';
                case 'gpsLongitude'
                    name = 'GPS Longitude'; unit = 'deg';
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
                    name = 'Engine RPM'; unit = 'rpm';
                case 'motorTorque'
                    name = 'Cascadia Cascadia Calculated Torque'; unit = 'Nm';
                case 'F_brake'
                    name = 'Brake Total Force'; unit = 'N';
                case 'drivenWheelRPM'
                    name = 'Driven Wheel RPM'; unit = 'rpm';
                case 'rpmLimitActive'
                    name = 'RPM Limit Active'; unit = 'bool';
                case 'onTrack'
                    name = 'On Track'; unit = 'bool';
                case 'trackWidth'
                    name = 'Track Width'; unit = 'm';
                case 'trackLimitMargin'
                    name = 'Track Limit Margin'; unit = 'm';
                otherwise
                    name = TelemetryExporter.fieldToChannelName(field);
                    unit = TelemetryExporter.inferUnit(field);
            end
        end

        function unit = inferUnit(field)
            unit = '';

            if startsWith(field, 'F_') || startsWith(field, 'Fz_') || ...
                    startsWith(field, 'tireFx_') || startsWith(field, 'tireFy_') || ...
                    startsWith(field, 'aeroFz_') || startsWith(field, 'brakeGrip_') || ...
                    startsWith(field, 'suspensionForce_') || ...
                    startsWith(field, 'antiRollBarForce_') || ...
                    startsWith(field, 'suspensionDemand_')
                unit = 'N';
            elseif contains(field, 'Torque') || contains(field, 'Moment')
                unit = 'Nm';
            elseif startsWith(field, 'damperPos_') || startsWith(field, 'wheelTravel_') || ...
                    startsWith(field, 'tireDeflection_') || ...
                    startsWith(field, 'sprungPosition_') || ...
                    startsWith(field, 'unsprungPosition_') || ...
                    strcmp(field, 'rideHeight') || strcmp(field, 'aeroDragHeight')
                unit = 'm';
            elseif startsWith(field, 'damperVel_') || ...
                    startsWith(field, 'sprungVelocity_') || ...
                    startsWith(field, 'unsprungVelocity_')
                unit = 'm/s';
            elseif startsWith(field, 'camber_') || startsWith(field, 'toe_') || ...
                    startsWith(field, 'wheelSteer_') || strcmp(field, 'pitchAngle') || ...
                    strcmp(field, 'bodySlipAngle')
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

            fprintf(fid, '%s\n', TelemetryExporter.csvRow(TelemetryExporter.motecHeaderNames(tableData)));

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

        function names = motecHeaderNames(tableData)
            names = tableData.names;
            for i = 1:numel(names)
                unit = char(tableData.units{i});
                if ~isempty(unit)
                    names{i} = sprintf('%s (%s)', names{i}, unit);
                end
            end
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

        function generatorPath = defaultGeneratorPath()
            srcDir = fileparts(mfilename('fullpath'));
            repoRoot = fileparts(srcDir);
            generatorPath = fullfile(repoRoot, 'external', 'MotecLogGenerator', ...
                'motec_log_generator.py');
        end

        function args = addOptionalCliArg(args, flag, value)
            value = char(value);
            if isempty(value)
                return;
            end

            args(end + 1:end + 2) = {flag, TelemetryExporter.quoteShellArg(value)};
        end

        function value = quoteShellArg(value)
            value = char(value);
            value = strrep(value, '"', '\"');
            value = ['"' value '"'];
        end

        function values = nameValueStructToCell(valueStruct)
            fields = fieldnames(valueStruct);
            values = cell(1, numel(fields) * 2);

            outIdx = 1;
            for i = 1:numel(fields)
                values{outIdx} = fields{i};
                values{outIdx + 1} = valueStruct.(fields{i});
                outIdx = outIdx + 2;
            end
        end
    end
end
