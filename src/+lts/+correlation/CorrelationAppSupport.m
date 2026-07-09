classdef CorrelationAppSupport
    % CORRELATIONAPPSUPPORT Loading, tuning, extraction, and preflight helpers.

    methods (Static)
        function mode = validateBrakeMode(mode)
            mode = lower(string(mode));
            if mode ~= "ratio" && mode ~= "pressure"
                error('run_correlation:InvalidBrakeMode', ...
                    'BrakeMode must be "ratio" or "pressure".');
            end
        end

        function mode = validatePowertrainMode(mode)
            mode = lower(string(mode));
            mode = strrep(mode, "-", "_");
            if mode == "motor_torque" || mode == "motor_command" || ...
                    mode == "calculated_cmd" || mode == "calculated_command"
                mode = "motor_torque_command";
            end
            if mode ~= "throttle" && mode ~= "motor_torque_command"
                error('run_correlation:InvalidPowertrainMode', ...
                    'PowertrainMode must be "throttle" or "motor_torque_command".');
            end
        end

        function extractMoTeCLap(opts, replayCsv, manifestFile, repoRoot)
            script = fullfile(repoRoot, 'scripts', 'extract_motec_lap.py');
            args = { ...
                char(opts.PythonCommand), script, ...
                '--input', char(opts.MoTeCFile), ...
                '--output', replayCsv, ...
                '--manifest', manifestFile, ...
                '--channel-map', char(opts.ChannelMap)};

            if ~isempty(opts.Lap)
                args(end+1:end+2) = { ...
                    '--laps', lts.correlation.CorrelationAppSupport.lapValue(opts.Lap)}; %#ok<AGROW>
            end
            if ~isempty(opts.LdxFile)
                args(end+1:end+2) = {'--ldx', char(opts.LdxFile)}; %#ok<AGROW>
            end
            if ~isempty(opts.ImportFrequency)
                args(end+1:end+2) = {'--frequency', sprintf('%.9g', opts.ImportFrequency)}; %#ok<AGROW>
            end

            command = lts.util.shellJoin(args);
            [status, output] = system(command);
            if status ~= 0
                error('run_correlation:ExtractionFailed', ...
                    'MoTeC extraction failed with status %d.\nCommand: %s\nOutput:\n%s', ...
                    status, command, output);
            end
            fprintf('%s\n', strtrim(output));
        end

        function value = lapValue(lap)
            if isnumeric(lap)
                if isscalar(lap)
                    value = sprintf('%d', lap);
                elseif numel(lap) == 2
                    value = sprintf('%d-%d', lap(1), lap(2));
                else
                    error('run_correlation:InvalidLap', ...
                        'Lap must be scalar or a two-element inclusive range.');
                end
            else
                value = char(lap);
            end
        end

        function base = buildOutputBase(opts, config, repoRoot)
            if ~isempty(opts.OutputBase)
                base = char(opts.OutputBase);
                return;
            end

            if ~isempty(opts.MoTeCFile)
                [~, name] = fileparts(char(opts.MoTeCFile));
            elseif ~isempty(opts.ReplayCsv)
                [~, name] = fileparts(char(opts.ReplayCsv));
            else
                name = 'replay';
            end

            lapSuffix = '';
            if ~isempty(opts.Lap)
                lapSuffix = ['_lap' regexprep( ...
                    lts.correlation.CorrelationAppSupport.lapValue(opts.Lap), ...
                    '[^A-Za-z0-9]', '_')];
            end

            exportDir = fullfile(repoRoot, 'exports');
            base = fullfile(exportDir, sprintf('correlation_%s%s_%s_%s', ...
                name, lapSuffix, char(config.name), datestr(now, 'yyyymmdd_HHMMSS')));
        end

        function track = loadTrack(trackSpec, repoRoot)
            if isa(trackSpec, 'lts.components.Track')
                track = trackSpec;
                return;
            end

            trackText = char(trackSpec);
            if strcmpi(trackText, '2026enduro')
                track = lts.components.WaypointTrack.loadMat( ...
                    fullfile(repoRoot, 'tracks', ...
                    'endurance_track_grid_25ft_from_matlab_smoothed.mat'));
                track.Width = 5.0;
            elseif endsWith(lower(trackText), '.mat')
                track = lts.components.WaypointTrack.loadMat(trackText);
            elseif endsWith(lower(trackText), '.csv')
                track = lts.components.WaypointTrack.fromCsv(trackText);
            else
                track = lts.components.TestTrack(trackText);
            end
        end

        function config = loadVehicleConfig(configSpec)
            if isa(configSpec, 'function_handle')
                config = configSpec();
            elseif isa(configSpec, 'lts.vehicle.VehicleConfig')
                config = configSpec;
            elseif ischar(configSpec) || isstring(configSpec)
                name = char(configSpec);
                if contains(name, '.')
                    fn = str2func(name);
                else
                    fn = str2func(['lts.vehicles.' name]);
                end
                config = fn();
            else
                config = configSpec;
            end
        end

        function config = applyVehicleTuning(config, vehicleTuning, tuningFile)
            hasVehicleTuning = ~isempty(vehicleTuning);
            hasTuningFile = ~isempty(tuningFile);
            if hasVehicleTuning && hasTuningFile
                error('run_correlation:DuplicateTuning', ...
                    'Use either VehicleTuning or TuningFile, not both.');
            end
            if ~hasVehicleTuning && ~hasTuningFile
                return;
            end

            tuningSpec = vehicleTuning;
            if hasTuningFile
                tuningSpec = tuningFile;
            end

            if isa(tuningSpec, 'lts.vehicle.VehicleConfig')
                config = tuningSpec;
            elseif isstruct(tuningSpec)
                config = lts.correlation.CorrelationAppSupport.applyVehicleOverrideStruct( ...
                    config, tuningSpec);
            elseif isa(tuningSpec, 'function_handle')
                config = lts.correlation.CorrelationAppSupport.applyVehicleTuningFunction( ...
                    config, tuningSpec);
            elseif ischar(tuningSpec) || isstring(tuningSpec)
                config = lts.correlation.CorrelationAppSupport.applyVehicleTuningFunction( ...
                    config, lts.correlation.CorrelationAppSupport.tuningFunctionFromName(tuningSpec));
            else
                error('run_correlation:InvalidTuning', ...
                    'Vehicle tuning must be a function, config, struct, or tuning file name.');
            end
        end

        function preflight(profile, track, vehicle, surfaceMu, manifestFile, brakeMode, powertrainMode, limitMotorTorqueByPackPower, packPowerAdvanceS, motorTorqueCommandDelayS)
            if nargin < 7 || isempty(powertrainMode)
                powertrainMode = "throttle";
            end
            if nargin < 8 || isempty(limitMotorTorqueByPackPower)
                limitMotorTorqueByPackPower = false;
            end
            if nargin < 9 || isempty(packPowerAdvanceS)
                packPowerAdvanceS = 0;
            end
            if nargin < 10 || isempty(motorTorqueCommandDelayS)
                motorTorqueCommandDelayS = 0;
            end
            powertrainMode = lts.correlation.CorrelationAppSupport.validatePowertrainMode(powertrainMode);
            packCapText = 'disabled';
            if logical(limitMotorTorqueByPackPower)
                packCapText = 'enabled';
            end
            fprintf('\n=== Correlation Preflight ===\n');
            lts.correlation.CorrelationAppSupport.printExtractionSummary(manifestFile);
            fprintf('Reference mode: free-space replay\n');
            fprintf('Surface mu: %.3f\n', surfaceMu);
            fprintf('Brake mode: %s\n', char(brakeMode));
            fprintf('Powertrain mode: %s\n', char(powertrainMode));
            fprintf('Pack power torque cap: %s\n', packCapText);
            fprintf('Pack power channel advance: %.3f s\n', double(packPowerAdvanceS));
            fprintf('Motor torque command delay: %.3f s\n', double(motorTorqueCommandDelayS));
            lts.correlation.CorrelationAppSupport.printReplayRanges(profile);
            lts.correlation.CorrelationAppSupport.validatePressureBrakeMode(profile, vehicle, brakeMode);
            lts.correlation.CorrelationAppSupport.validatePowertrainCommandMode( ...
                profile, powertrainMode, limitMotorTorqueByPackPower);
            lts.correlation.CorrelationAppSupport.warnOnSteeringScale(profile, vehicle, 120);
            lts.correlation.CorrelationAppSupport.warnOnLateralAccelConsistency(profile, vehicle);
            lts.correlation.CorrelationAppSupport.warnOnBrakeScale(profile);

            if nargin >= 2 && ~isempty(track)
                fprintf('Environment track length: %.2f m (not used for path projection)\n', ...
                    track.getTotalLength());
            end
        end

        function name = trackName(track)
            name = 'track';
            if isprop(track, 'Name') && ~isempty(track.Name)
                name = track.Name;
            end
        end

        function config = loadCorrelationConfig(configFile)
            config = struct();
            if nargin < 1 || isempty(configFile)
                return;
            end

            configFile = char(configFile);
            if strlength(string(configFile)) == 0
                return;
            end
            if ~exist(configFile, 'file')
                error('run_correlation:MissingCorrelationConfig', ...
                    'Correlation config "%s" does not exist.', configFile);
            end

            try
                config = jsondecode(fileread(configFile));
            catch err
                error('run_correlation:InvalidCorrelationConfig', ...
                    'Could not read correlation config "%s": %s', ...
                    configFile, err.message);
            end
            if ~isstruct(config)
                error('run_correlation:InvalidCorrelationConfig', ...
                    'Correlation config "%s" must decode to a JSON object.', configFile);
            end
        end

        function value = correlationConfigScalar(config, fieldName, defaultValue)
            value = defaultValue;
            if nargin < 1 || isempty(config) || ~isstruct(config)
                return;
            end

            containers = {config};
            nestedNames = {'offsets', 'runCorrelationOptions', ...
                'plotCorrelationPositionOverlayOptions'};
            for i = 1:numel(nestedNames)
                nested = lts.correlation.CorrelationAppSupport.structField( ...
                    config, nestedNames{i});
                if isstruct(nested)
                    containers{end+1} = nested; %#ok<AGROW>
                end
            end

            for i = 1:numel(containers)
                candidate = lts.correlation.CorrelationAppSupport.structField( ...
                    containers{i}, fieldName);
                if isnumeric(candidate) && isscalar(candidate) && isfinite(candidate)
                    value = double(candidate);
                    return;
                end
            end
        end

        function mu = vehicleCorrelationSurfaceMu(config)
            mu = NaN;
            correlation = lts.correlation.CorrelationAppSupport.vehicleCorrelationStruct(config);
            if isempty(correlation) || ~isfield(correlation, 'surfaceMu')
                return;
            end

            candidate = correlation.surfaceMu;
            if isnumeric(candidate) && isscalar(candidate) && ...
                    isfinite(candidate) && candidate > 0
                mu = double(candidate);
            end
        end

        function value = vehicleCorrelationFlag(config, fieldName, defaultValue)
            value = logical(defaultValue);
            correlation = lts.correlation.CorrelationAppSupport.vehicleCorrelationStruct(config);
            if isempty(correlation) || ~isfield(correlation, fieldName)
                return;
            end

            candidate = correlation.(fieldName);
            if islogical(candidate) || isnumeric(candidate)
                value = logical(candidate);
            end
        end

        function value = vehicleCorrelationScalar(config, fieldName, defaultValue)
            value = double(defaultValue);
            correlation = lts.correlation.CorrelationAppSupport.vehicleCorrelationStruct(config);
            if isempty(correlation) || ~isfield(correlation, fieldName)
                return;
            end

            candidate = correlation.(fieldName);
            if isnumeric(candidate) && isscalar(candidate) && isfinite(candidate)
                value = double(candidate);
            end
        end

        function correlation = vehicleCorrelationStruct(config)
            correlation = [];
            if isempty(config)
                return;
            end

            correlation = [];
            if isobject(config) && isprop(config, 'correlation')
                correlation = config.correlation;
            elseif isstruct(config) && isfield(config, 'correlation')
                correlation = config.correlation;
            end
            if ~isstruct(correlation)
                correlation = [];
            end
        end
    end

    methods (Static, Access=private)
        function printExtractionSummary(manifestFile)
            if isempty(manifestFile) || ~exist(manifestFile, 'file')
                return;
            end

            try
                manifest = jsondecode(fileread(manifestFile));
            catch err
                warning('run_correlation:ManifestReadFailed', ...
                    'Could not read extraction manifest "%s": %s', manifestFile, err.message);
                return;
            end

            if ~isfield(manifest, 'channels')
                return;
            end

            names = {'throttle_ratio', 'brake_ratio', ...
                'brake_pressure_front_bar', 'brake_pressure_rear_bar', ...
                'regen_torque_nm', 'motor_torque_command_nm', ...
                'motor_rpm', 'pack_voltage_v', 'pack_current_a', ...
                'steer_rad', 'speed_mps', 'distance_m', 'yaw_rad', ...
                'yaw_rate_radps', 'vx_mps', 'vy_mps', 'body_slip_rad'};
            for i = 1:numel(names)
                name = names{i};
                if ~isfield(manifest.channels, name)
                    continue;
                end
                channel = manifest.channels.(name);
                if ~isstruct(channel)
                    fprintf('%-16s: missing\n', name);
                    continue;
                end
                sourceName = lts.correlation.CorrelationAppSupport.jsonField(channel, 'name', '');
                sourceLabel = lts.correlation.CorrelationAppSupport.jsonField(channel, 'source_label', '');
                scale = lts.correlation.CorrelationAppSupport.jsonField(channel, 'scale_applied', NaN);
                minValue = lts.correlation.CorrelationAppSupport.jsonField(channel, 'min_value', NaN);
                maxValue = lts.correlation.CorrelationAppSupport.jsonField(channel, 'max_value', NaN);
                if isempty(sourceLabel)
                    fprintf('%-16s: %s, scale %.6g, range [%.4g, %.4g]\n', ...
                        name, sourceName, scale, minValue, maxValue);
                else
                    fprintf('%-16s: %s (%s), scale %.6g, range [%.4g, %.4g]\n', ...
                        name, sourceName, sourceLabel, scale, minValue, maxValue);
                end
            end
        end

        function printReplayRanges(profile)
            fprintf('Initial speed: %.2f m/s\n', profile.speed(1));
            fprintf('Initial steer: %.2f deg\n', profile.steer(1) * 180 / pi);
            fprintf('Throttle range: %.3f to %.3f\n', min(profile.throttle), max(profile.throttle));
            fprintf('Steer range: %.2f to %.2f deg\n', ...
                min(profile.steer) * 180 / pi, max(profile.steer) * 180 / pi);
            fprintf('Brake range: %.3f to %.3f\n', min(profile.brake), max(profile.brake));
            if profile.hasBrakePressure()
                fprintf('Brake pressure front range: %.3f to %.3f bar\n', ...
                    lts.util.minFinite(profile.brakePressureFrontBar), ...
                    lts.util.maxFinite(profile.brakePressureFrontBar));
                fprintf('Brake pressure rear range: %.3f to %.3f bar\n', ...
                    lts.util.minFinite(profile.brakePressureRearBar), ...
                    lts.util.maxFinite(profile.brakePressureRearBar));
            end
            if profile.hasRegenTorque()
                fprintf('Regen torque range: %.3f to %.3f Nm\n', ...
                    lts.util.minFinite(profile.regenTorqueNm), ...
                    lts.util.maxFinite(profile.regenTorqueNm));
            end
            if profile.hasMotorTorqueCommand()
                fprintf('Motor torque command range: %.3f to %.3f Nm\n', ...
                    lts.util.minFinite(profile.motorTorqueCommandNm), ...
                    lts.util.maxFinite(profile.motorTorqueCommandNm));
            end
            if profile.hasMotorRpm()
                fprintf('Motor RPM range: %.3f to %.3f rpm\n', ...
                    lts.util.minFinite(profile.motorRpm), ...
                    lts.util.maxFinite(profile.motorRpm));
            end
            if profile.hasPackPower()
                packPowerKw = profile.packVoltageV .* profile.packCurrentA / 1000;
                fprintf('Pack voltage range: %.3f to %.3f V\n', ...
                    lts.util.minFinite(profile.packVoltageV), ...
                    lts.util.maxFinite(profile.packVoltageV));
                fprintf('Pack current range: %.3f to %.3f A\n', ...
                    lts.util.minFinite(profile.packCurrentA), ...
                    lts.util.maxFinite(profile.packCurrentA));
                fprintf('Pack power range: %.3f to %.3f kW\n', ...
                    lts.util.minFinite(packPowerKw), ...
                    lts.util.maxFinite(packPowerKw));
            end
        end

        function validatePressureBrakeMode(profile, vehicle, brakeMode)
            if brakeMode ~= "pressure"
                return;
            end

            if ~any(isfinite(profile.brakePressureFrontBar)) || ...
                    ~any(isfinite(profile.brakePressureRearBar))
                error('run_correlation:MissingBrakePressureChannels', ...
                    ['BrakeMode "pressure" requires brake_pressure_front_bar and ' ...
                    'brake_pressure_rear_bar in the replay CSV. Re-extract the log with ' ...
                    'the R25 channel map so Brake Pressure Front/Rear are carried through.']);
            end

            if ~isfinite(vehicle.brakePressureFrontForcePerBar) || ...
                    ~isfinite(vehicle.brakePressureRearForcePerBar)
                error('run_correlation:MissingBrakePressureCalibration', ...
                    ['BrakeMode "pressure" requires vehicle brakePressure.frontForcePerBar ' ...
                    'and brakePressure.rearForcePerBar calibration.']);
            end
        end

        function validatePowertrainCommandMode(profile, powertrainMode, limitMotorTorqueByPackPower)
            if nargin < 3 || isempty(limitMotorTorqueByPackPower)
                limitMotorTorqueByPackPower = false;
            end
            powertrainMode = lts.correlation.CorrelationAppSupport.validatePowertrainMode(powertrainMode);
            if powertrainMode ~= "motor_torque_command"
                return;
            end

            if ~profile.hasMotorTorqueCommand()
                error('run_correlation:MissingMotorTorqueCommandChannel', ...
                    ['PowertrainMode "motor_torque_command" requires ' ...
                    'motor_torque_command_nm in the replay CSV. Re-extract the log ' ...
                    'with the R25 channel map so BAMOCAR Channels Calculated Cmd ' ...
                    'is decoded and carried through.']);
            end

            if logical(limitMotorTorqueByPackPower) && ~profile.hasPackPower()
                warning('run_correlation:MissingPackPowerChannels', ...
                    ['PowertrainMode "motor_torque_command" is missing ' ...
                    'pack_voltage_v and/or pack_current_a, so the replay cannot ' ...
                    'limit the calculated command against measured DC pack power. ' ...
                    'Re-extract with the R25 channel map to use pack-power limiting.']);
            end
        end

        function warnOnSteeringScale(profile, vehicle, alignmentDistanceM)
            if isempty(profile.steer) || isempty(profile.speed)
                return;
            end

            window = isfinite(profile.distance) & profile.distance >= 0 & ...
                profile.distance <= min(alignmentDistanceM, profile.distance(end));
            if nnz(window) < 3
                window = true(size(profile.steer));
            end

            wheelbase = 1.5;
            if isprop(vehicle, 'wheelbase') && isfinite(vehicle.wheelbase) && vehicle.wheelbase > 0
                wheelbase = vehicle.wheelbase;
            end

            kinLatG = profile.speed(window).^2 .* tan(profile.steer(window)) ./ wheelbase ./ 9.80665;
            kinLatG = abs(kinLatG(isfinite(kinLatG)));
            if isempty(kinLatG)
                return;
            end

            measured = [];
            if profile.hasLatAccel()
                measured = abs(profile.latAccelG(window));
                measured = measured(isfinite(measured));
            elseif ~isempty(profile.yawRate) && any(isfinite(profile.yawRate))
                yawLatG = profile.speed(window) .* profile.yawRate(window) ./ 9.80665;
                measured = abs(yawLatG(isfinite(yawLatG)));
            end

            medianKin = median(kinLatG);
            fprintf('Median steering-implied lateral demand: %.2f g\n', medianKin);
            if isempty(measured)
                return;
            end

            medianMeasured = median(measured);
            fprintf('Median logged lateral demand reference: %.2f g\n', medianMeasured);
            ratio = medianKin / max(medianMeasured, 0.05);
            if medianKin > 0.8 && ratio > 2.5
                warning('run_correlation:ImplausibleSteeringScale', ...
                    ['Steering input implies %.2f g median lateral demand, %.1fx the logged reference. ' ...
                     'Check the steer_rad source scale or steering ratio.'], ...
                    medianKin, ratio);
            end
        end

        function warnOnLateralAccelConsistency(profile, vehicle)
            if ~profile.hasLatAccel() || isempty(profile.yawRate) || ...
                    ~any(isfinite(profile.yawRate))
                return;
            end

            wheelbase = 1.5;
            if isprop(vehicle, 'wheelbase') && isfinite(vehicle.wheelbase) && vehicle.wheelbase > 0
                wheelbase = vehicle.wheelbase;
            end

            report = lts.diagnostics.LateralGDiagnostics.assessSignals( ...
                profile.time, profile.latAccelG, profile.speed, profile.yawRate, ...
                profile.steer, wheelbase);

            fprintf('Raw lateral accel peak: %.2f g | yaw-rate-derived peak: %.2f g | steer-demand peak: %.2f g\n', ...
                report.rawPeakAbsG, report.yawPeakAbsG, report.steerPeakAbsG);
            for i = 1:numel(report.messages)
                warning('run_correlation:LateralAccelKinematicMismatch', '%s', char(report.messages(i)));
            end
        end

        function warnOnBrakeScale(profile)
            maxBrake = max(profile.brake(isfinite(profile.brake)));
            if isempty(maxBrake)
                return;
            end
            if maxBrake < 0.2
                warning('run_correlation:LowBrakeScale', ...
                    ['Maximum brake_ratio is %.3f. If logged brake pressure is valid, ' ...
                     'check the brake channel map or direct brake source before judging braking correlation.'], ...
                    maxBrake);
            end
        end

        function value = jsonField(s, field, defaultValue)
            value = defaultValue;
            if isstruct(s) && isfield(s, field)
                value = s.(field);
                if isempty(value)
                    value = defaultValue;
                end
            end
        end

        function value = structField(s, fieldName)
            value = [];
            if ~isstruct(s)
                return;
            end

            names = fieldnames(s);
            idx = find(strcmpi(names, fieldName), 1, 'first');
            if ~isempty(idx)
                value = s.(names{idx});
            end
        end

        function config = applyVehicleTuningFunction(config, fn)
            try
                if nargin(fn) == 0
                    tuned = fn();
                else
                    tuned = fn(config);
                end
            catch err
                error('run_correlation:TuningFailed', ...
                    'Vehicle tuning function failed: %s', err.message);
            end

            if isa(tuned, 'lts.vehicle.VehicleConfig')
                config = tuned;
            elseif isstruct(tuned)
                config = lts.correlation.CorrelationAppSupport.applyVehicleOverrideStruct( ...
                    config, tuned);
            else
                error('run_correlation:InvalidTuningResult', ...
                    'Vehicle tuning function must return a lts.vehicle.VehicleConfig or override struct.');
            end
        end

        function fn = tuningFunctionFromName(tuningSpec)
            name = char(tuningSpec);
            [folder, baseName, ext] = fileparts(name);
            if strcmpi(ext, '.m')
                packageName = lts.correlation.CorrelationAppSupport.packageNameFromFolder(folder);
                if ~isempty(packageName)
                    fn = str2func([packageName '.' baseName]);
                else
                    if ~isempty(folder)
                        addpath(folder);
                    end
                    fn = str2func(baseName);
                end
            elseif contains(name, '.')
                fn = str2func(name);
            else
                fn = str2func(['lts.vehicles.' name]);
            end
        end

        function packageName = packageNameFromFolder(folder)
            packageParts = {};
            while ~isempty(folder)
                [parent, leaf] = fileparts(folder);
                if startsWith(leaf, '+')
                    packageParts = [{leaf(2:end)} packageParts]; %#ok<AGROW>
                    folder = parent;
                else
                    break;
                end
            end
            packageName = strjoin(packageParts, '.');
        end

        function config = applyVehicleOverrideStruct(config, overrides)
            fields = fieldnames(overrides);
            for i = 1:numel(fields)
                field = fields{i};
                if ~isprop(config, field)
                    error('run_correlation:UnknownVehicleOverride', ...
                        'Unknown lts.vehicle.VehicleConfig override field "%s".', field);
                end

                currentValue = config.(field);
                overrideValue = overrides.(field);
                if isstruct(currentValue) && isstruct(overrideValue)
                    config.(field) = lts.util.mergeStructRecursive(currentValue, overrideValue);
                else
                    config.(field) = overrideValue;
                end
            end
        end
    end
end
