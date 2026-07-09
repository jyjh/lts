function [stateLog, lapTime, outputs] = run_correlation(varargin)
% RUN_CORRELATION Replay real MoTeC controls through the simulator.
%
% Example:
%   lts.app.run_correlation( ...
%       'MoTeCFile', 'data/real_run.ld', ...
%       'Lap', 4, ...
%       'VehicleConfig', @lts.vehicles.R25, ...
%       'TuningFile', 'R25_correlation_tuning', ...
%       'PowertrainMode', 'motor_torque_command', ...
%       'LimitMotorTorqueByPackPower', true, ...
%       'CorrelationConfig', 'exports/correlation_lap5_config.json', ...
%       'PackPowerAdvanceS', 0.06, ...
%       'Track', '2026enduro')

repoRoot = lts.util.repoRoot(mfilename('fullpath'));
defaultChannelMap = fullfile(repoRoot, 'config', 'motec', 'r25_real_channel_map.json');
if ~exist(defaultChannelMap, 'file')
    defaultChannelMap = fullfile(repoRoot, 'config', 'motec', 'default_channel_map.json');
end

parser = inputParser;
providedOptionNames = lower(string(varargin(1:2:end)));
hasPackPowerAdvanceOption = any(providedOptionNames == "packpoweradvances");
parser.addParameter('MoTeCFile', '', @(x) ischar(x) || isstring(x));
parser.addParameter('ReplayCsv', '', @(x) ischar(x) || isstring(x));
parser.addParameter('Lap', [], @(x) isempty(x) || isnumeric(x) || ischar(x) || isstring(x));
parser.addParameter('LdxFile', '', @(x) ischar(x) || isstring(x));
parser.addParameter('ChannelMap', defaultChannelMap, @(x) ischar(x) || isstring(x));
parser.addParameter('CorrelationConfig', '', @(x) ischar(x) || isstring(x));
parser.addParameter('VehicleConfig', @lts.vehicles.R25);
parser.addParameter('VehicleTuning', [], @(x) isempty(x) || isa(x, 'function_handle') || ischar(x) || isstring(x) || isa(x, 'lts.vehicle.VehicleConfig') || isstruct(x));
parser.addParameter('TuningFile', [], @(x) isempty(x) || isa(x, 'function_handle') || ischar(x) || isstring(x) || isa(x, 'lts.vehicle.VehicleConfig') || isstruct(x));
parser.addParameter('Track', '2026enduro');
parser.addParameter('ReplayDomain', 'time', @(x) ischar(x) || isstring(x));
parser.addParameter('BrakeMode', 'ratio', @(x) ischar(x) || isstring(x));
parser.addParameter('PowertrainMode', 'throttle', @(x) ischar(x) || isstring(x));
parser.addParameter('LimitMotorTorqueByPackPower', true, @(x) islogical(x) || isnumeric(x));
parser.addParameter('PackPowerAdvanceS', 0, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x));
parser.addParameter('Dt', 0.001, @(x) isnumeric(x) && isscalar(x) && x > 0);
parser.addParameter('TelemetryMode', 'full', ...
    @(x) any(strcmpi(string(x), ["full", "lean"])));
parser.addParameter('WheelSolveIterations', [], ...
    @(x) isempty(x) || (isnumeric(x) && isscalar(x) && isfinite(x) && x > 0));
parser.addParameter('ImportFrequency', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x) && x > 0));
parser.addParameter('StopOnOffTrack', false, @(x) islogical(x) || isnumeric(x));
parser.addParameter('StopAtTrackEnd', false, @(x) islogical(x) || isnumeric(x));
parser.addParameter('StopAtReplayEnd', true, @(x) islogical(x) || isnumeric(x));
parser.addParameter('SurfaceMu', NaN, @(x) isempty(x) || (isnumeric(x) && isscalar(x)));
parser.addParameter('OutputBase', '', @(x) ischar(x) || isstring(x));
parser.addParameter('PythonCommand', 'python', @(x) ischar(x) || isstring(x));
parser.addParameter('ExportMoTeC', true, @(x) islogical(x) || isnumeric(x));
parser.addParameter('ExportChannels', {}, @(x) iscell(x) || isstring(x) || ischar(x));
parser.addParameter('IncludeDerived', true, @(x) islogical(x) || isnumeric(x));
parser.addParameter('UseLoggedPosition', true, @(x) islogical(x) || isnumeric(x));
parser.addParameter('UseLoggedYawRate', [], @(x) isempty(x) || islogical(x) || isnumeric(x));
parser.addParameter('UseLoggedTransientState', [], @(x) isempty(x) || islogical(x) || isnumeric(x));
parser.addParameter('InitialTransientWindowS', [], ...
    @(x) isempty(x) || (isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0));
parser.addParameter('ShowPlots', false, @(x) islogical(x) || isnumeric(x));
parser.parse(varargin{:});
opts = parser.Results;
opts.BrakeMode = lts.correlation.CorrelationAppSupport.validateBrakeMode(opts.BrakeMode);
opts.PowertrainMode = lts.correlation.CorrelationAppSupport.validatePowertrainMode(opts.PowertrainMode);
correlationConfig = lts.correlation.CorrelationAppSupport.loadCorrelationConfig( ...
    opts.CorrelationConfig);
if ~hasPackPowerAdvanceOption
    opts.PackPowerAdvanceS = lts.correlation.CorrelationAppSupport.correlationConfigScalar( ...
        correlationConfig, 'PackPowerAdvanceS', opts.PackPowerAdvanceS);
end

if isempty(opts.ReplayCsv) && isempty(opts.MoTeCFile)
    error('run_correlation:MissingInput', ...
        'Provide either MoTeCFile or ReplayCsv.');
end

track = lts.correlation.CorrelationAppSupport.loadTrack(opts.Track, repoRoot);
config = lts.correlation.CorrelationAppSupport.loadVehicleConfig(opts.VehicleConfig);
config = lts.correlation.CorrelationAppSupport.applyVehicleTuning( ...
    config, opts.VehicleTuning, opts.TuningFile);
dt = opts.Dt;

outputs = struct();
outputs.outputBase = lts.correlation.CorrelationAppSupport.buildOutputBase( ...
    opts, config, repoRoot);
outputs.replayCsv = char(opts.ReplayCsv);
outputs.extractManifest = '';
outputs.vehicleConfig = char(config.name);
outputs.correlationConfig = char(opts.CorrelationConfig);

if isempty(outputs.replayCsv)
    outputs.replayCsv = [outputs.outputBase '_replay.csv'];
    outputs.extractManifest = [outputs.outputBase '_extract_manifest.json'];
    lts.correlation.CorrelationAppSupport.extractMoTeCLap( ...
        opts, outputs.replayCsv, outputs.extractManifest, repoRoot);
end

profile = lts.correlation.CorrelationReplayProfile.fromCsv(outputs.replayCsv);
profile = profile.withPackPowerAdvance(opts.PackPowerAdvanceS);
surfaceMu = opts.SurfaceMu;
if isempty(surfaceMu) || ~isfinite(surfaceMu) || surfaceMu <= 0
    surfaceMu = lts.correlation.CorrelationAppSupport.vehicleCorrelationSurfaceMu(config);
end
if ~isfinite(surfaceMu) || surfaceMu <= 0
    surfaceMu = lts.util.representativeSurfaceMu(track);
end
outputs.referenceMode = 'free';
outputs.surfaceMu = surfaceMu;
outputs.brakeMode = char(opts.BrakeMode);
outputs.powertrainMode = char(opts.PowertrainMode);
outputs.limitMotorTorqueByPackPower = logical(opts.LimitMotorTorqueByPackPower);
outputs.packPowerAdvanceS = double(opts.PackPowerAdvanceS);
outputs.gpsAdvanceS = lts.correlation.CorrelationAppSupport.correlationConfigScalar( ...
    correlationConfig, 'GpsAdvanceS', 0);
useLoggedYawRate = lts.correlation.CorrelationAppSupport.vehicleCorrelationFlag( ...
    config, 'useLoggedYawRate', true);
if ~isempty(opts.UseLoggedYawRate)
    useLoggedYawRate = logical(opts.UseLoggedYawRate);
end
useLoggedTransientState = lts.correlation.CorrelationAppSupport.vehicleCorrelationFlag( ...
    config, 'useLoggedTransientState', true);
if ~isempty(opts.UseLoggedTransientState)
    useLoggedTransientState = logical(opts.UseLoggedTransientState);
end
initialTransientWindowS = lts.correlation.CorrelationAppSupport.vehicleCorrelationScalar( ...
    config, 'initialTransientWindowS', 0);
if ~isempty(opts.InitialTransientWindowS)
    initialTransientWindowS = double(opts.InitialTransientWindowS);
end
outputs.useLoggedYawRate = useLoggedYawRate;
outputs.useLoggedTransientState = useLoggedTransientState;
outputs.initialTransientWindowS = initialTransientWindowS;

vehicle = lts.vehicle.VehicleManager.fromConfig(config, track, dt);
lts.correlation.CorrelationAppSupport.preflight( ...
    profile, track, vehicle, surfaceMu, outputs.extractManifest, ...
    opts.BrakeMode, opts.PowertrainMode, opts.LimitMotorTorqueByPackPower, ...
    opts.PackPowerAdvanceS);
initialState = lts.correlation.CorrelationStateInitializer.fromReplayProfile( ...
    profile, [], vehicle, ...
    'UseLoggedPosition', opts.UseLoggedPosition, ...
    'UseLoggedYawRate', useLoggedYawRate, ...
    'UseLoggedTransientState', useLoggedTransientState, ...
    'InitialTransientWindowS', initialTransientWindowS);

simulator = lts.simulation.Simulator(vehicle, [], dt);
simulator.telemetryMode = lower(string(opts.TelemetryMode));
if ~isempty(opts.WheelSolveIterations)
    simulator.wheelSolveIterations = max(1, round(double(opts.WheelSolveIterations)));
end
[stateLog, lapTime] = simulator.simulateReplay( ...
    initialState, track, profile, ...
    'ReplayDomain', opts.ReplayDomain, ...
    'BrakeMode', opts.BrakeMode, ...
    'PowertrainMode', opts.PowertrainMode, ...
    'LimitMotorTorqueByPackPower', opts.LimitMotorTorqueByPackPower, ...
    'AllowPedalOverlap', true, ...
    'ApplySteeringSlew', false, ...
    'StopOnOffTrack', opts.StopOnOffTrack, ...
    'StopAtTrackEnd', opts.StopAtTrackEnd, ...
    'StopAtReplayEnd', opts.StopAtReplayEnd, ...
    'ReferenceMode', 'free', ...
    'SurfaceMu', surfaceMu);

outputs.csvFile = [outputs.outputBase '.csv'];
outputs.ldFile = [outputs.outputBase '.ld'];
outputs.telemetryMode = char(simulator.telemetryMode);
outputs.wheelSolveIterations = simulator.wheelSolveIterations;
exportArgs = {'Channels', opts.ExportChannels, ...
    'IncludeDerived', logical(opts.IncludeDerived)};

if logical(opts.ExportMoTeC)
    lts.telemetry.TelemetryExporter.exportToMoTeCLog( ...
        stateLog, outputs.csvFile, exportArgs{:}, ...
        'OutputFile', outputs.ldFile, ...
        'Frequency', 1 / dt, ...
        'VehicleWeight', round(vehicle.totalMass), ...
        'VehicleId', config.name, ...
        'VenueName', lts.correlation.CorrelationAppSupport.trackName(track), ...
        'EventName', 'FSAE LTS Correlation Replay', ...
        'VehicleType', 'FSAE');
else
    lts.telemetry.TelemetryExporter.writeToMoTeCFormat( ...
        stateLog, outputs.csvFile, exportArgs{:});
    outputs.ldFile = '';
end

if logical(opts.ShowPlots)
    lts.telemetry.GraphPlotter.plotAll(stateLog, lapTime, track, vehicle, vehicle.aero, false);
end

fprintf('\n=== Correlation Replay Complete ===\n');
fprintf('Replay CSV: %s\n', outputs.replayCsv);
fprintf('Sim CSV:    %s\n', outputs.csvFile);
if ~isempty(outputs.ldFile)
    fprintf('Sim LD:     %s\n', outputs.ldFile);
end
fprintf('Lap Time:   %.3f s\n', lapTime);
end
