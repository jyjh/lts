function [stateLog, lapTime, outputs] = run_correlation(varargin)
% RUN_CORRELATION Replay real MoTeC controls through the simulator.
%
% Example:
%   run_correlation( ...
%       'MoTeCFile', 'data/real_run.ld', ...
%       'Lap', 4, ...
%       'VehicleConfig', @vehicles.R25, ...
%       'Track', '2026enduro')

repoRoot = fileparts(fileparts(mfilename('fullpath')));
defaultChannelMap = fullfile(repoRoot, 'config', 'motec', 'r25_real_channel_map.json');
if ~exist(defaultChannelMap, 'file')
    defaultChannelMap = fullfile(repoRoot, 'config', 'motec', 'default_channel_map.json');
end

parser = inputParser;
parser.addParameter('MoTeCFile', '', @(x) ischar(x) || isstring(x));
parser.addParameter('ReplayCsv', '', @(x) ischar(x) || isstring(x));
parser.addParameter('Lap', [], @(x) isempty(x) || isnumeric(x) || ischar(x) || isstring(x));
parser.addParameter('LdxFile', '', @(x) ischar(x) || isstring(x));
parser.addParameter('ChannelMap', defaultChannelMap, @(x) ischar(x) || isstring(x));
parser.addParameter('VehicleConfig', @vehicles.R25);
parser.addParameter('Track', '2026enduro');
parser.addParameter('ReplayDomain', 'time', @(x) ischar(x) || isstring(x));
parser.addParameter('Dt', 0.001, @(x) isnumeric(x) && isscalar(x) && x > 0);
parser.addParameter('ImportFrequency', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x) && x > 0));
parser.addParameter('StopOnOffTrack', false, @(x) islogical(x) || isnumeric(x));
parser.addParameter('StopAtTrackEnd', false, @(x) islogical(x) || isnumeric(x));
parser.addParameter('StopAtReplayEnd', true, @(x) islogical(x) || isnumeric(x));
parser.addParameter('OutputBase', '', @(x) ischar(x) || isstring(x));
parser.addParameter('PythonCommand', 'python', @(x) ischar(x) || isstring(x));
parser.addParameter('ExportMoTeC', true, @(x) islogical(x) || isnumeric(x));
parser.addParameter('UseLoggedPosition', true, @(x) islogical(x) || isnumeric(x));
parser.addParameter('UseLoggedYawRate', true, @(x) islogical(x) || isnumeric(x));
parser.addParameter('ShowPlots', false, @(x) islogical(x) || isnumeric(x));
parser.parse(varargin{:});
opts = parser.Results;

if isempty(opts.ReplayCsv) && isempty(opts.MoTeCFile)
    error('run_correlation:MissingInput', ...
        'Provide either MoTeCFile or ReplayCsv.');
end

track = loadCorrelationTrack(opts.Track, repoRoot);
config = loadVehicleConfig(opts.VehicleConfig);
dt = opts.Dt;

outputs = struct();
outputs.outputBase = buildOutputBase(opts, config, repoRoot);
outputs.replayCsv = char(opts.ReplayCsv);
outputs.extractManifest = '';

if isempty(outputs.replayCsv)
    outputs.replayCsv = [outputs.outputBase '_replay.csv'];
    outputs.extractManifest = [outputs.outputBase '_extract_manifest.json'];
    extractMoTeCLap(opts, outputs.replayCsv, outputs.extractManifest, repoRoot);
end

profile = CorrelationReplayProfile.fromCsv(outputs.replayCsv);
surfaceMu = representativeSurfaceMu(track);
outputs.referenceMode = 'free';
outputs.surfaceMu = surfaceMu;

vehicle = VehicleManager.fromConfig(config, track, dt);
preflightCorrelation(profile, track, vehicle, surfaceMu, outputs.extractManifest);
initialState = CorrelationStateInitializer.fromReplayProfile( ...
    profile, [], vehicle, ...
    'UseLoggedPosition', opts.UseLoggedPosition, ...
    'UseLoggedYawRate', opts.UseLoggedYawRate);

simulator = Simulator(vehicle, [], dt);
[stateLog, lapTime] = simulator.simulateReplay( ...
    initialState, track, profile, ...
    'ReplayDomain', opts.ReplayDomain, ...
    'AllowPedalOverlap', true, ...
    'ApplySteeringSlew', false, ...
    'StopOnOffTrack', opts.StopOnOffTrack, ...
    'StopAtTrackEnd', opts.StopAtTrackEnd, ...
    'StopAtReplayEnd', opts.StopAtReplayEnd, ...
    'ReferenceMode', 'free', ...
    'SurfaceMu', surfaceMu);

outputs.csvFile = [outputs.outputBase '.csv'];
outputs.ldFile = [outputs.outputBase '.ld'];

if logical(opts.ExportMoTeC)
    TelemetryExporter.exportToMoTeCLog( ...
        stateLog, outputs.csvFile, ...
        'OutputFile', outputs.ldFile, ...
        'Frequency', 1 / dt, ...
        'VehicleWeight', round(vehicle.totalMass), ...
        'VehicleId', config.name, ...
        'VenueName', trackName(track), ...
        'EventName', 'FSAE LTS Correlation Replay', ...
        'VehicleType', 'FSAE');
else
    TelemetryExporter.writeToMoTeCFormat(stateLog, outputs.csvFile);
    outputs.ldFile = '';
end

if logical(opts.ShowPlots)
    GraphPlotter.plotAll(stateLog, lapTime, track, vehicle, vehicle.aero, false);
end

fprintf('\n=== Correlation Replay Complete ===\n');
fprintf('Replay CSV: %s\n', outputs.replayCsv);
fprintf('Sim CSV:    %s\n', outputs.csvFile);
if ~isempty(outputs.ldFile)
    fprintf('Sim LD:     %s\n', outputs.ldFile);
end
fprintf('Lap Time:   %.3f s\n', lapTime);
end

function preflightCorrelation(profile, track, vehicle, surfaceMu, manifestFile)
fprintf('\n=== Correlation Preflight ===\n');
printExtractionSummary(manifestFile);
fprintf('Reference mode: free-space replay\n');
fprintf('Surface mu: %.3f\n', surfaceMu);
printReplayRanges(profile);
warnOnSteeringScale(profile, vehicle, 120);
warnOnBrakeScale(profile);

if nargin >= 2 && ~isempty(track)
    fprintf('Environment track length: %.2f m (not used for path projection)\n', ...
        track.getTotalLength());
end
end

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

names = {'throttle_ratio', 'brake_ratio', 'steer_rad', 'yaw_rad', ...
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
    sourceName = jsonField(channel, 'name', '');
    sourceLabel = jsonField(channel, 'source_label', '');
    scale = jsonField(channel, 'scale_applied', NaN);
    minValue = jsonField(channel, 'min_value', NaN);
    maxValue = jsonField(channel, 'max_value', NaN);
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

function extractMoTeCLap(opts, replayCsv, manifestFile, repoRoot)
script = fullfile(repoRoot, 'scripts', 'extract_motec_lap.py');
args = { ...
    char(opts.PythonCommand), script, ...
    '--input', char(opts.MoTeCFile), ...
    '--output', replayCsv, ...
    '--manifest', manifestFile, ...
    '--channel-map', char(opts.ChannelMap)};

if ~isempty(opts.Lap)
    args(end+1:end+2) = {'--laps', lapValue(opts.Lap)}; %#ok<AGROW>
end
if ~isempty(opts.LdxFile)
    args(end+1:end+2) = {'--ldx', char(opts.LdxFile)}; %#ok<AGROW>
end
if ~isempty(opts.ImportFrequency)
    args(end+1:end+2) = {'--frequency', sprintf('%.9g', opts.ImportFrequency)}; %#ok<AGROW>
end

command = joinCommand(args);
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
    lapSuffix = ['_lap' regexprep(lapValue(opts.Lap), '[^A-Za-z0-9]', '_')];
end

exportDir = fullfile(repoRoot, 'exports');
base = fullfile(exportDir, sprintf('correlation_%s%s_%s_%s', ...
    name, lapSuffix, char(config.name), datestr(now, 'yyyymmdd_HHMMSS')));
end

function track = loadCorrelationTrack(trackSpec, repoRoot)
if isa(trackSpec, 'components.Track')
    track = trackSpec;
    return;
end

trackText = char(trackSpec);
if strcmpi(trackText, '2026enduro')
    track = components.WaypointTrack.loadMat( ...
        fullfile(repoRoot, 'tracks', ...
        'endurance_track_grid_25ft_from_matlab_smoothed.mat'));
    track.Width = 5.0;
elseif endsWith(lower(trackText), '.mat')
    track = components.WaypointTrack.loadMat(trackText);
elseif endsWith(lower(trackText), '.csv')
    track = components.WaypointTrack.fromCsv(trackText);
else
    track = components.TestTrack(trackText);
end
end

function config = loadVehicleConfig(configSpec)
if isa(configSpec, 'function_handle')
    config = configSpec();
elseif isa(configSpec, 'VehicleConfig')
    config = configSpec;
elseif ischar(configSpec) || isstring(configSpec)
    name = char(configSpec);
    if contains(name, '.')
        fn = str2func(name);
    else
        fn = str2func(['vehicles.' name]);
    end
    config = fn();
else
    config = configSpec;
end
end

function mu = representativeSurfaceMu(track)
mu = 1.2;
if isempty(track)
    return;
end

try
    values = track.getSurfaceFriction();
catch
    return;
end

values = values(:);
values = values(isfinite(values) & values > 0);
if ~isempty(values)
    mu = values(1);
end
end

function name = trackName(track)
name = 'track';
if isprop(track, 'Name') && ~isempty(track.Name)
    name = track.Name;
end
end

function command = joinCommand(args)
quoted = cell(size(args));
for i = 1:numel(args)
    quoted{i} = quoteShellArg(args{i});
end
command = strjoin(quoted, ' ');
end

function value = quoteShellArg(value)
value = char(value);
value = strrep(value, '"', '\"');
value = ['"' value '"'];
end
