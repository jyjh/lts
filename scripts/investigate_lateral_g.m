function varargout = investigate_lateral_g(varargin)
% INVESTIGATE_LATERAL_G Compare raw, kinematic, simulated, and tire-capacity Ay.
%
% Example:
%   investigate_lateral_g( ...
%       'SimCsv', 'exports/correlation_run.csv', ...
%       'ReplayCsv', 'exports/correlation_run_replay.csv', ...
%       'ReportFile', 'exports/lateral_g_report.md')

scriptDir = fileparts(mfilename('fullpath'));
repoRoot = fileparts(scriptDir);
addpath(fullfile(repoRoot, 'src'));

parser = inputParser;
parser.addParameter('SimCsv', '', @(x) ischar(x) || isstring(x));
parser.addParameter('ReplayCsv', '', @(x) ischar(x) || isstring(x));
parser.addParameter('VehicleConfig', @lts.vehicles.R25);
parser.addParameter('Track', '2026enduro');
parser.addParameter('ReportFile', '', @(x) ischar(x) || isstring(x));
parser.parse(varargin{:});
opts = parser.Results;

if isempty(opts.SimCsv) || isempty(opts.ReplayCsv)
    error('investigate_lateral_g:MissingInput', ...
        'SimCsv and ReplayCsv are required.');
end

cfg = loadVehicleConfig(opts.VehicleConfig);
track = loadDiagnosticTrack(opts.Track, repoRoot);
surfaceMu = lts.util.representativeSurfaceMu(track);

simCsv = char(opts.SimCsv);
replayCsv = char(opts.ReplayCsv);
simRaw = readNumericCsv(simCsv);
realRaw = readNumericCsv(replayCsv);

sim = extractSimSignals(simRaw, cfg);
real = extractReplaySignals(realRaw);

realReport = lts.diagnostics.LateralGDiagnostics.assessSignals( ...
    real.time, real.rawLatG, real.speedMps, real.yawRateRadps, ...
    real.steerRad, cfg.wheelbase);
simReport = lts.diagnostics.LateralGDiagnostics.assessSignals( ...
    sim.time, sim.ayG, sim.speedMps, sim.yawRateRadps, ...
    sim.steerRad, cfg.wheelbase);

realMismatchEvents = lts.diagnostics.LateralGDiagnostics.topMismatchEvents( ...
    realReport.time, realReport.rawLatG, realReport.yawLatG, ...
    realReport.steerLatG, realReport.speedMps, 5, 0.25);

realRawOnSim = interpChannel(real.time, real.rawLatG, sim.time);
realYawOnSim = interpChannel(real.time, realReport.yawLatG, sim.time);
simDiffEvents = lts.diagnostics.LateralGDiagnostics.topMismatchEvents( ...
    sim.time, realRawOnSim, sim.ayG, realYawOnSim, sim.speedMps, 5, 0.25);

capacity = computeTireCapacity(simRaw, sim, cfg);

lines = buildReportLines( ...
    simCsv, replayCsv, cfg, surfaceMu, realReport, simReport, ...
    realMismatchEvents, simDiffEvents, sim, realRawOnSim, capacity);

reportText = strjoin(lines, newline);
fprintf('%s\n', reportText);

reportFile = char(opts.ReportFile);
if ~isempty(reportFile)
    [folder, ~, ~] = fileparts(reportFile);
    if ~isempty(folder) && ~exist(folder, 'dir')
        mkdir(folder);
    end
    fid = fopen(reportFile, 'w');
    if fid < 0
        error('investigate_lateral_g:FileOpenFailed', ...
            'Could not open report file "%s".', reportFile);
    end
    cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid, '%s\n', reportText);
    fprintf('Wrote lateral-G report: %s\n', reportFile);
end

result = struct( ...
    'realReport', realReport, ...
    'simReport', simReport, ...
    'realMismatchEvents', realMismatchEvents, ...
    'simDiffEvents', simDiffEvents, ...
    'capacity', capacity, ...
    'reportText', reportText);

if nargout > 0
    varargout{1} = result;
end
end

function lines = buildReportLines(simCsv, replayCsv, cfg, surfaceMu, ...
        realReport, simReport, realMismatchEvents, simDiffEvents, ...
        sim, realRawOnSim, capacity)
lines = {};
lines{end + 1} = '# Lateral-G Diagnostic Report';
lines{end + 1} = '';
lines{end + 1} = sprintf('- Sim CSV: `%s`', simCsv);
lines{end + 1} = sprintf('- Replay CSV: `%s`', replayCsv);
lines{end + 1} = sprintf('- Vehicle: `%s`, mass %.1f kg, wheelbase %.3f m', ...
    char(cfg.name), cfg.totalMass, cfg.wheelbase);
lines{end + 1} = sprintf('- Track surface mu reference: %.3f', surfaceMu);
lines{end + 1} = '';

lines{end + 1} = '## Signal Sanity';
lines{end + 1} = sprintf('- Real raw lateral accel peak: %.2f g (p95 %.2f g)', ...
    realReport.rawPeakAbsG, realReport.rawP95AbsG);
lines{end + 1} = sprintf('- Real speed*yaw-rate peak: %.2f g (p95 %.2f g)', ...
    realReport.yawPeakAbsG, realReport.yawP95AbsG);
lines{end + 1} = sprintf('- Real steering-implied demand peak: %.2f g (p95 %.2f g)', ...
    realReport.steerPeakAbsG, realReport.steerP95AbsG);
lines{end + 1} = sprintf('- Real raw-vs-yaw sign mismatch: %.1f%% of active samples', ...
    100 * realReport.signMismatchFraction);
lines{end + 1} = sprintf('- Sim body Ay peak: %.2f g (p95 %.2f g)', ...
    simReport.rawPeakAbsG, simReport.rawP95AbsG);
lines{end + 1} = sprintf('- Sim speed*yaw-rate peak: %.2f g (p95 %.2f g)', ...
    simReport.yawPeakAbsG, simReport.yawP95AbsG);
if ~isempty(realReport.messages)
    for i = 1:numel(realReport.messages)
        lines{end + 1} = sprintf('- WARNING: %s', realReport.messages(i));
    end
end
lines{end + 1} = '';

lines{end + 1} = '## Real Raw-vs-Kinematic Suspect Windows';
lines = addEventTable(lines, realMismatchEvents, ...
    {'time_s', 'raw_g', 'yaw_g', 'steer_g', 'speed_mps', 'error_g'});
lines{end + 1} = '';

lines{end + 1} = '## Time-Aligned Real-vs-Sim Ay Gaps';
lines = addEventTable(lines, simDiffEvents, ...
    {'time_s', 'real_raw_g', 'sim_ay_g', 'real_yaw_g', 'sim_speed_mps', 'gap_g'});
lines{end + 1} = '';

lines{end + 1} = '## Tire Capacity And Utilization';
if capacity.available
    simTireLatPeak = max(abs(sim.tireLatG(isfinite(sim.tireLatG))));
    if isempty(simTireLatPeak)
        simTireLatPeak = NaN;
    end
    utilization = capacity.utilization(isfinite(capacity.utilization));
    if isempty(utilization)
        utilizationPeak = NaN;
        utilizationP95 = NaN;
    else
        utilizationPeak = max(utilization);
        utilizationP95 = percentile(utilization, 95);
    end
    capPeak = max(capacity.capacityG(isfinite(capacity.capacityG)));
    capP95 = percentile(capacity.capacityG, 95);
    rawOverCapacity = realRawOnSim;
    overCapacity = isfinite(rawOverCapacity) & isfinite(capacity.capacityG) & ...
        abs(rawOverCapacity) > capacity.capacityG + 0.2;
    lines{end + 1} = sprintf('- Sim tire-force-derived Ay peak: %.2f g', ...
        simTireLatPeak);
    lines{end + 1} = sprintf('- Sim tire capacity peak: %.2f g (p95 %.2f g)', ...
        capPeak, capP95);
    lines{end + 1} = sprintf('- Sim tire utilization peak: %.0f%% (p95 %.0f%%)', ...
        100 * utilizationPeak, 100 * utilizationP95);
    lines{end + 1} = sprintf(['- Time-aligned real raw samples above sim-state ' ...
        'capacity by >0.2 g: %.1f%% (diagnostic only; replay state is not constrained)'], ...
        100 * nnz(overCapacity) / max(nnz(isfinite(rawOverCapacity) & ...
        isfinite(capacity.capacityG)), 1));
else
    lines{end + 1} = '- Tire capacity unavailable: sim CSV is missing FZ corner loads.';
end
end

function lines = addEventTable(lines, events, headers)
if isempty(events)
    lines{end + 1} = '- No finite mismatch events found.';
    return;
end

lines{end + 1} = ['| ' strjoin(headers, ' | ') ' |'];
lines{end + 1} = ['| ' strjoin(repmat({'---'}, size(headers)), ' | ') ' |'];
for i = 1:numel(events)
    e = events(i);
    lines{end + 1} = sprintf('| %.3f | %.3f | %.3f | %.3f | %.3f | %.3f |', ...
        e.time, e.rawLatG, e.yawLatG, e.steerLatG, e.speedMps, e.scoreG);
end
end

function data = readNumericCsv(filepath)
filepath = char(filepath);
if ~exist(filepath, 'file')
    error('investigate_lateral_g:MissingCsv', ...
        'CSV file "%s" does not exist.', filepath);
end

fid = fopen(filepath, 'r');
if fid < 0
    error('investigate_lateral_g:FileOpenFailed', ...
        'Could not open "%s".', filepath);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
headerLine = fgetl(fid);
headers = strsplit(headerLine, ',');
values = readmatrix(filepath, 'NumHeaderLines', 1);
data = struct('filepath', filepath, 'headers', {headers}, 'values', values);
end

function sim = extractSimSignals(raw, cfg)
[time, ~] = channel(raw, {'Time', 'time_s'}, NaN);
[speed, speedUnit] = channel(raw, {'Speed mps', 'speed_mps'}, NaN);
if all(~isfinite(speed))
    [speed, speedUnit] = channel(raw, {'Vehicle Speed Value', 'Vehicle Speed'}, NaN);
end
speed = speedToMps(speed, speedUnit);

[ay, ayUnit] = channel(raw, {'G Sensor Front Acceleration Lateral', ...
    'Lat Accel Raw', 'ay', 'lat_accel_g'}, NaN);
ayG = accelToG(ay, ayUnit, 'm/s/s');

[yawRate, yawUnit] = channel(raw, {'Yaw Rate', 'yaw_rate_radps'}, NaN);
yawRate = angleRateToRadps(yawRate, yawUnit);

[steer, steerUnit] = channel(raw, {'Steer', 'Steer Raw', 'steer_rad'}, NaN);
steer = angleToRad(steer, steerUnit, 'deg');

[tireLat, ~] = channel(raw, {'F Tire Lat'}, NaN);
tireLatG = tireLat ./ max(cfg.totalMass * lts.diagnostics.LateralGDiagnostics.g, eps);

sim = struct( ...
    'time', time, ...
    'speedMps', speed, ...
    'ayG', ayG, ...
    'yawRateRadps', yawRate, ...
    'steerRad', steer, ...
    'tireLatG', tireLatG);
end

function replay = extractReplaySignals(raw)
[time, ~] = channel(raw, {'time_s', 'Time'}, NaN);
[speed, speedUnit] = channel(raw, {'speed_mps', 'Speed mps', 'Vehicle Speed Value'}, NaN);
speed = speedToMps(speed, speedUnit);

[rawLat, latUnit] = channel(raw, {'lat_accel_g', 'lateral_accel_g', ...
    'G Sensor Front Acceleration Lateral', 'Lat Accel Raw'}, NaN);
rawLatG = accelToG(rawLat, latUnit, 'G');

[yawRate, yawUnit] = channel(raw, {'yaw_rate_radps', 'Yaw Rate'}, NaN);
yawRate = angleRateToRadps(yawRate, yawUnit);

[steer, steerUnit] = channel(raw, {'steer_rad', 'Steer', 'Steer Raw'}, NaN);
steer = angleToRad(steer, steerUnit, 'rad');

replay = struct( ...
    'time', time, ...
    'speedMps', speed, ...
    'rawLatG', rawLatG, ...
    'yawRateRadps', yawRate, ...
    'steerRad', steer);
end

function capacity = computeTireCapacity(raw, sim, cfg)
corners = {'FL', 'FR', 'RL', 'RR'};
n = numel(sim.time);
loads = NaN(n, 4);
peakMu = NaN(n, 4);
for i = 1:4
    corner = corners{i};
    [loads(:, i), ~] = channel(raw, {['FZ ' corner], ['Fz ' corner], ['Fz_' corner]}, NaN);
    [peakMu(:, i), ~] = channel(raw, {['Peak MU ' corner], ['Peak Mu ' corner], ['peakMu_' corner]}, NaN);
end

capacity = struct('available', false, 'capacityG', NaN(n, 1), ...
    'utilization', NaN(n, 1));
if all(~isfinite(loads(:)))
    return;
end

missingPeakMu = all(~isfinite(peakMu(:)));
if missingPeakMu
    tire = lts.components.Tire.PacejkaTire(cfg.tire.tirFile);
    peakMu = computePeakMuForLoads(tire, loads);
end

capacityN = sum(max(loads, 0) .* max(peakMu, 0), 2, 'omitnan');
capacityG = capacityN ./ max(cfg.totalMass * lts.diagnostics.LateralGDiagnostics.g, eps);
utilization = abs(sim.tireLatG) ./ capacityG;
utilization(~isfinite(utilization)) = NaN;

capacity.available = any(isfinite(capacityG));
capacity.capacityG = capacityG;
capacity.utilization = utilization;
end

function peakMu = computePeakMuForLoads(tire, loads)
rounded = round(max(loads(:), 0) / 10) * 10;
uniqueLoads = unique(rounded(isfinite(rounded) & rounded > 0));
muByLoad = containers.Map('KeyType', 'double', 'ValueType', 'double');
for i = 1:numel(uniqueLoads)
    muByLoad(uniqueLoads(i)) = tire.getPeakFriction(uniqueLoads(i));
end

peakMu = NaN(size(loads));
for i = 1:numel(rounded)
    loadKey = rounded(i);
    if isfinite(loadKey) && loadKey > 0 && isKey(muByLoad, loadKey)
        peakMu(i) = muByLoad(loadKey);
    end
end
end

function values = interpChannel(axis, channel, query)
axis = double(axis(:));
channel = double(channel(:));
query = double(query(:));
keep = isfinite(axis) & isfinite(channel);
axis = axis(keep);
channel = channel(keep);
if isempty(axis)
    values = NaN(size(query));
    return;
end
[axis, ia] = unique(axis, 'stable');
channel = channel(ia);
if numel(axis) == 1
    values = repmat(channel(1), size(query));
else
    values = interp1(axis, channel, query, 'linear', NaN);
end
end

function [values, unit] = channel(data, aliases, defaultValue)
idx = findChannelIndex(data.headers, aliases);
if isempty(idx)
    values = repmat(defaultValue, size(data.values, 1), 1);
    unit = '';
    return;
end

values = data.values(:, idx);
unit = headerUnit(data.headers{idx});
end

function idx = findChannelIndex(headers, aliases)
idx = [];
normalizedHeaders = cellfun(@normalizeHeader, headers, 'UniformOutput', false);
normalizedBaseHeaders = cellfun(@(x) normalizeHeader(headerBase(x)), ...
    headers, 'UniformOutput', false);
for i = 1:numel(aliases)
    alias = normalizeHeader(aliases{i});
    match = find(strcmp(normalizedBaseHeaders, alias) | ...
        strcmp(normalizedHeaders, alias), 1, 'first');
    if ~isempty(match)
        idx = match;
        return;
    end
end
end

function name = headerBase(header)
name = regexprep(char(header), '\s*\([^)]*\)\s*$', '');
end

function unit = headerUnit(header)
tokens = regexp(char(header), '\(([^)]*)\)\s*$', 'tokens', 'once');
if isempty(tokens)
    unit = '';
else
    unit = tokens{1};
end
end

function name = normalizeHeader(name)
name = lower(char(name));
name = regexprep(name, '[^a-z0-9]', '');
end

function values = speedToMps(values, unit)
unit = normalizeHeader(unit);
if strcmp(unit, 'kmh') || strcmp(unit, 'kph')
    values = values / 3.6;
end
end

function values = accelToG(values, unit, defaultUnit)
unit = normalizeHeader(unit);
if isempty(unit)
    unit = normalizeHeader(defaultUnit);
end
if strcmp(unit, 'mss') || strcmp(unit, 'ms2') || strcmp(unit, 'mps2')
    values = values / lts.diagnostics.LateralGDiagnostics.g;
end
end

function values = angleToRad(values, unit, defaultUnit)
unit = normalizeHeader(unit);
if isempty(unit)
    unit = normalizeHeader(defaultUnit);
end
if strcmp(unit, 'deg') || strcmp(unit, 'degree') || strcmp(unit, 'degrees')
    values = values * pi / 180;
end
end

function values = angleRateToRadps(values, unit)
unit = normalizeHeader(unit);
if strcmp(unit, 'degs') || strcmp(unit, 'degsec') || strcmp(unit, 'degrees') || ...
        strcmp(unit, 'degps')
    values = values * pi / 180;
end
end

function value = percentile(values, pct)
values = sort(values(isfinite(values)));
if isempty(values)
    value = NaN;
    return;
end
rank = 1 + (numel(values) - 1) * pct / 100;
lo = floor(rank);
hi = ceil(rank);
if lo == hi
    value = values(lo);
else
    value = values(lo) * (hi - rank) + values(hi) * (rank - lo);
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

function track = loadDiagnosticTrack(trackSpec, repoRoot)
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


try
    values = track.getSurfaceFriction();
catch err
    % Fall back to the caller's default mu, but surface the failure instead
    % of swallowing it: getSurfaceFriction is part of the Track interface, so
    % an error here signals a broken track object, not missing metadata.
    warning('investigate_lateral_g:SurfaceFrictionUnavailable', ...
        'Could not read track surface friction; keeping default mu. Cause: %s', ...
        err.message);
    return;
end

values = values(:);
values = values(isfinite(values) & values > 0);
if ~isempty(values)
    mu = values(1);
end
end
