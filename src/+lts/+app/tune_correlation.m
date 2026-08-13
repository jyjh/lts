function outputs = tune_correlation(varargin)
%TUNE_CORRELATION Surrogate-assisted physical calibration from replay windows.
%
% outputs = lts.app.tune_correlation( ...
%   'MoTeCFile', 'data/lap5_raw.ld')

repoRoot = lts.util.repoRoot(mfilename('fullpath'));
defaultChannelMap = fullfile(repoRoot, 'config', 'motec', ...
    'r25_real_channel_map.json');
if ~exist(defaultChannelMap, 'file')
    defaultChannelMap = fullfile(repoRoot, 'config', 'motec', ...
        'default_channel_map.json');
end
parser = inputParser;
parser.addParameter('ReplayCsv', '', @(x) ischar(x) || isstring(x));
parser.addParameter('MoTeCFile', '', @(x) ischar(x) || isstring(x));
parser.addParameter('Lap', [], ...
    @(x) isempty(x) || isnumeric(x) || ischar(x) || isstring(x));
parser.addParameter('LdxFile', '', @(x) ischar(x) || isstring(x));
parser.addParameter('ChannelMap', defaultChannelMap, ...
    @(x) ischar(x) || isstring(x));
parser.addParameter('ImportFrequency', [], ...
    @(x) isempty(x) || (isnumeric(x) && isscalar(x) && isfinite(x) && x > 0));
parser.addParameter('ParameterSpace', fullfile(repoRoot, 'config', ...
    'correlation', 'lap5_ml_parameter_space.json'), ...
    @(x) ischar(x) || isstring(x));
parser.addParameter('Track', '2026enduro');
parser.addParameter('HorizonS', [3 6 12], ...
    @(x) isnumeric(x) && isvector(x) && ~isempty(x) && ...
    all(isfinite(x)) && all(x > 0));
parser.addParameter('PreferGpsKinematics', true, ...
    @(x) islogical(x) || isnumeric(x));
parser.addParameter('GpsSmoothingS', 0.35, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0);
parser.addParameter('ExcludeInitialS', 0.1, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0);
parser.addParameter('MaxCandidates', 1200, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 1);
parser.addParameter('MaxHours', 8, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0);
parser.addParameter('Workers', 0, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0);
parser.addParameter('Seed', 25, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x));
parser.addParameter('CheckpointDirectory', '', ...
    @(x) ischar(x) || isstring(x));
parser.addParameter('Resume', true, @(x) islogical(x) || isnumeric(x));
parser.addParameter('BatchSize', 0, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0);
parser.addParameter('InitialCandidates', 256, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 1);
parser.addParameter('ValidationFinalists', 10, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 1);
parser.addParameter('Dt', 0.001, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0);
parser.addParameter('WheelSolveIterations', 2, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 1);
parser.addParameter('PythonCommand', 'python', ...
    @(x) ischar(x) || isstring(x));
parser.addParameter('RunFullLap', true, @(x) islogical(x) || isnumeric(x));
parser.addParameter('GeneratePlots', true, @(x) islogical(x) || isnumeric(x));
parser.parse(varargin{:});
opts = parser.Results;
opts.EvaluationSplit = "train";

opts.MaxCandidates = round(opts.MaxCandidates);
opts.InitialCandidates = min(round(opts.InitialCandidates), opts.MaxCandidates);
opts.ValidationFinalists = round(opts.ValidationFinalists);
opts.WheelSolveIterations = round(opts.WheelSolveIterations);
opts.HorizonS = unique(sort(double(opts.HorizonS(:).')));
if opts.ExcludeInitialS >= min(opts.HorizonS)
    error('tune_correlation:InvalidExclusion', ...
        'ExcludeInitialS must be smaller than every prediction horizon.');
end
opts.ParameterSpace = char(opts.ParameterSpace);
if isempty(opts.CheckpointDirectory)
    stamp = datestr(now, 'yyyymmdd_HHMMSS');
    opts.CheckpointDirectory = fullfile(repoRoot, 'exports', ...
        ['correlation_tuning_lap5_' stamp]);
end
checkpointDir = char(opts.CheckpointDirectory);
if ~exist(checkpointDir, 'dir')
    mkdir(checkpointDir);
elseif ~logical(opts.Resume) && localHasCheckpoint(checkpointDir)
    error('tune_correlation:CheckpointExists', ...
        'Checkpoint directory is not empty and Resume is false: %s', checkpointDir);
end
[opts.ReplayCsv, extractionManifest, inputDescriptor] = ...
    localResolveReplayInput(opts, checkpointDir, repoRoot);

registry = lts.correlation.CorrelationParameterRegistry.load(opts.ParameterSpace);
parameterNames = cellstr(lts.correlation.CorrelationParameterRegistry.names(registry));
baseConfig = lts.vehicles.R25_correlation_tuning(lts.vehicles.R25());
profile = lts.correlation.CorrelationReplayProfile.fromCsv(opts.ReplayCsv);
[profile, preprocessing] = ...
    lts.correlation.CorrelationTuningEvaluator.prepareProfile(profile, baseConfig, ...
    'PreferGpsKinematics', opts.PreferGpsKinematics, ...
    'GpsSmoothingS', opts.GpsSmoothingS);
windows = lts.correlation.CorrelationTuningEvaluator.makeWindows( ...
    profile, opts.HorizonS);
if ~any(windows.split == "train") || ~any(windows.split == "validation")
    error('tune_correlation:InsufficientWindows', ...
        ['Replay must contain at least two complete anchor blocks at the ' ...
        'largest prediction horizon (%.3g seconds).'], max(opts.HorizonS));
end
windowsFile = fullfile(checkpointDir, 'windows.csv');
localValidateOrWriteWindows(windows, windowsFile, checkpointDir, opts.Resume);
localValidateOrWriteTuningDefinition(opts, checkpointDir);
track = lts.correlation.CorrelationAppSupport.loadTrack(opts.Track, repoRoot);

workers = localResolveWorkers(opts.Workers);
batchSize = round(opts.BatchSize);
if batchSize == 0
    batchSize = max(1, 2 * workers);
end
localEnsurePool(workers);

historyFile = fullfile(checkpointDir, 'candidate_scores.csv');
detailFile = fullfile(checkpointDir, 'window_scores.csv');
validationFile = fullfile(checkpointDir, 'validation_scores.csv');
validationDetailFile = fullfile(checkpointDir, 'validation_window_scores.csv');
surrogateScript = fullfile(repoRoot, 'scripts', 'correlation_surrogate.py');
initialFile = fullfile(checkpointDir, 'initial_candidates.csv');
metadataFile = fullfile(checkpointDir, 'surrogate_metadata.json');
if exist(historyFile, 'file') && logical(opts.Resume)
    history = readtable(historyFile, 'VariableNamingRule', 'preserve');
else
    history = table();
end
if ~exist(initialFile, 'file')
    localRunPython(opts.PythonCommand, surrogateScript, { ...
        'initial', '--space', opts.ParameterSpace, ...
        '--count', num2str(opts.InitialCandidates), ...
        '--seed', num2str(round(opts.Seed)), ...
        '--output', initialFile});
end
initialCandidates = readtable(initialFile, 'VariableNamingRule', 'preserve');
pending = localRemoveCompleted(initialCandidates, history);

started = tic;
batchIndex = 0;
while height(history) < opts.MaxCandidates && toc(started) < opts.MaxHours * 3600
    remaining = opts.MaxCandidates - height(history);
    if isempty(pending)
        if height(history) < 2
            error('tune_correlation:InsufficientHistory', ...
                'Surrogate proposal requires at least two completed candidates.');
        end
        batchIndex = batchIndex + 1;
        proposalFile = fullfile(checkpointDir, ...
            sprintf('proposed_candidates_%04d.csv', batchIndex));
        localRunPython(opts.PythonCommand, surrogateScript, { ...
            'propose', '--space', opts.ParameterSpace, ...
            '--history', historyFile, ...
            '--count', num2str(min(batchSize, remaining)), ...
            '--seed', num2str(round(opts.Seed + batchIndex)), ...
            '--output', proposalFile, ...
            '--metadata', metadataFile});
        pending = readtable(proposalFile, 'VariableNamingRule', 'preserve');
    end

    take = min([batchSize, remaining, height(pending)]);
    batch = pending(1:take, :);
    pending(1:take, :) = [];
    [summaries, details] = localEvaluateBatch(batch, parameterNames, ...
        registry, baseConfig, profile, track, windows, opts);
    candidateRows = batch(:, [{'candidate_id'}, parameterNames]);
    batchHistory = [candidateRows, summaries(:, 2:end)];
    history = [history; batchHistory]; %#ok<AGROW>
    writetable(history, historyFile);
    localAppendTable(details, detailFile);
    fprintf('Correlation tuning: %d/%d candidates, best training score %.6g\n', ...
        height(history), opts.MaxCandidates, min(history.score));
end

finite = isfinite(history.score);
ranked = sortrows(history(finite, :), 'score', 'ascend');
if isempty(ranked)
    error('tune_correlation:NoFiniteCandidates', ...
        'No candidate completed with a finite training score.');
end
finalistCount = min(opts.ValidationFinalists, height(ranked));
finalists = ranked(1:finalistCount, :);
baselineRow = history(history.candidate_id == 1, :);
if ~isempty(baselineRow) && ~any(finalists.candidate_id == 1)
    finalists = [finalists; baselineRow(1, :)]; %#ok<AGROW>
end
[validationSummary, validationDetail] = localEvaluateBatch( ...
    finalists(:, [{'candidate_id'}, parameterNames]), parameterNames, ...
    registry, baseConfig, profile, track, windows, ...
    localWithSplit(opts, "validation"));
validation = [finalists(:, [{'candidate_id'}, parameterNames, {'score'}]), ...
    renamevars(validationSummary(:, 2:end), 'score', 'validation_score')];
writetable(validation, validationFile);
writetable(validationDetail, validationDetailFile);
validation = sortrows(validation, 'validation_score', 'ascend');
winner = validation(1, :);
winnerValues = table2array(winner(1, parameterNames));

fullLap = table();
if logical(opts.RunFullLap)
    fullWindows = table(1, 0, profile.duration(), "full", ...
        'VariableNames', {'window_id', 'start_s', 'horizon_s', 'split'});
    [fullLap, fullDetail] = ...
        lts.correlation.CorrelationTuningEvaluator.evaluateCandidate( ...
        winner.candidate_id, winnerValues, registry, baseConfig, ...
        profile, track, fullWindows, ...
        'Dt', opts.Dt, 'ExcludeInitialS', opts.ExcludeInitialS, ...
        'Split', "full", ...
        'WheelSolveIterations', opts.WheelSolveIterations);
    writetable(fullDetail, fullfile(checkpointDir, 'full_lap_score.csv'));
end

overlayFile = fullfile(checkpointDir, 'R25_ml_lap5_tuning.m');
localWriteOverlay(overlayFile, registry, winnerValues);
acceptance = localAcceptance(validation, validationDetail);
manifestFile = fullfile(checkpointDir, 'result_manifest.json');
manifest = localManifest(opts, registry, windows, history, validation, ...
    winner, fullLap, preprocessing, acceptance, overlayFile, ...
    extractionManifest, inputDescriptor);
localWriteJson(manifestFile, manifest);
if logical(opts.GeneratePlots)
    localGeneratePlots(checkpointDir, history, validation, validationDetail, ...
        registry, metadataFile);
end

outputs = struct( ...
    'checkpointDirectory', checkpointDir, ...
    'replayCsv', opts.ReplayCsv, ...
    'extractionManifest', extractionManifest, ...
    'historyFile', historyFile, ...
    'validationFile', validationFile, ...
    'manifestFile', manifestFile, ...
    'overlayFile', overlayFile, ...
    'winnerCandidateId', winner.candidate_id, ...
    'winnerTrainingScore', winner.score, ...
    'winnerValidationScore', winner.validation_score, ...
    'acceptancePassed', acceptance.passed, ...
    'candidateCount', height(history), ...
    'elapsedHours', toc(started) / 3600);
end

function [replayCsv, extractionManifest, descriptor] = ...
        localResolveReplayInput(opts, checkpointDir, repoRoot)
hasReplay = ~isempty(opts.ReplayCsv);
hasMotec = ~isempty(opts.MoTeCFile);
if hasReplay && hasMotec
    error('tune_correlation:AmbiguousInput', ...
        'Provide ReplayCsv or MoTeCFile, not both.');
end
if ~hasMotec
    if ~hasReplay
        replayCsv = localLatestReplay(repoRoot);
    else
        replayCsv = char(opts.ReplayCsv);
    end
    if ~exist(replayCsv, 'file')
        error('tune_correlation:MissingReplay', ...
            'Replay CSV does not exist: %s', replayCsv);
    end
    replayCsv = localCanonicalPath(replayCsv);
    extractionManifest = '';
    descriptor = struct( ...
        'kind', 'replay_csv', ...
        'sourceFile', replayCsv, ...
        'sourceSha256', localSha256(replayCsv));
    descriptorFile = fullfile(checkpointDir, 'input_source.json');
    if exist(descriptorFile, 'file')
        previous = jsondecode(fileread(descriptorFile));
        if ~strcmp(jsonencode(previous), jsonencode(descriptor))
            error('tune_correlation:InputChanged', ...
                ['Replay CSV differs from this checkpoint. Use a new ' ...
                'CheckpointDirectory.']);
        end
    elseif logical(opts.Resume) && localHasCheckpoint(checkpointDir)
        error('tune_correlation:MissingInputDescriptor', ...
            ['Checkpoint contains tuning artifacts but no input_source.json; ' ...
            'refusing an unverifiable resume.']);
    else
        localWriteJson(descriptorFile, descriptor);
    end
    return;
end

motecFile = localCanonicalPath(char(opts.MoTeCFile));
if ~exist(motecFile, 'file')
    error('tune_correlation:MissingMoTeCFile', ...
        'MoTeC LD file does not exist: %s', motecFile);
end
channelMap = localCanonicalPath(char(opts.ChannelMap));
if ~exist(channelMap, 'file')
    error('tune_correlation:MissingChannelMap', ...
        'Channel map does not exist: %s', channelMap);
end
ldxFile = char(opts.LdxFile);
if isempty(ldxFile)
    [folder, name] = fileparts(motecFile);
    automaticLdx = fullfile(folder, [name '.ldx']);
    if exist(automaticLdx, 'file')
        ldxFile = automaticLdx;
    end
elseif ~exist(ldxFile, 'file')
    error('tune_correlation:MissingLdxFile', ...
        'MoTeC LDX file does not exist: %s', ldxFile);
end
if ~isempty(ldxFile)
    ldxFile = localCanonicalPath(ldxFile);
end

lapText = '';
if ~isempty(opts.Lap)
    lapText = lts.correlation.CorrelationAppSupport.lapValue(opts.Lap);
end
descriptor = struct( ...
    'kind', 'motec_ld', ...
    'sourceFile', motecFile, ...
    'sourceSha256', localSha256(motecFile), ...
    'channelMap', channelMap, ...
    'channelMapSha256', localSha256(channelMap), ...
    'lap', lapText, ...
    'ldxFile', ldxFile, ...
    'ldxSha256', localOptionalSha256(ldxFile), ...
    'importFrequencyHz', opts.ImportFrequency);
replayCsv = fullfile(checkpointDir, 'normalized_replay.csv');
extractionManifest = fullfile(checkpointDir, 'extraction_manifest.json');
descriptorFile = fullfile(checkpointDir, 'input_source.json');

if logical(opts.Resume) && exist(replayCsv, 'file')
    if ~exist(descriptorFile, 'file')
        error('tune_correlation:MissingInputDescriptor', ...
            ['Checkpoint contains normalized_replay.csv but no input_source.json; ' ...
            'refusing an unverifiable resume.']);
    end
    previous = jsondecode(fileread(descriptorFile));
    if ~strcmp(jsonencode(previous), jsonencode(descriptor))
        error('tune_correlation:InputChanged', ...
            ['MoTeC source or extraction settings differ from this checkpoint. ' ...
            'Use a new CheckpointDirectory.']);
    end
    return;
end

extractOpts = opts;
extractOpts.MoTeCFile = motecFile;
extractOpts.ChannelMap = channelMap;
extractOpts.LdxFile = ldxFile;
lts.correlation.CorrelationAppSupport.extractMoTeCLap( ...
    extractOpts, replayCsv, extractionManifest, repoRoot);
localWriteJson(descriptorFile, descriptor);
end

function path = localCanonicalPath(path)
file = javaObject('java.io.File', char(path));
path = char(file.getCanonicalPath());
end

function hash = localOptionalSha256(file)
if isempty(file)
    hash = '';
else
    hash = localSha256(file);
end
end

function [summaries, details] = localEvaluateBatch(batch, parameterNames, ...
        registry, baseConfig, profile, track, windows, opts)
n = height(batch);
summaryCells = cell(n, 1);
detailCells = cell(n, 1);
values = table2array(batch(:, parameterNames));
ids = batch.candidate_id;
pool = gcp('nocreate');
if isempty(pool) || n == 1
    for i = 1:n
        [summaryCells{i}, detailCells{i}] = localEvaluateOne( ...
            ids(i), values(i, :), registry, baseConfig, profile, ...
            track, windows, opts);
    end
else
    parfor i = 1:n
        [summaryCells{i}, detailCells{i}] = localEvaluateOne( ...
            ids(i), values(i, :), registry, baseConfig, profile, ...
            track, windows, opts);
    end
end
summaries = vertcat(summaryCells{:});
details = vertcat(detailCells{:});
end

function [summary, detail] = localEvaluateOne(id, values, registry, ...
        baseConfig, profile, track, windows, opts)
[summary, detail] = ...
    lts.correlation.CorrelationTuningEvaluator.evaluateCandidate( ...
    id, values, registry, baseConfig, profile, track, windows, ...
    'Dt', opts.Dt, ...
    'ExcludeInitialS', opts.ExcludeInitialS, ...
    'Split', opts.EvaluationSplit, ...
    'WheelSolveIterations', opts.WheelSolveIterations);
end

function opts = localWithSplit(opts, split)
opts.EvaluationSplit = string(split);
end

function replay = localLatestReplay(repoRoot)
files = dir(fullfile(repoRoot, 'exports', '*_replay.csv'));
if isempty(files)
    error('tune_correlation:MissingReplay', ...
        'No replay CSV was supplied and exports contains no *_replay.csv file.');
end
[~, order] = sort([files.datenum], 'descend');
replay = fullfile(files(order(1)).folder, files(order(1)).name);
end

function workers = localResolveWorkers(requested)
if requested > 0
    workers = max(1, round(requested));
else
    cluster = parcluster('local');
    workers = max(1, min(8, cluster.NumWorkers));
end
end

function localEnsurePool(workers)
pool = gcp('nocreate');
if workers <= 1
    if ~isempty(pool)
        delete(pool);
    end
    return;
end
if isempty(pool) || pool.NumWorkers ~= workers
    if ~isempty(pool)
        delete(pool);
    end
    parpool('Processes', workers);
end
end

function localRunPython(pythonCommand, script, args)
command = lts.util.shellJoin([{char(pythonCommand), script}, args]);
[status, output] = system(command);
if status ~= 0
    error('tune_correlation:PythonFailed', ...
        'Surrogate command failed (%d):\n%s\n%s', status, command, output);
end
end

function pending = localRemoveCompleted(candidates, history)
if isempty(history)
    pending = candidates;
else
    pending = candidates(~ismember(candidates.candidate_id, history.candidate_id), :);
end
end

function tf = localHasCheckpoint(folder)
files = dir(folder);
tf = any(~ismember({files.name}, {'.', '..'}));
end

function localValidateOrWriteWindows(windows, file, checkpointDir, resume)
historyFile = fullfile(checkpointDir, 'candidate_scores.csv');
if logical(resume) && exist(file, 'file') && exist(historyFile, 'file')
    previous = readtable(file, 'VariableNamingRule', 'preserve');
    sameSchema = isequal(previous.Properties.VariableNames, ...
        windows.Properties.VariableNames);
    if ~sameSchema || height(previous) ~= height(windows) || ...
            ~isequaln(table2cell(previous), table2cell(windows))
        error('tune_correlation:WindowDefinitionChanged', ...
            ['Prediction horizons or window splits differ from this checkpoint. ' ...
            'Use a new CheckpointDirectory.']);
    end
    return;
end
writetable(windows, file);
end

function localValidateOrWriteTuningDefinition(opts, checkpointDir)
definition = struct( ...
    'horizonS', opts.HorizonS, ...
    'excludeInitialS', opts.ExcludeInitialS, ...
    'preferGpsKinematics', logical(opts.PreferGpsKinematics), ...
    'gpsSmoothingS', opts.GpsSmoothingS, ...
    'dt', opts.Dt, ...
    'wheelSolveIterations', opts.WheelSolveIterations, ...
    'track', char(string(opts.Track)), ...
    'parameterSpaceSha256', localSha256(opts.ParameterSpace));
file = fullfile(checkpointDir, 'tuning_definition.json');
historyFile = fullfile(checkpointDir, 'candidate_scores.csv');
if exist(file, 'file')
    previous = jsondecode(fileread(file));
    if ~strcmp(jsonencode(previous), jsonencode(definition))
        error('tune_correlation:TuningDefinitionChanged', ...
            ['Horizons, GPS preprocessing, simulation settings, track, or ' ...
            'parameter space differ from this checkpoint. Use a new ' ...
            'CheckpointDirectory.']);
    end
elseif logical(opts.Resume) && exist(historyFile, 'file')
    error('tune_correlation:MissingTuningDefinition', ...
        ['Checkpoint contains candidate scores but no tuning_definition.json; ' ...
        'refusing an unverifiable resume.']);
else
    localWriteJson(file, definition);
end
end

function localAppendTable(T, file)
if isempty(T)
    return;
end
if exist(file, 'file')
    writetable(T, file, 'WriteMode', 'append', 'WriteVariableNames', false);
else
    writetable(T, file);
end
end

function localWriteOverlay(file, registry, values)
fid = fopen(file, 'w');
if fid < 0
    error('tune_correlation:OverlayWriteFailed', 'Could not write %s.', file);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, 'function cfg = R25_ml_lap5_tuning(cfg)\n');
fprintf(fid, '%%R25_ML_LAP5_TUNING Generated physical overlay; do not hand-edit.\n');
fprintf(fid, 'if nargin < 1 || isempty(cfg)\n');
fprintf(fid, '    cfg = lts.vehicles.R25();\n');
fprintf(fid, 'end\n');
fprintf(fid, 'cfg = lts.vehicles.R25_correlation_tuning(cfg);\n');
params = registry.parameters(:);
for i = 1:numel(params)
    path = char(params(i).path);
    if isfield(params(i), 'indices') && ~isempty(params(i).indices)
        indexText = strtrim(sprintf('%d ', params(i).indices));
        fprintf(fid, 'cfg.%s([%s]) = %.17g;\n', path, indexText, values(i));
    else
        fprintf(fid, 'cfg.%s = %.17g;\n', path, values(i));
    end
end
fprintf(fid, 'cfg.name = string(cfg.name) + "_mlLap5";\n');
fprintf(fid, 'end\n');
end

function acceptance = localAcceptance(validation, detail)
baseline = validation(validation.candidate_id == 1, :);
winner = validation(1, :);
if isempty(baseline) || ~isfinite(baseline.validation_score)
    acceptance = struct('passed', false, 'reason', 'baseline_unavailable', ...
        'heldOutImprovement', NaN, 'maximumGroupRegression', NaN);
    return;
end
improvement = 1 - winner.validation_score / baseline.validation_score;
groupNames = {'gps_trace', 'speed', 'yaw_rate', 'lateral_accel', ...
    'longitudinal_accel', 'wheel_speed'};
baseDetail = detail(detail.candidate_id == 1, :);
winnerDetail = detail(detail.candidate_id == winner.candidate_id, :);
regressions = nan(size(groupNames));
for i = 1:numel(groupNames)
    baseValue = mean(baseDetail.(groupNames{i}), 'omitnan');
    winnerValue = mean(winnerDetail.(groupNames{i}), 'omitnan');
    if isfinite(baseValue) && baseValue > 0 && isfinite(winnerValue)
        regressions(i) = winnerValue / baseValue - 1;
    end
end
maximumRegression = max(regressions, [], 'omitnan');
if isempty(maximumRegression) || ~isfinite(maximumRegression)
    maximumRegression = Inf;
end
passed = improvement >= 0.15 && maximumRegression <= 0.10 && ...
    winner.completed_windows == winner.total_windows;
acceptance = struct('passed', passed, 'reason', 'threshold_check', ...
    'heldOutImprovement', improvement, ...
    'maximumGroupRegression', maximumRegression);
end

function manifest = localManifest(opts, registry, windows, history, validation, ...
        winner, fullLap, preprocessing, acceptance, overlayFile, ...
        extractionManifest, inputDescriptor)
manifest = struct();
manifest.schema = 'lts.correlation.tuning-result.v2';
manifest.createdAt = datestr(now, 31);
manifest.replayCsv = opts.ReplayCsv;
manifest.replaySha256 = localSha256(opts.ReplayCsv);
manifest.input = inputDescriptor;
manifest.extractionManifest = extractionManifest;
manifest.parameterSpace = opts.ParameterSpace;
manifest.parameterSpaceSha256 = localSha256(opts.ParameterSpace);
manifest.seed = opts.Seed;
manifest.horizonS = opts.HorizonS;
manifest.gpsKinematicsPreferred = logical(opts.PreferGpsKinematics);
manifest.gpsSmoothingS = opts.GpsSmoothingS;
manifest.excludeInitialS = opts.ExcludeInitialS;
manifest.dt = opts.Dt;
manifest.trainingWindowCount = nnz(windows.split == "train");
manifest.validationWindowCount = nnz(windows.split == "validation");
manifest.candidateCount = height(history);
manifest.preprocessing = preprocessing;
manifest.parameters = registry.parameters;
manifest.winner = table2struct(winner);
manifest.acceptance = acceptance;
manifest.overlayFile = overlayFile;
if ~isempty(fullLap)
    manifest.fullLap = table2struct(fullLap);
else
    manifest.fullLap = struct();
end
manifest.validationRanking = table2struct(validation);
end

function hash = localSha256(file)
stream = java.io.FileInputStream(java.io.File(file));
cleanup = onCleanup(@() stream.close()); %#ok<NASGU>
digest = java.security.MessageDigest.getInstance('SHA-256');
buffer = zeros(1, 65536, 'int8');
while true
    count = stream.read(buffer, 0, numel(buffer));
    if count < 0
        break;
    end
    digest.update(buffer(1:count));
end
bytes = typecast(digest.digest(), 'uint8');
hash = lower(reshape(dec2hex(bytes, 2).', 1, []));
end

function localWriteJson(file, value)
fid = fopen(file, 'w');
if fid < 0
    error('tune_correlation:ManifestWriteFailed', 'Could not write %s.', file);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s\n', jsonencode(value, 'PrettyPrint', true));
end

function localGeneratePlots(folder, history, validation, validationDetail, ...
        registry, metadataFile)
fig = figure('Visible', 'off');
cleanup = onCleanup(@() close(fig)); %#ok<NASGU>
best = cummin(history.score);
plot(history.candidate_id, best, 'LineWidth', 1.5);
xlabel('Completed candidate'); ylabel('Best training score'); grid on;
title('Correlation tuning convergence');
exportgraphics(fig, fullfile(folder, 'convergence.png'));

clf(fig);
bar(categorical(string(validation.candidate_id)), validation.validation_score);
xlabel('Candidate'); ylabel('Held-out score'); grid on;
title('Finalist held-out validation');
exportgraphics(fig, fullfile(folder, 'validation_scores.png'));

baseline = validation(validation.candidate_id == 1, :);
winner = validation(1, :);
if ~isempty(baseline)
    groupNames = {'gps_trace', 'speed', 'yaw_rate', 'lateral_accel', ...
        'longitudinal_accel', 'wheel_speed'};
    groupLabels = {'GPS trace', 'Speed', 'Yaw rate', 'Lateral accel', ...
        'Longitudinal accel', 'Wheel speed'};
    grouped = nan(numel(groupNames), 2);
    ids = [baseline.candidate_id(1), winner.candidate_id];
    for column = 1:2
        rows = validationDetail(validationDetail.candidate_id == ids(column), :);
        for group = 1:numel(groupNames)
            grouped(group, column) = mean(rows.(groupNames{group}), 'omitnan');
        end
    end
    clf(fig);
    bar(categorical(groupLabels), grouped);
    ylabel('Held-out robust loss'); grid on;
    legend({'Baseline', 'Winner'}, 'Location', 'best');
    title('Held-out channel-group errors');
    exportgraphics(fig, fullfile(folder, 'channel_group_errors.png'));

    horizons = unique(validationDetail.horizon_s);
    horizonGrouped = nan(numel(horizons), 2);
    for column = 1:2
        rows = validationDetail(validationDetail.candidate_id == ids(column), :);
        for horizon = 1:numel(horizons)
            horizonGrouped(horizon, column) = mean( ...
                rows.score(rows.horizon_s == horizons(horizon)), 'omitnan');
        end
    end
    clf(fig);
    bar(categorical(compose('%g s', horizons)), horizonGrouped);
    ylabel('Held-out robust loss'); grid on;
    legend({'Baseline', 'Winner'}, 'Location', 'best');
    title('Held-out loss by prediction horizon');
    exportgraphics(fig, fullfile(folder, 'validation_horizon_scores.png'));
end

names = lts.correlation.CorrelationParameterRegistry.names(registry);
params = registry.parameters(:);
clf(fig);
layout = tiledlayout(fig, 5, 5, 'TileSpacing', 'compact', ...
    'Padding', 'compact');
for i = 1:numel(names)
    nexttile(layout);
    scatter(history.(char(names(i))), history.score, 8, ...
        history.candidate_id, 'filled');
    if lower(string(params(i).transform)) == "log"
        set(gca, 'XScale', 'log');
    end
    title(strrep(names(i), '_', ' '), 'FontSize', 7);
    grid on;
end
xlabel(layout, 'Physical parameter value');
ylabel(layout, 'Training score');
title(layout, 'Parameter versus score diagnostics');
exportgraphics(fig, fullfile(folder, 'parameter_score_diagnostics.png'), ...
    'Resolution', 160);

if exist(metadataFile, 'file')
    metadata = jsondecode(fileread(metadataFile));
    if isfield(metadata, 'featureImportances')
        importance = zeros(size(names));
        for i = 1:numel(names)
            importance(i) = metadata.featureImportances.(char(names(i)));
        end
        [importance, order] = sort(importance, 'descend');
        clf(fig);
        barh(categorical(names(order)), importance);
        xlabel('Extra Trees importance'); grid on;
        title('Surrogate parameter importance');
        exportgraphics(fig, fullfile(folder, 'parameter_importance.png'));
    end
end
end
