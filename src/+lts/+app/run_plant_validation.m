function report = run_plant_validation(varargin)
%RUN_PLANT_VALIDATION Validate one complete replay from one initial state.

root = lts.util.repoRoot(mfilename('fullpath'));
parser = inputParser;
parser.addParameter('ReplayCsv', '', @(x) ischar(x) || isstring(x));
parser.addParameter('Manifest', fullfile(root, 'config', 'governance', ...
    'r25_parameter_manifest.json'), @(x) ischar(x) || isstring(x));
parser.addParameter('CalibrationArtifact', fullfile(root, 'config', ...
    'governance', 'r25_provisional_calibration.json'), ...
    @(x) ischar(x) || isstring(x));
parser.addParameter('Track', '2026enduro');
parser.addParameter('Scenario', struct(), @isstruct);
parser.addParameter('Dt', 0.001, @isnumeric);
parser.addParameter('PreferGpsKinematics', true, ...
    @(x) islogical(x) || isnumeric(x));
parser.addParameter('GpsSmoothingS', 0.35, @isnumeric);
parser.parse(varargin{:});
opts = parser.Results;
if isempty(opts.ReplayCsv) || ~exist(opts.ReplayCsv, 'file')
    error('run_plant_validation:MissingReplay', ...
        'ReplayCsv must identify an existing normalized replay.');
end

governed = lts.governance.GovernedVehicle.build(@lts.vehicles.R25, ...
    opts.Manifest, opts.CalibrationArtifact, 'Scenario', opts.Scenario);
profile = lts.correlation.CorrelationReplayProfile.fromCsv(opts.ReplayCsv);
[profile, preprocessing] = ...
    lts.correlation.CorrelationTuningEvaluator.prepareProfile( ...
        profile, governed.config, ...
        'PreferGpsKinematics', opts.PreferGpsKinematics, ...
        'GpsSmoothingS', opts.GpsSmoothingS);
track = lts.correlation.CorrelationAppSupport.loadTrack(opts.Track, root);
report = lts.validation.PlantValidator.validate( ...
    profile, governed, track, 'Dt', opts.Dt);
report.preprocessing = preprocessing;
end
