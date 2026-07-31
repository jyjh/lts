function result = predict_design_change(varargin)
%PREDICT_DESIGN_CHANGE Governed paired baseline/variant laptime study.

root = lts.util.repoRoot(mfilename('fullpath'));
parser = inputParser;
parser.addParameter('VehicleConfig', @lts.vehicles.R25, ...
    @(x) isa(x, 'function_handle') || isa(x, 'lts.vehicle.VehicleConfig'));
parser.addParameter('Manifest', fullfile(root, 'config', 'governance', ...
    'r25_parameter_manifest.json'), @(x) ischar(x) || isstring(x));
parser.addParameter('CalibrationArtifact', fullfile(root, 'config', ...
    'governance', 'r25_provisional_calibration.json'), ...
    @(x) ischar(x) || isstring(x));
parser.addParameter('Change', '', @(x) ischar(x) || isstring(x) || isstruct(x));
parser.addParameter('Track', '2026enduro');
parser.addParameter('Scenario', struct(), @isstruct);
parser.addParameter('SampleCount', 0, @isnumeric);
parser.addParameter('Seed', 25, @isnumeric);
parser.addParameter('AllowProvisional', false, ...
    @(x) islogical(x) || isnumeric(x));
parser.addParameter('OptimizerOptions', struct(), @isstruct);
parser.parse(varargin{:});
opts = parser.Results;
if isempty(opts.Change)
    error('predict_design_change:MissingChange', ...
        'Pass a design-change struct or JSON file with Change.');
end

governed = lts.governance.GovernedVehicle.build(opts.VehicleConfig, ...
    opts.Manifest, opts.CalibrationArtifact, 'Scenario', opts.Scenario);
if ischar(opts.Change) || isstring(opts.Change)
    change = lts.prediction.DesignChange.load(opts.Change);
else
    change = lts.prediction.DesignChange.validate(opts.Change);
end
track = lts.correlation.CorrelationAppSupport.loadTrack(opts.Track, root);
result = lts.prediction.DesignStudy.run(governed, change, track, ...
    'SampleCount', opts.SampleCount, 'Seed', opts.Seed, ...
    'AllowProvisional', opts.AllowProvisional, ...
    'OptimizerOptions', opts.OptimizerOptions);
end
