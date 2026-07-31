function output = calibrate_plant(varargin)
%CALIBRATE_PLANT Run governed, staged, whole-run plant identification.
%
% Stages are structs consumed by lts.calibration.StagedCalibrator. Residual
% callbacks must evaluate complete physical experiments or explicitly
% selected excitation segments; dataset assignment remains whole-run based.

root = lts.util.repoRoot(mfilename('fullpath'));
parser = inputParser;
parser.addParameter('VehicleConfig', @lts.vehicles.R25, ...
    @(x) isa(x, 'function_handle') || isa(x, 'lts.vehicle.VehicleConfig'));
parser.addParameter('Manifest', fullfile(root, 'config', 'governance', ...
    'r25_parameter_manifest.json'), @(x) ischar(x) || isstring(x) || isstruct(x));
parser.addParameter('DatasetCatalog', fullfile(root, 'config', 'governance', ...
    'r25_dataset_catalog.json'), @(x) ischar(x) || isstring(x) || isstruct(x));
parser.addParameter('DatasetIds', strings(0, 1), ...
    @(x) ischar(x) || isstring(x) || iscellstr(x));
parser.addParameter('Stages', struct([]), @(x) isstruct(x) || iscell(x));
parser.addParameter('ArtifactId', 'r25-provisional-calibration', ...
    @(x) ischar(x) || isstring(x));
parser.addParameter('OutputFile', '', @(x) ischar(x) || isstring(x));
parser.addParameter('MaxIterations', 100, @isnumeric);
parser.parse(varargin{:});
opts = parser.Results;
if isempty(opts.Stages)
    error('calibrate_plant:MissingStages', ...
        'Pass at least one physically motivated calibration stage.');
end

if ischar(opts.Manifest) || isstring(opts.Manifest)
    manifest = lts.governance.ParameterManifest.load(opts.Manifest);
else
    manifest = lts.governance.ParameterManifest.validate(opts.Manifest);
end
if ischar(opts.DatasetCatalog) || isstring(opts.DatasetCatalog)
    catalog = lts.governance.DatasetCatalog.load(opts.DatasetCatalog);
else
    catalog = lts.governance.DatasetCatalog.validate(opts.DatasetCatalog);
end
datasetIds = string(opts.DatasetIds(:));
if isempty(datasetIds)
    error('calibrate_plant:MissingDatasets', ...
        'Calibration requires explicit whole-run DatasetIds from the catalog.');
end
catalogIds = string({catalog.datasets.id});
for i = 1:numel(datasetIds)
    idx = find(catalogIds == datasetIds(i), 1);
    if isempty(idx)
        error('calibrate_plant:UnknownDataset', ...
            'Dataset "%s" is not in the catalog.', datasetIds(i));
    end
    if string(catalog.datasets(idx).role) ~= "calibration"
        error('calibrate_plant:DatasetLeakage', ...
            'Dataset "%s" has role "%s" and cannot influence calibration.', ...
            datasetIds(i), catalog.datasets(idx).role);
    end
end
lts.governance.DatasetCatalog.verifySources(catalog, root);

if isa(opts.VehicleConfig, 'function_handle')
    config = opts.VehicleConfig();
else
    config = opts.VehicleConfig;
end
output = lts.calibration.StagedCalibrator.fit( ...
    config, manifest, opts.Stages, ...
    'ArtifactId', opts.ArtifactId, ...
    'MaxIterations', opts.MaxIterations);
output.artifact.sourceDatasetIds = datasetIds;
output.artifact.provenance = sprintf( ...
    'Governed staged calibration from cataloged whole runs: %s', ...
    strjoin(datasetIds, ', '));
output.artifact = lts.governance.CalibrationArtifact.validate( ...
    output.artifact, manifest);
if ~isempty(opts.OutputFile)
    fid = fopen(opts.OutputFile, 'w');
    if fid < 0
        error('calibrate_plant:WriteFailed', ...
            'Could not write calibration artifact: %s', opts.OutputFile);
    end
    cleanup = onCleanup(@() fclose(fid));
    fprintf(fid, '%s\n', jsonencode(output.artifact, 'PrettyPrint', true));
end
end
