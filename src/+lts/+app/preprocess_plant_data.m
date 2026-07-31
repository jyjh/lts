function outputs = preprocess_plant_data(varargin)
%PREPROCESS_PLANT_DATA Extract and hash one whole-run plant dataset.

root = lts.util.repoRoot(mfilename('fullpath'));
defaultMap = fullfile(root, 'config', 'motec', 'r25_real_channel_map.json');
parser = inputParser;
parser.addParameter('MoTeCFile', '', @(x) ischar(x) || isstring(x));
parser.addParameter('Lap', [], @(x) isempty(x) || isnumeric(x) || ischar(x) || isstring(x));
parser.addParameter('LdxFile', '', @(x) ischar(x) || isstring(x));
parser.addParameter('ChannelMap', defaultMap, @(x) ischar(x) || isstring(x));
parser.addParameter('ImportFrequency', [], @isnumeric);
parser.addParameter('OutputBase', '', @(x) ischar(x) || isstring(x));
parser.addParameter('PythonCommand', 'python', @(x) ischar(x) || isstring(x));
parser.parse(varargin{:});
opts = parser.Results;
if isempty(opts.MoTeCFile) || ~exist(opts.MoTeCFile, 'file')
    error('preprocess_plant_data:MissingMoTeCFile', ...
        'MoTeCFile must identify an existing log.');
end
if isempty(opts.OutputBase)
    [~, name] = fileparts(opts.MoTeCFile);
    opts.OutputBase = fullfile(root, 'exports', 'plant_data', name);
end
folder = fileparts(opts.OutputBase);
if ~exist(folder, 'dir')
    mkdir(folder);
end
replayCsv = [char(opts.OutputBase) '_replay.csv'];
manifestFile = [char(opts.OutputBase) '_extract_manifest.json'];
lts.correlation.CorrelationAppSupport.extractMoTeCLap( ...
    opts, replayCsv, manifestFile, root);
outputs = struct('replayCsv', replayCsv, ...
    'extractManifest', manifestFile, ...
    'splitPolicy', "whole-run-only", ...
    'note', "Assign complete runs to calibration or validation in the dataset catalog.");
end
