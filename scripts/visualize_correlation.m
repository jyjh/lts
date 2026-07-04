function varargout = visualize_correlation(varargin)
%VISUALIZE_CORRELATION Thin wrapper for the LTS telemetry visualization submodule.
%
% Example:
%   visualize_correlation( ...
%       'SimCsv', 'exports/correlation_run.csv', ...
%       'RealMoTeCFile', 'data/lap5_raw.ld', ...
%       'TrackFile', 'tracks/endurance_track_grid_25ft_from_matlab_smoothed.mat')

scriptDir = fileparts(mfilename('fullpath'));
repoRoot = fileparts(scriptDir);
vizRoot = fullfile(repoRoot, 'external', 'LTSTelemetryVisualizer');
plotlyRoot = fullfile(vizRoot, 'external', 'plotly_matlab');

if ~exist(vizRoot, 'dir')
    error('visualize_correlation:MissingSubmodule', ...
        ['Missing %s. Run "git submodule update --init --recursive" ' ...
         'after the LTSTelemetryVisualizer remote has been created.'], vizRoot);
end

addpath(fullfile(repoRoot, 'src'));
addpath(vizRoot);
if exist(plotlyRoot, 'dir')
    addpath(genpath(plotlyRoot));
end

result = ltsviz.visualizeCorrelation(varargin{:}, 'RepoRoot', repoRoot);
if nargout > 0
    varargout{1} = result;
end
end
