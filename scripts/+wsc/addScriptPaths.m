function addScriptPaths()
%ADDSCRIPTPATHS Put scripts/ (+wsc) and src/ (+lts) on the MATLAB path.
wscDir = fileparts(mfilename('fullpath'));
scriptsDir = fileparts(wscDir);
repoRoot = fileparts(scriptsDir);
addpath(scriptsDir);
addpath(fullfile(repoRoot, 'src'));
end
