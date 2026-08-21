function tf = tireDataAvailable()
%TIREDATAAVAILABLE True when the TTC tire data files are present locally.
%   The .tir files under src/+lts/+components/+Tire are untracked (FSAE
%   Tire Test Consortium member data; see that folder's README.md).
%   Tire-dependent tests use this to skip cleanly when the files are
%   absent, e.g. on a fresh clone or CI runner.
tireFolder = fullfile(fileparts(mfilename('fullpath')), ...
    '..', 'src', '+lts', '+components', '+Tire');
required = { ...
    '43105_18x7.5_10_R25B_7.tir', ...
    'Hoosier 43100 18.0x6.0-10 R20_7 - Scaled.tir'};
tf = all(cellfun(@(name) isfile(fullfile(tireFolder, name)), required));
end
