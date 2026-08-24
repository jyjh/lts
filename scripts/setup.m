% setup.m — one-command bootstrap for the lts repository.
%
% Initializes exactly the submodules the test suites need and prints the
% MATLAB commands to run them. The private external/LTSTelemetryVisualizer
% is deliberately skipped so the bootstrap works for anyone without team
% access. Requires git on PATH.
%
% Run from anywhere:  run('<repo>/scripts/setup.m')

here = fileparts(mfilename('fullpath'));   % .../scripts
root = fileparts(here);                    % repository root
old = cd(root);  % git submodule pathspecs are root-relative
cleanup = onCleanup(@() cd(old));

fprintf('Initializing submodules in %s ...\n', root);
[status, cmdout] = system([ ...
    'git submodule update --init ' ...
    'src/+lts/+util ' ...
    'src/+lts/+components/+Aero ' ...
    'src/+lts/+components/+Suspension ' ...
    'src/+lts/+components/+Powertrain ' ...
    'src/+lts/+components/+Chassis ' ...
    'external/MotecLogGenerator']);
fprintf('%s', cmdout);
if status ~= 0
    error('lts:setup:submodulesFailed', ...
        'git submodule update failed with exit code %d. Is git installed and on PATH?', ...
        status);
end

fprintf('\nDone. To run the tests in MATLAB:\n\n');
fprintf('    addpath(''%s'', ''%s'')\n', ...
    fullfile(root, 'src'), fullfile(root, 'third-party', 'mfeval'));
fprintf('    addpath(''%s'')  %% optional: analysis scripts\n', fullfile(root, 'scripts'));
fprintf('    run_audit_tests\n\n');
fprintf('Tire .tir data files are not committed (licensing); tests that need them skip.\n');
