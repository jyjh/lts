function run_audit_tests(varargin)
% Test runner for audit refactor: runs the given test files (or all tests
% when called with no args) and prints failures with diagnostics.
% Usage: run_audit_tests('ChassisLoadTransferTest.m', ...) or run_audit_tests()
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
cd(repoRoot);

if ~isempty(varargin)
    suites = cell(1, numel(varargin));
    for i = 1:numel(varargin)
        suites{i} = runtests(fullfile('tests', varargin{i}));
    end
    results = [suites{:}];
else
    results = runtests('tests');
end

failed = results([results.Failed]);
for i = 1:numel(failed)
    fprintf('\n===== FAILED: %s =====\n', failed(i).Name);
    try
        disp(failed(i).Details.Diagnostics);
    catch
        fprintf('(no diagnostics available)\n');
    end
end
fprintf('\nTotals: %d passed, %d failed, %d incomplete\n', ...
    nnz([results.Passed]), nnz([results.Failed]), nnz([results.Incomplete]));
if any([results.Failed])
    error('run_audit_tests:Failures', '%d test(s) failed', nnz([results.Failed]));
end
end
