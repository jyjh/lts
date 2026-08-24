function tests = UtilClampTest
tests = functiontests(localfunctions);
end

function testClampConstrainsScalar(testCase)
verifyEqual(testCase, lts.util.clamp(5, 0, 1), 1);
verifyEqual(testCase, lts.util.clamp(-3, 0, 1), 0);
verifyEqual(testCase, lts.util.clamp(0.4, 0, 1), 0.4);
end

function testClampIsElementwise(testCase)
verifyEqual(testCase, lts.util.clamp([-2, 0.5, 3], -1, 1), [-1, 0.5, 1]);
end

function testSaturateMatchesUnitClamp(testCase)
verifyEqual(testCase, lts.util.saturate(1.7), 1);
verifyEqual(testCase, lts.util.saturate(-0.2), 0);
verifyEqual(testCase, lts.util.saturate([2, -1, 0.25]), [1, 0, 0.25]);
end

function testSaturatePreservesLegacyNaNBehavior(testCase)
% max(0, min(1, NaN)) evaluates to 1 because min/max ignore NaN; the shared
% helper must keep that behavior so existing call sites are unchanged.
verifyEqual(testCase, lts.util.saturate(NaN), max(0, min(1, NaN)));
end

function testLoadMatSafeRejectsObjectVariables(testCase)
% C1 regression: a .mat containing a saved object must be rejected before
% load(), because load() reconstructs objects by running their class
% constructor / loadobj, which executes arbitrary code. Only plain data is
% permitted. Build a file with a value object and confirm it is rejected.
root = lts.util.repoRoot(mfilename('fullpath'));
tmpDir = fullfile(root, 'exports', 'tmp_matsafe_test');
mkdir(tmpDir);
cleanup = onCleanup(@() localRmdirRecursive(tmpDir));

objFile = fullfile(tmpDir, 'has_object.mat');
% Save a string scalar (plain data) under a name alongside a simple value
% object. containers.Map is a value class available in base MATLAB.
m = containers.Map({'a'}, {1});
plain = [1 2 3];
save(objFile, 'm', 'plain');

verifyError(testCase, @() lts.util.loadMatSafe(objFile, 'test'), ...
    'lts_test_loadMatSafe:ObjectVariable');

% A plain-data file must load successfully.
plainFile = fullfile(tmpDir, 'plain.mat');
save(plainFile, 'plain');
data = lts.util.loadMatSafe(plainFile, 'test');
verifyEqual(testCase, data.plain, [1 2 3]);
end

function testValidatePythonCommandRejectsMetacharacters(testCase)
% C4 regression: a PythonCommand containing shell metacharacters must be
% rejected, so placing it unquoted at the head of a system() string cannot
% inject commands ('python & calc', 'python;rm -rf').
verifyEqual(testCase, lts.util.validatePythonCommand('python'), 'python');
verifyEqual(testCase, lts.util.validatePythonCommand('python3'), 'python3');
verifyError(testCase, @() lts.util.validatePythonCommand('python & calc'), ...
    'lts_util_validatePythonCommand:InvalidPythonCommand');
verifyError(testCase, @() lts.util.validatePythonCommand('python;rm'), ...
    'lts_util_validatePythonCommand:InvalidPythonCommand');
verifyError(testCase, @() lts.util.validatePythonCommand('python`whoami`'), ...
    'lts_util_validatePythonCommand:InvalidPythonCommand');
verifyError(testCase, @() lts.util.validatePythonCommand(''), ...
    'lts_util_validatePythonCommand:InvalidPythonCommand');
end

function testShellQuoteEscapesShellMetacharacters(testCase)
% C4 regression: shellQuote must neutralize cmd.exe / POSIX metacharacters
% so a free-text field cannot break out of its quoted argument. Quoting is
% shell-specific: cmd.exe double quotes with '^' escapes; POSIX wraps in
% single quotes.
if ispc
    plain = lts.util.shellQuote('hello');
    verifyEqual(testCase, plain(1), '"');
    verifyEqual(testCase, plain(end), '"');
    % On Windows the raw metacharacter must be escaped (^$ / ^&), not bare.
    dollar = lts.util.shellQuote('a$b');
    verifyTrue(testCase, contains(dollar, '^$'));
    verifyFalse(testCase, contains(dollar, 'a$b'));
    amp = lts.util.shellQuote('a&b');
    verifyTrue(testCase, contains(amp, '^&'));
    verifyFalse(testCase, contains(amp, 'a&b'));
    pipe = lts.util.shellQuote('a|b');
    verifyTrue(testCase, contains(pipe, '^|'));
else
    % On POSIX, single-quoting neutralizes metacharacters: the value sits
    % verbatim inside single quotes, so metacharacters stay inert.
    plain = lts.util.shellQuote('hello');
    verifyEqual(testCase, plain(1), '''');
    verifyEqual(testCase, plain(end), '''');
    amp = lts.util.shellQuote('a&b');
    verifyEqual(testCase, amp(1), '''');
    verifyEqual(testCase, amp(end), '''');
    verifyTrue(testCase, contains(amp, 'a&b'));
end
end

function localRmdirRecursive(dir)
if exist(dir, 'dir')
    rmdir(dir, 's');
end
end
