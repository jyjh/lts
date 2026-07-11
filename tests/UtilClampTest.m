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
