function tests = DriverInputPlannerTest
tests = functiontests(localfunctions);
end

function testConstantSpeedProfileCoastsAtTarget(testCase)
planner = DriverInputPlanner([], 0.6);
profile = createConstantSpeedProfile(1, 0);

input = planner.sampleAtProgress(profile, 0.5, 10.0);

verifyEqual(testCase, input.throttle, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, input.brake, 0, 'AbsTol', 1e-12);
end

function testUnderspeedUsesPartialThrottleWithoutBrake(testCase)
planner = DriverInputPlanner([], 0.6);
profile = createConstantSpeedProfile(0, 0);

input = planner.sampleAtProgress(profile, 0.5, 9.5);

verifyGreaterThan(testCase, input.throttle, 0);
verifyLessThan(testCase, input.throttle, 1);
verifyEqual(testCase, input.brake, 0, 'AbsTol', 1e-12);
end

function testOverspeedUsesBrakeAndClearsThrottle(testCase)
planner = DriverInputPlanner([], 0.6);
profile = createConstantSpeedProfile(1, 0);

input = planner.sampleAtProgress(profile, 0.5, 10.7);

verifyEqual(testCase, input.throttle, 0, 'AbsTol', 1e-12);
verifyGreaterThan(testCase, input.brake, 0);
verifyLessThan(testCase, input.brake, 1);
end

function profile = createConstantSpeedProfile(throttle, brake)
profile = struct( ...
    's', [0; 1], ...
    'vTarget', [10; 10], ...
    'vLimit', [10; 10], ...
    'axRef', [0; 0], ...
    'throttle', throttle * ones(2, 1), ...
    'brake', brake * ones(2, 1), ...
    'steer', zeros(2, 1));
end
