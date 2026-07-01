function tests = DriverInputPlannerTest_legacy
tests = functiontests(localfunctions);
end

function testConstantSpeedProfileCoastsAtTarget(testCase)
planner = DriverInputPlanner_legacy([], 0.6);
profile = createConstantSpeedProfile(1, 0);

input = planner.sampleAtProgress(profile, 0.5, 10.0);

verifyEqual(testCase, input.throttle, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, input.brake, 0, 'AbsTol', 1e-12);
end

function testUnderspeedUsesPartialThrottleWithoutBrake(testCase)
planner = DriverInputPlanner_legacy([], 0.6);
profile = createConstantSpeedProfile(0, 0);

input = planner.sampleAtProgress(profile, 0.5, 9.5);

verifyGreaterThan(testCase, input.throttle, 0);
verifyLessThan(testCase, input.throttle, 1);
verifyEqual(testCase, input.brake, 0, 'AbsTol', 1e-12);
end

function testOverspeedUsesBrakeAndClearsThrottle(testCase)
planner = DriverInputPlanner_legacy([], 0.6);
profile = createConstantSpeedProfile(1, 0);

input = planner.sampleAtProgress(profile, 0.5, 10.7);

verifyEqual(testCase, input.throttle, 0, 'AbsTol', 1e-12);
verifyGreaterThan(testCase, input.brake, 0);
verifyLessThan(testCase, input.brake, 1);
end

% ============================================================
% Physics-based pedal map (computePedals) unit tests.
% Pure function — no VehicleManager required. Each test pins one
% regime: WOT, partial/cruise throttle, true coast, gradual brake,
% full brake, and pedal exclusivity.
% ============================================================

function testComputePedalsWOTWhenForceSaturates(testCase)
% Required drive force meets/exceeds full-throttle capability -> WOT.
mass = 256;
F_drive_full = 3000;     % [N]
F_resistance = 300;      % drag + rolling [N]
axRef = 15;              % large positive -> F_req = 15*256 + 300 >> 3000
[throttle, brake] = DriverInputPlanner_legacy.computePedals( ...
    axRef, F_drive_full, F_resistance, mass, 10);
verifyEqual(testCase, throttle, 1, 'AbsTol', 1e-9);
verifyEqual(testCase, brake, 0, 'AbsTol', 1e-12);
end

function testComputePedalsPartialThrottleToMaintainSpeed(testCase)
% Speed-hold: axRef = 0 but drag must be overcome -> partial throttle.
mass = 256;
F_drive_full = 3000;
F_resistance = 300;
axRef = 0;
[throttle, brake] = DriverInputPlanner_legacy.computePedals( ...
    axRef, F_drive_full, F_resistance, mass, 10);
verifyGreaterThan(testCase, throttle, 0);
verifyLessThan(testCase, throttle, 1);
verifyEqual(testCase, throttle, F_resistance / F_drive_full, 'RelTol', 1e-9);
verifyEqual(testCase, brake, 0, 'AbsTol', 1e-12);
end

function testComputePedalsCoastWhenDragCoversDecel(testCase)
% Required decel is no more than what drag/rolling provide -> coast.
mass = 256;
F_resistance = 300;
coastDecel = F_resistance / mass;   % ~1.17 m/s^2
axRef = -coastDecel;                % exactly covered by drag
[throttle, brake] = DriverInputPlanner_legacy.computePedals( ...
    axRef, 3000, F_resistance, mass, 10);
verifyEqual(testCase, throttle, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, brake, 0, 'AbsTol', 1e-9);
end

function testComputePedalsCoastDeadbandAbsorbsNegligiblePedals(testCase)
% A negligibly small required drive force (would be <3% throttle) snaps to
% coast, modeling a real driver's lift-off rather than a held micro-throttle.
mass = 256;
F_resistance = 300;
F_drive_full = 3000;
% F_req just above 0 but below 3% of full-scale -> throttle < coastFraction.
axRef = -F_resistance/mass + 0.001;   % F_req = 0.001*mass, tiny positive
[throttle, brake] = DriverInputPlanner_legacy.computePedals( ...
    axRef, F_drive_full, F_resistance, mass, 12);
verifyEqual(testCase, throttle, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, brake, 0, 'AbsTol', 1e-12);
end

function testComputePedalsGradualBrakeScalesWithDecel(testCase)
% Decel beyond coast maps to a proportional, sub-1 brake command.
mass = 256;
F_resistance = 300;
brakeForceAccel = 12;               % decel per unit brake [m/s^2]
coastDecel = F_resistance / mass;
requiredDecel = coastDecel + 3;     % 3 m/s^2 beyond coast
axRef = -requiredDecel;
[throttle, brake] = DriverInputPlanner_legacy.computePedals( ...
    axRef, 3000, F_resistance, mass, brakeForceAccel);
verifyEqual(testCase, throttle, 0, 'AbsTol', 1e-12);
verifyGreaterThan(testCase, brake, 0);
verifyLessThan(testCase, brake, 1);
verifyEqual(testCase, brake, 3 / brakeForceAccel, 'RelTol', 1e-9);
end

function testComputePedalsFullBrakeClampsToOne(testCase)
% Huge required decel -> brake saturates at 1 (no >1 overshoot).
mass = 256;
[throttle, brake] = DriverInputPlanner_legacy.computePedals( ...
    -100, 3000, 300, mass, 12);
verifyEqual(testCase, throttle, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, brake, 1, 'AbsTol', 1e-9);
end

function testComputePedalsNeverAppliesBothPedals(testCase)
% Across a sweep of axRef, throttle and brake are never both > 0.
mass = 256;
axRefs = linspace(-20, 20, 51);
for k = 1:numel(axRefs)
    [throttle, brake] = DriverInputPlanner_legacy.computePedals( ...
        axRefs(k), 3000, 300, mass, 12);
    verifyLessThanOrEqual(testCase, min(throttle, brake), 0);
    verifyLessThanOrEqual(testCase, throttle, 1 + 1e-12);
    verifyLessThanOrEqual(testCase, brake, 1 + 1e-12);
    verifyGreaterThanOrEqual(testCase, throttle, -1e-12);
    verifyGreaterThanOrEqual(testCase, brake, -1e-12);
end
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
