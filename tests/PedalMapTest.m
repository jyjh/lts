function tests = PedalMapTest
% PEDALMAPTEST Unit tests for the physics-based pedal map (PedalMap.compute).
tests = functiontests(localfunctions);
end

% ============================================================
% Physics-based pedal map unit tests.
% Pure function — no VehicleManager required. Each test pins one
% regime: WOT, partial/cruise throttle, true coast, gradual brake,
% full brake, and pedal exclusivity.
% ============================================================

function testWOTWhenForceSaturates(testCase)
% Required drive force meets/exceeds full-throttle capability -> WOT.
mass = 256;
F_drive_full = 3000;     % [N]
F_resistance = 300;      % drag + rolling [N]
axRef = 15;              % large positive -> F_req = 15*256 + 300 >> 3000
[throttle, brake] = PedalMap.compute( ...
    axRef, F_drive_full, F_resistance, mass, 10);
verifyEqual(testCase, throttle, 1, 'AbsTol', 1e-9);
verifyEqual(testCase, brake, 0, 'AbsTol', 1e-12);
end

function testPartialThrottleToMaintainSpeed(testCase)
% Speed-hold: axRef = 0 but drag must be overcome -> partial throttle.
mass = 256;
F_drive_full = 3000;
F_resistance = 300;
axRef = 0;
[throttle, brake] = PedalMap.compute( ...
    axRef, F_drive_full, F_resistance, mass, 10);
verifyGreaterThan(testCase, throttle, 0);
verifyLessThan(testCase, throttle, 1);
verifyEqual(testCase, throttle, F_resistance / F_drive_full, 'RelTol', 1e-9);
verifyEqual(testCase, brake, 0, 'AbsTol', 1e-12);
end

function testCoastWhenDragCoversDecel(testCase)
% Required decel is no more than what drag/rolling provide -> coast.
mass = 256;
F_resistance = 300;
coastDecel = F_resistance / mass;   % ~1.17 m/s^2
axRef = -coastDecel;                % exactly covered by drag
[throttle, brake] = PedalMap.compute( ...
    axRef, 3000, F_resistance, mass, 10);
verifyEqual(testCase, throttle, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, brake, 0, 'AbsTol', 1e-9);
end

function testCoastDeadbandAbsorbsNegligiblePedals(testCase)
% A negligibly small required drive force (would be <3% throttle) snaps to
% coast, modeling a real driver's lift-off rather than a held micro-throttle.
mass = 256;
F_resistance = 300;
F_drive_full = 3000;
% F_req just above 0 but below 3% of full-scale -> throttle < coastFraction.
axRef = -F_resistance/mass + 0.001;   % F_req = 0.001*mass, tiny positive
[throttle, brake] = PedalMap.compute( ...
    axRef, F_drive_full, F_resistance, mass, 12);
verifyEqual(testCase, throttle, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, brake, 0, 'AbsTol', 1e-12);
end

function testGradualBrakeScalesWithDecel(testCase)
% Decel beyond coast maps to a proportional, sub-1 brake command.
mass = 256;
F_resistance = 300;
brakeForceAccel = 12;               % decel per unit brake [m/s^2]
coastDecel = F_resistance / mass;
requiredDecel = coastDecel + 3;     % 3 m/s^2 beyond coast
axRef = -requiredDecel;
[throttle, brake] = PedalMap.compute( ...
    axRef, 3000, F_resistance, mass, brakeForceAccel);
verifyEqual(testCase, throttle, 0, 'AbsTol', 1e-12);
verifyGreaterThan(testCase, brake, 0);
verifyLessThan(testCase, brake, 1);
verifyEqual(testCase, brake, 3 / brakeForceAccel, 'RelTol', 1e-9);
end

function testFullBrakeClampsToOne(testCase)
% Huge required decel -> brake saturates at 1 (no >1 overshoot).
mass = 256;
[throttle, brake] = PedalMap.compute( ...
    -100, 3000, 300, mass, 12);
verifyEqual(testCase, throttle, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, brake, 1, 'AbsTol', 1e-9);
end

function testNeverAppliesBothPedals(testCase)
% Across a sweep of axRef, throttle and brake are never both > 0.
mass = 256;
axRefs = linspace(-20, 20, 51);
for k = 1:numel(axRefs)
    [throttle, brake] = PedalMap.compute( ...
        axRefs(k), 3000, 300, mass, 12);
    verifyLessThanOrEqual(testCase, min(throttle, brake), 0);
    verifyLessThanOrEqual(testCase, throttle, 1 + 1e-12);
    verifyLessThanOrEqual(testCase, brake, 1 + 1e-12);
    verifyGreaterThanOrEqual(testCase, throttle, -1e-12);
    verifyGreaterThanOrEqual(testCase, brake, -1e-12);
end
end
