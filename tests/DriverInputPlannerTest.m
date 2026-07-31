function tests = DriverInputPlannerTest
tests = functiontests(localfunctions);
end

function testConstantSpeedProfileCoastsAtTarget(testCase)
planner = lts.driver.DriverInputPlanner([], 0.6);
profile = createConstantSpeedProfile(1, 0);

input = planner.sampleAtProgress(profile, 0.5, 10.0);

verifyEqual(testCase, input.throttle, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, input.brake, 0, 'AbsTol', 1e-12);
end

function testUnderspeedUsesPartialThrottleWithoutBrake(testCase)
planner = lts.driver.DriverInputPlanner([], 0.6);
profile = createConstantSpeedProfile(0, 0);

input = planner.sampleAtProgress(profile, 0.5, 9.5);

verifyGreaterThan(testCase, input.throttle, 0);
verifyLessThan(testCase, input.throttle, 1);
verifyEqual(testCase, input.brake, 0, 'AbsTol', 1e-12);
end

function testOverspeedUsesBrakeAndClearsThrottle(testCase)
planner = lts.driver.DriverInputPlanner([], 0.6);
profile = createConstantSpeedProfile(1, 0);

input = planner.sampleAtProgress(profile, 0.5, 10.7);

verifyEqual(testCase, input.throttle, 0, 'AbsTol', 1e-12);
verifyGreaterThan(testCase, input.brake, 0);
verifyLessThan(testCase, input.brake, 1);
end

function testModestSpeedErrorsDoNotForceFullPedals(testCase)
planner = lts.driver.DriverInputPlanner([], 0.6);
profile = createConstantSpeedProfile(0, 0);

underSpeed = planner.sampleAtProgress(profile, 0.5, 8.8);
overSpeed = planner.sampleAtProgress(profile, 0.5, 11.2);

verifyGreaterThan(testCase, underSpeed.throttle, 0);
verifyLessThan(testCase, underSpeed.throttle, 0.5);
verifyEqual(testCase, underSpeed.brake, 0, 'AbsTol', 1e-12);
verifyGreaterThan(testCase, overSpeed.brake, 0);
verifyLessThan(testCase, overSpeed.brake, 0.5);
verifyEqual(testCase, overSpeed.throttle, 0, 'AbsTol', 1e-12);
end

function testPlannerUsesConfiguredRollingResistance(testCase)
lowProfile = createStraightProfileWithCrr(0);
highProfile = createStraightProfileWithCrr(0.10);

verifyEqual(testCase, max(lowProfile.brake), 0, 'AbsTol', 1e-12);
verifyEqual(testCase, max(highProfile.brake), 0, 'AbsTol', 1e-12);
verifyGreaterThan(testCase, mean(highProfile.throttle), mean(lowProfile.throttle));
end

function testRacingLineUsesOutsideApexOutsideOnNinetyTurn(testCase)
[profile, trackData, vehicle] = createPlannedProfile(lts.components.TestTrack('90turn'));
[iStart, iEnd] = findCornerSegment(trackData.curvature, 1);
iEntry = min(iStart + 4, iEnd);
iApex = round((iStart + iEnd) / 2);
iExit = max(iEnd - 4, iStart);

offset = profile.targetLateralError;
verifyLessThan(testCase, offset(iEntry), -0.05);
verifyGreaterThan(testCase, offset(iApex), 0.05);
verifyLessThan(testCase, offset(iExit), -0.05);
verifyLessThanOrEqual(testCase, max(abs(offset)), ...
    trackData.trackHalfWidth - 0.5 * vehicle.trackWidth + 1e-9);
end

function testRacingLineHandlesLeftAndRightCorners(testCase)
[profile, trackData, ~] = createPlannedProfile(lts.components.TestTrack('busstop'));
[leftStart, leftEnd] = findCornerSegment(trackData.curvature, 1);
[rightStart, rightEnd] = findCornerSegment(trackData.curvature, -1);
leftApex = round((leftStart + leftEnd) / 2);
rightApex = round((rightStart + rightEnd) / 2);

offset = profile.targetLateralError;
verifyLessThan(testCase, offset(min(leftStart + 4, leftEnd)), -0.05);
verifyGreaterThan(testCase, offset(leftApex), 0.05);
verifyGreaterThan(testCase, offset(min(rightStart + 4, rightEnd)), 0.05);
verifyLessThan(testCase, offset(rightApex), -0.05);
end

function testSlalomRacingLineOffsetIsSmooth(testCase)
[profile, ~, ~] = createPlannedProfile(lts.components.TestTrack('slalom'));

offsetStep = abs(diff(profile.targetLateralError));
verifyLessThan(testCase, max(offsetStep), 0.20);
verifyGreaterThan(testCase, max(abs(profile.targetLateralError)), 0.05);
end

function testEnduranceProfileHasBoundedLongitudinalReference(testCase)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
track = lts.components.WaypointTrack.loadMat( ...
    fullfile(repoRoot, 'tracks', ...
    'endurance_track_grid_25ft_from_matlab_smoothed.mat'));
track.Width = 5.0;
[profile, ~, ~] = createPlannedProfile(track);

verifyGreaterThan(testCase, min(profile.axRef), -15);
verifyLessThan(testCase, max(profile.axRef), 15);
verifyTrue(testCase, isfield(profile, 'targetLateralError'));
verifyTrue(testCase, isfield(profile, 'lineCurvature'));
end

function testPlannedProfileStaysFiniteOnSharpCorner(testCase)
% Regression guard for the backward-speed-profile sqrt guards in
% DriverModel.computeBackwardSpeedProfile / computeCornerSpeedLimit: a
% degenerate (<=0) maxLateralAccel or signed maxBrakeAccel must not poison
% the profile with NaN. Built on the 90turn track which has real curvature.
[profile, ~, ~] = createPlannedProfile(lts.components.TestTrack('90turn'));
verifyTrue(testCase, all(isfinite(profile.vTarget)), ...
    'vTarget must stay finite across the backward speed profile.');
verifyTrue(testCase, all(isfinite(profile.vLimit)), ...
    'vLimit must stay finite across the corner-speed limit.');
verifyTrue(testCase, all(isfinite(profile.axRef)), ...
    'axRef must stay finite.');
end

% ============================================================
% Physics-based pedal map (computePedals) unit tests.
% Pure function — no lts.vehicle.VehicleManager required. Each test pins one
% regime: WOT, partial/cruise throttle, true coast, gradual brake,
% full brake, and pedal exclusivity.
% ============================================================

function testComputePedalsWOTWhenForceSaturates(testCase)
% Required drive force meets/exceeds full-throttle capability -> WOT.
mass = 256;
F_drive_full = 3000;     % [N]
F_resistance = 300;      % drag + rolling [N]
axRef = 15;              % large positive -> F_req = 15*256 + 300 >> 3000
[throttle, brake] = lts.driver.DriverInputPlanner.computePedals( ...
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
[throttle, brake] = lts.driver.DriverInputPlanner.computePedals( ...
    axRef, F_drive_full, F_resistance, mass, 10);
verifyGreaterThan(testCase, throttle, 0);
verifyLessThan(testCase, throttle, 1);
verifyEqual(testCase, throttle, F_resistance / F_drive_full, 'RelTol', 1e-9);
verifyEqual(testCase, brake, 0, 'AbsTol', 1e-12);
end

function testComputePedalsInvertsNonlinearPowertrainMap(testCase)
powertrain = lts.components.Powertrain.EMRAX228Powertrain();
powertrain.throttleDeadband = 0.2;
powertrain.throttleMapInput = [0, 0.5, 1];
powertrain.throttleMapOutput = [0, 0.25, 1];

% Required contact-patch force is exactly half of WOT capability.
[throttle, brake] = lts.driver.DriverInputPlanner.computePedals( ...
    0, 1000, 500, 250, 10, powertrain);
expectedPedal = powertrain.pedalForTorqueFraction(0.5);

verifyEqual(testCase, throttle, expectedPedal, 'AbsTol', 1e-12);
verifyGreaterThan(testCase, throttle, 0.5);
verifyEqual(testCase, brake, 0, 'AbsTol', 1e-12);
end

function testComputePedalsAppliesCombinedGripScaleInTorqueSpace(testCase)
powertrain = lts.components.Powertrain.EMRAX228Powertrain();
powertrain.throttleMapInput = [0, 0.5, 1];
powertrain.throttleMapOutput = [0, 0.25, 1];

[throttle, ~] = lts.driver.DriverInputPlanner.computePedals( ...
    0, 1000, 500, 250, 10, powertrain, 0.5);

verifyEqual(testCase, throttle, ...
    powertrain.pedalForTorqueFraction(0.25), 'AbsTol', 1e-12);
end

function testComputePedalsCoastWhenDragCoversDecel(testCase)
% Required decel is no more than what drag/rolling provide -> coast.
mass = 256;
F_resistance = 300;
coastDecel = F_resistance / mass;   % ~1.17 m/s^2
axRef = -coastDecel;                % exactly covered by drag
[throttle, brake] = lts.driver.DriverInputPlanner.computePedals( ...
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
[throttle, brake] = lts.driver.DriverInputPlanner.computePedals( ...
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
[throttle, brake] = lts.driver.DriverInputPlanner.computePedals( ...
    axRef, 3000, F_resistance, mass, brakeForceAccel);
verifyEqual(testCase, throttle, 0, 'AbsTol', 1e-12);
verifyGreaterThan(testCase, brake, 0);
verifyLessThan(testCase, brake, 1);
verifyEqual(testCase, brake, 3 / brakeForceAccel, 'RelTol', 1e-9);
end

function testComputePedalsFullBrakeClampsToOne(testCase)
% Huge required decel -> brake saturates at 1 (no >1 overshoot).
mass = 256;
[throttle, brake] = lts.driver.DriverInputPlanner.computePedals( ...
    -100, 3000, 300, mass, 12);
verifyEqual(testCase, throttle, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, brake, 1, 'AbsTol', 1e-9);
end

function testComputePedalsNeverAppliesBothPedals(testCase)
% Across a sweep of axRef, throttle and brake are never both > 0.
mass = 256;
axRefs = linspace(-20, 20, 51);
for k = 1:numel(axRefs)
    [throttle, brake] = lts.driver.DriverInputPlanner.computePedals( ...
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

function [profile, trackData, vehicle] = createPlannedProfile(track)
dt = 0.001;
vehicle = lts.vehicle.VehicleManager.fromConfig(lts.vehicles.baseline(), track, dt);
driver = lts.driver.DriverModel(vehicle);
planner = lts.driver.DriverInputPlanner(vehicle, driver);
initialState = lts.simulation.VehicleState('s', 0, 'speed', 0.1);
initialState.vehicleManager = vehicle;
trackData = createTrackData(track);
profile = planner.buildOpenLoopProfile(initialState, trackData);
end

function profile = createStraightProfileWithCrr(crr)
config = lts.vehicles.baseline();
config.maxSpeed = 10;
config.tire.rollingResistanceCoeff = crr;
track = lts.components.TestTrack('straight10');
vehicle = lts.vehicle.VehicleManager.fromConfig(config, track, 0.001);
driver = lts.driver.DriverModel(vehicle);
planner = lts.driver.DriverInputPlanner(vehicle, driver);
initialState = lts.simulation.VehicleState('s', 0, 'speed', 10);
initialState.vehicleManager = vehicle;
profile = planner.buildOpenLoopProfile(initialState, createTrackData(track));
end

function trackData = createTrackData(track)
points = track.getTrackPoints();
arcLen = [0; cumsum(hypot(diff(points(:,1)), diff(points(:,2))))];
closedLoop = false;
if ismethod(track, 'isClosedLoop')
    closedLoop = track.isClosedLoop();
elseif isprop(track, 'Closed')
    closedLoop = track.Closed;
end
trackData = struct( ...
    'points', points, ...
    'arcLen', arcLen, ...
    'curvature', track.getCurvature(), ...
    'mu', track.getSurfaceFriction(), ...
    'heading', track.getHeading(), ...
    'length', arcLen(end), ...
    'trackWidth', track.getTrackWidth(), ...
    'trackHalfWidth', track.getTrackWidth() / 2, ...
    'closedLoop', logical(closedLoop), ...
    'baseTrackLength', track.getTotalLength(), ...
    'totalLaps', 1, ...
    'lapBreakS', [0; track.getTotalLength()], ...
    'nPts', size(points, 1));
end

function [iStart, iEnd] = findCornerSegment(curvature, turnSign)
active = sign(curvature(:)) == turnSign & abs(curvature(:)) > 1e-3;
idx = find(active);
if isempty(idx)
    error('DriverInputPlannerTest:MissingCorner', ...
        'No corner segment found for turn sign %d.', turnSign);
end
breaks = [0; find(diff(idx) > 1); numel(idx)];
bestLen = -inf;
iStart = idx(1);
iEnd = idx(end);
for k = 1:numel(breaks)-1
    runIdx = idx(breaks(k)+1:breaks(k+1));
    runLen = numel(runIdx);
    if runLen > bestLen
        bestLen = runLen;
        iStart = runIdx(1);
        iEnd = runIdx(end);
    end
end
end
