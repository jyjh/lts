function tests = CorrelationReplayTest
tests = functiontests(localfunctions);
end

function testReplayProfileIntegratesDistanceFromSpeed(testCase)
profile = CorrelationReplayProfile( ...
    'Time', [10; 11; 12], ...
    'Throttle', [0; 1; 0], ...
    'Brake', [0; 0; 0.5], ...
    'Steer', [0; 0.2; 0.4], ...
    'Speed', [10; 10; 10]);

verifyEqual(testCase, profile.time, [0; 1; 2], 'AbsTol', 1e-12);
verifyEqual(testCase, profile.distance, [0; 10; 20], 'AbsTol', 1e-12);
end

function testReplayProfileSamplesByDistance(testCase)
profile = CorrelationReplayProfile( ...
    'Time', [0; 1; 2], ...
    'Distance', [0; 10; 20], ...
    'Throttle', [0; 1; 0], ...
    'Brake', [0; 0; 0.5], ...
    'Steer', [0; 0.2; 0.4], ...
    'Speed', [10; 20; 10]);

input = profile.sampleByDistance(5);

verifyEqual(testCase, input.throttle, 0.5, 'AbsTol', 1e-12);
verifyEqual(testCase, input.brake, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, input.steer, 0.1, 'AbsTol', 1e-12);
verifyEqual(testCase, input.targetSpeed, 15, 'AbsTol', 1e-12);
end

function testReplayProfileReadsOptionalCorrelationColumns(testCase)
fileName = [tempname '.csv'];
cleanup = onCleanup(@() deleteIfExists(fileName)); %#ok<NASGU>
T = table( ...
    [0; 1], [0; 10], [0.1; 0.2], [0; 0.3], [0.01; 0.02], [8; 9], ...
    [1.0; 1.1], [2.0; 2.1], [0.3; 0.4], [0.5; 0.6], [-0.1; -0.2], ...
    'VariableNames', {'time_s', 'distance_m', 'throttle_ratio', ...
    'brake_ratio', 'steer_rad', 'speed_mps', 'gps_lat_deg', ...
    'gps_lon_deg', 'gps_course_rad', 'lat_accel_g', 'long_accel_g'});
writetable(T, fileName);

profile = CorrelationReplayProfile.fromCsv(fileName);

verifyTrue(testCase, profile.hasGpsCourse());
verifyTrue(testCase, profile.hasLatAccel());
verifyEqual(testCase, profile.gpsCourse, [0.3; 0.4], 'AbsTol', 1e-12);
verifyEqual(testCase, profile.latAccelG, [0.5; 0.6], 'AbsTol', 1e-12);
verifyEqual(testCase, profile.longAccelG, [-0.1; -0.2], 'AbsTol', 1e-12);
end

function testTrackAlignmentEstimatesKnownStationFromCourse(testCase)
track = localCircleTrack(50, 120);
knownStation = 80;
distance = (0:5:100).';
[trackS, trackHeading, trackLength] = ...
    CorrelationTrackAlignment.trackHeadingSamples(track);
course = CorrelationTrackAlignment.headingAtStation( ...
    trackS, trackHeading, trackLength, knownStation + distance);
profile = localProfileWithCourse(distance, course);

[station, errorDeg, sampleCount] = ...
    CorrelationTrackAlignment.estimateStartStationFromCourse(track, profile, 100, 0.5);

verifyEqual(testCase, station, knownStation, 'AbsTol', 0.51);
verifyLessThan(testCase, errorDeg, 0.1);
verifyEqual(testCase, sampleCount, numel(distance));
end

function testTrackRebasePreservesLengthAndMovesStart(testCase)
track = localCircleTrack(50, 120);
startStation = 80;
expectedStart = localTrackPointAtStation(track, startStation);

rebased = CorrelationTrackAlignment.rebaseTrack(track, startStation);
points = rebased.getTrackPoints();

verifyEqual(testCase, rebased.getTotalLength(), track.getTotalLength(), 'AbsTol', 1e-9);
verifyEqual(testCase, points(1, :), expectedStart, 'AbsTol', 1e-10);
verifyTrue(testCase, rebased.Closed);
end

function testTrackAlignmentReportsLargeHeadingMismatch(testCase)
track = localCircleTrack(50, 120);
heading = track.getHeading();
profile = localProfileWithCourse([0; 1; 2], repmat(heading(1) + pi, 3, 1));

errorDeg = CorrelationTrackAlignment.initialHeadingErrorDeg(track, profile);

verifyEqual(testCase, errorDeg, 180, 'AbsTol', 1e-9);
end

function testStateInitializerCanIgnoreLoggedYawRate(testCase)
track = localCircleTrack(50, 120);
profile = CorrelationReplayProfile( ...
    'Time', [0; 1], ...
    'Distance', [0; 10], ...
    'Throttle', [0; 0], ...
    'Brake', [0; 0], ...
    'Steer', [0; 0], ...
    'Speed', [5; 5], ...
    'YawRate', [2; 2]);

stateWithoutYawRate = CorrelationStateInitializer.fromReplayProfile( ...
    profile, track, [], 'UseLoggedYawRate', false);
stateWithYawRate = CorrelationStateInitializer.fromReplayProfile( ...
    profile, track, [], 'UseLoggedYawRate', true);

verifyEqual(testCase, stateWithoutYawRate.yawRate, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, stateWithYawRate.yawRate, 2, 'AbsTol', 1e-12);
end

function testReplayDriverSamplesByTime(testCase)
profile = CorrelationReplayProfile( ...
    'Time', [0; 1; 2], ...
    'Distance', [0; 10; 20], ...
    'Throttle', [0; 1; 0], ...
    'Brake', [0; 0; 0.5], ...
    'Steer', [0; 0.2; 0.4], ...
    'Speed', [10; 20; 10]);
driver = TelemetryReplayDriver(profile, 'ReplayDomain', 'time');
state = VehicleState('time', 0.5, 'speed', 10);

input = driver.computeInput(state, struct('s', 0));

verifyEqual(testCase, input.throttle, 0.5, 'AbsTol', 1e-12);
verifyEqual(testCase, input.steer, 0.1, 'AbsTol', 1e-12);
end

function testSimulatorReplayInputPolicyAllowsMeasuredOverlap(testCase)
simulator = Simulator(VehicleManager([], [], [], [], []), [], 0.1);
simulator.enforcePedalExclusivity = false;
simulator.applySteeringSlew = false;
state = VehicleState('steer', 0);
input = struct('throttle', 0.7, 'brake', 0.2, 'steer', 0.5);

normalized = simulator.normalizeDriverInput(input, state);

verifyEqual(testCase, normalized.throttle, 0.7, 'AbsTol', 1e-12);
verifyEqual(testCase, normalized.brake, 0.2, 'AbsTol', 1e-12);
verifyEqual(testCase, normalized.steer, 0.5, 'AbsTol', 1e-12);
end

function testSimulatorDefaultInputPolicyKeepsLegacyExclusivity(testCase)
simulator = Simulator(VehicleManager([], [], [], [], []), [], 0.1);
state = VehicleState('steer', 0);
input = struct('throttle', 0.7, 'brake', 0.2, 'steer', 0.5);

normalized = simulator.normalizeDriverInput(input, state);

verifyEqual(testCase, normalized.throttle, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, normalized.brake, 0.2, 'AbsTol', 1e-12);
end

function testReplayPolicyCanDisableOffTrackStop(testCase)
simulator = Simulator(VehicleManager([], [], [], [], []), [], 0.1);

verifyTrue(testCase, simulator.stopOnOffTrack);
verifyTrue(testCase, simulator.stopAtTrackEnd);
verifyTrue(testCase, isinf(simulator.stopTime));
simulator.restoreReplayPolicies([], [], true, true, false, false, 12.5);

verifyFalse(testCase, simulator.stopOnOffTrack);
verifyFalse(testCase, simulator.stopAtTrackEnd);
verifyEqual(testCase, simulator.stopTime, 12.5, 'AbsTol', 1e-12);
end

function testReplayContinuesOffTrackToReplayEnd(testCase)
dt = 0.01;
track = localCircleTrack(20, 40);
track.Width = 2;
profile = CorrelationReplayProfile( ...
    'Time', (0:dt:0.05).', ...
    'Distance', (0:dt:0.05).' * 2, ...
    'Throttle', linspace(0.2, 0.7, 6).', ...
    'Brake', zeros(6, 1), ...
    'Steer', zeros(6, 1), ...
    'Speed', 2 * ones(6, 1));
vehicle = VehicleManager.fromConfig(vehicles.R25(), track, dt);
simulator = Simulator(vehicle, [], dt);
initialState = VehicleState( ...
    'x', 100, ...
    'y', 100, ...
    'yaw', 0, ...
    'speed', 2, ...
    'vx', 2);

[stateLog, lapTime] = simulator.simulateReplay( ...
    initialState, track, profile, ...
    'ReplayDomain', 'time', ...
    'StopOnOffTrack', false, ...
    'StopAtTrackEnd', false, ...
    'StopAtReplayEnd', true);

verifyGreaterThanOrEqual(testCase, lapTime, 0.04);
verifyTrue(testCase, any(~stateLog.onTrack));
verifyLessThan(testCase, min(stateLog.trackLimitMargin), 0);
verifyEqual(testCase, stateLog.controlTime, (0:dt:0.04)', 'AbsTol', 1e-12);
verifyEqual(testCase, stateLog.controlS, (0:dt:0.04)' * 2, 'AbsTol', 1e-12);
verifyEqual(testCase, stateLog.replayThrottle, linspace(0.2, 0.6, 5)', 'AbsTol', 1e-12);
verifyEqual(testCase, stateLog.replayBrake, zeros(5, 1), 'AbsTol', 1e-12);
verifyEqual(testCase, stateLog.replaySpeed, 2 * ones(5, 1), 'AbsTol', 1e-12);
end

function track = localCircleTrack(radius, nPoints)
theta = linspace(0, 2*pi, nPoints + 1).';
theta(end) = [];
points = radius * [cos(theta), sin(theta)];
track = components.WaypointTrack(points, ...
    'Width', 5, ...
    'Closed', true, ...
    'Name', 'CorrelationTestCircle');
end

function profile = localProfileWithCourse(distance, course)
n = numel(distance);
profile = CorrelationReplayProfile( ...
    'Time', (0:n-1).', ...
    'Distance', distance(:), ...
    'Throttle', zeros(n, 1), ...
    'Brake', zeros(n, 1), ...
    'Steer', zeros(n, 1), ...
    'Speed', ones(n, 1), ...
    'GpsCourse', course(:));
end

function point = localTrackPointAtStation(track, station)
points = track.getTrackPoints();
s = components.Track.cumulativeArcLength(points, true);
p = [points; points(1, :)];
station = mod(station, s(end));
point = [ ...
    interp1(s, p(:, 1), station, 'linear'), ...
    interp1(s, p(:, 2), station, 'linear')];
end

function deleteIfExists(fileName)
if exist(fileName, 'file')
    delete(fileName);
end
end
