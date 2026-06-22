function tests = TrackTelemetryWindowTest
tests = functiontests(localfunctions);
end

function testSkidpadDeclaresWarmupAndRecordedLap(testCase)
track = components.TestTrack('skidpad');

verifyTrue(testCase, track.isClosedLoop());
verifyEqual(testCase, track.getWarmupLaps(), 1);
verifyEqual(testCase, track.getRecordedLaps(), 1);
end

function testDefaultInitialYawAlignsToTrackHeading(testCase)
track = components.TestTrack('skidpad');
trackData = createTrackData(track);
simulator = Simulator([], [], 0.001);
state = VehicleState('s', 0, 'speed', 0.1);

state = simulator.initializePlanarState(state, trackData);

verifyEqual(testCase, state.yaw, trackData.heading(1), 'AbsTol', 1e-12);
verifyEqual(testCase, state.heading, trackData.heading(1), 'AbsTol', 1e-12);
end

function testExplicitInitialYawIsPreserved(testCase)
track = components.TestTrack('skidpad');
trackData = createTrackData(track);
simulator = Simulator([], [], 0.001);
state = VehicleState('s', 0, 'speed', 0.1, 'yaw', 0);

state = simulator.initializePlanarState(state, trackData);

verifyEqual(testCase, state.yaw, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, state.heading, 0, 'AbsTol', 1e-12);
end

function testClosedTrackRepeatHasContinuousArcLength(testCase)
track = components.TestTrack('skidpad');
simulator = Simulator([], [], 0.001);

[points, curvature, mu, heading] = simulator.repeatClosedTrack( ...
    track.getTrackPoints(), ...
    track.getCurvature(), ...
    track.getSurfaceFriction(), ...
    track.getHeading(), ...
    2);

arcLen = [0; cumsum(sqrt(diff(points(:,1)).^2 + diff(points(:,2)).^2))];

verifyEqual(testCase, numel(curvature), size(points, 1));
verifyEqual(testCase, numel(mu), size(points, 1));
verifyEqual(testCase, numel(heading), size(points, 1));
verifyTrue(testCase, all(diff(arcLen) > 0));
verifyEqual(testCase, arcLen(end), 2 * track.getTotalLength(), 'AbsTol', 1e-6);
end

function testTelemetryWindowDropsWarmupAndRezeros(testCase)
simulator = Simulator([], [], 0.001);
stateLog = createStateLog();

[stateLog, lapTime, recordedSteps] = simulator.applyTelemetryLapWindow( ...
    stateLog, 50, 100);

verifyEqual(testCase, recordedSteps, 3);
verifyEqual(testCase, stateLog.time(1), 0);
verifyEqual(testCase, stateLog.controlTime(1), 0);
verifyEqual(testCase, stateLog.s, [1; 10; 30]);
verifyEqual(testCase, stateLog.controlS, [0; 9; 29]);
verifyEqual(testCase, stateLog.refS, [1; 10; 30]);
verifyEqual(testCase, lapTime, 2);
end

function testTelemetryWindowPreservesNormalLapTiming(testCase)
simulator = Simulator([], [], 0.001);
stateLog = createStateLog();

[stateLog, lapTime, recordedSteps] = simulator.applyTelemetryLapWindow( ...
    stateLog, 0, 102);

verifyEqual(testCase, recordedSteps, 6);
verifyEqual(testCase, stateLog.time, (1:6)');
verifyEqual(testCase, stateLog.controlTime, (0:5)');
verifyEqual(testCase, lapTime, 6);
end

function testTelemetryWindowCanReturnEmptyTimedLap(testCase)
simulator = Simulator([], [], 0.001);
stateLog = createStateLog();

warnState = warning('off', 'Simulator:NoRecordedTelemetry');
cleanup = onCleanup(@() warning(warnState));

[stateLog, lapTime, recordedSteps] = simulator.applyTelemetryLapWindow( ...
    stateLog, 200, 250);

verifyEqual(testCase, recordedSteps, 0);
verifyEqual(testCase, lapTime, 0);
verifyTrue(testCase, isempty(stateLog.time));
verifyTrue(testCase, isempty(stateLog.s));
end

function stateLog = createStateLog()
stateLog = struct( ...
    'time', (1:6)', ...
    's', [10; 40; 51; 60; 80; 102], ...
    'controlTime', (0:5)', ...
    'controlS', [9; 39; 50; 59; 79; 101], ...
    'refS', [10; 40; 51; 60; 80; 102], ...
    'speedKmh', (11:16)');
end

function trackData = createTrackData(track)
trackData = struct( ...
    'points', track.getTrackPoints(), ...
    'heading', track.getHeading(), ...
    'curvature', track.getCurvature(), ...
    'mu', track.getSurfaceFriction());
end
