function tests = TrackTelemetryWindowTest
tests = functiontests(localfunctions);
end

function testSkidpadDeclaresWarmupAndRecordedLap(testCase)
track = lts.components.TestTrack('skidpad');

verifyTrue(testCase, track.isClosedLoop());
verifyEqual(testCase, track.getWarmupLaps(), 1);
verifyEqual(testCase, track.getRecordedLaps(), 1);
end

function testDefaultInitialYawAlignsToTrackHeading(testCase)
track = lts.components.TestTrack('skidpad');
trackData = createTrackData(track);
simulator = lts.simulation.Simulator([], [], 0.001);
state = lts.simulation.VehicleState('s', 0, 'speed', 0.1);

state = simulator.initializePlanarState(state, trackData);

verifyEqual(testCase, state.yaw, trackData.heading(1), 'AbsTol', 1e-12);
verifyEqual(testCase, state.heading, trackData.heading(1), 'AbsTol', 1e-12);
end

function testExplicitInitialYawIsPreserved(testCase)
track = lts.components.TestTrack('skidpad');
trackData = createTrackData(track);
simulator = lts.simulation.Simulator([], [], 0.001);
state = lts.simulation.VehicleState('s', 0, 'speed', 0.1, 'yaw', 0);

state = simulator.initializePlanarState(state, trackData);

verifyEqual(testCase, state.yaw, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, state.heading, 0, 'AbsTol', 1e-12);
end

function testClosedTrackRepeatHasContinuousArcLength(testCase)
track = lts.components.TestTrack('skidpad');
simulator = lts.simulation.Simulator([], [], 0.001);

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
simulator = lts.simulation.Simulator([], [], 0.001);
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
simulator = lts.simulation.Simulator([], [], 0.001);
stateLog = createStateLog();

[stateLog, lapTime, recordedSteps] = simulator.applyTelemetryLapWindow( ...
    stateLog, 0, 102);

verifyEqual(testCase, recordedSteps, 6);
verifyEqual(testCase, stateLog.time, (1:6)');
verifyEqual(testCase, stateLog.controlTime, (0:5)');
verifyEqual(testCase, lapTime, 6);
end

function testTelemetryWindowCanReturnEmptyTimedLap(testCase)
simulator = lts.simulation.Simulator([], [], 0.001);
stateLog = createStateLog();

lastwarn('');

[stateLog, lapTime, recordedSteps] = simulator.applyTelemetryLapWindow( ...
    stateLog, 200, 250);
[warnMsg, warnId] = lastwarn();

verifyEqual(testCase, recordedSteps, 0);
verifyEqual(testCase, lapTime, 0);
verifyTrue(testCase, isempty(stateLog.time));
verifyTrue(testCase, isempty(stateLog.s));
verifyEqual(testCase, warnId, 'lts_simulation_Simulator:NoRecordedTelemetry');
verifyTrue(testCase, contains(warnMsg, 'Max simulated s'));
verifyTrue(testCase, contains(warnMsg, 'minimum track margin'));
end

function testProjectToReferenceMatchesWithPrecomputedSegments(testCase)
simulator = lts.simulation.Simulator([], [], 0.001);
trackData = projectionTrackData();
cachedTrackData = simulator.precomputeTrackSegments(trackData);

uncachedRef = simulator.projectToReference(9.2, 2.1, trackData, 1);
cachedRef = simulator.projectToReference(9.2, 2.1, cachedTrackData, 1);

verifyEqual(testCase, cachedRef.idx, uncachedRef.idx);
verifyEqual(testCase, cachedRef.s, uncachedRef.s, 'AbsTol', 1e-12);
verifyEqual(testCase, cachedRef.lateralError, uncachedRef.lateralError, 'AbsTol', 1e-12);
verifyEqual(testCase, cachedRef.x, uncachedRef.x, 'AbsTol', 1e-12);
verifyEqual(testCase, cachedRef.y, uncachedRef.y, 'AbsTol', 1e-12);
verifyEqual(testCase, cachedRef.mu, uncachedRef.mu, 'AbsTol', 1e-12);
end

function testProjectToReferenceFallsBackWhenLocalWindowIsStale(testCase)
simulator = lts.simulation.Simulator([], [], 0.001);
trackData = simulator.precomputeTrackSegments(longProjectionTrackData());

ref = simulator.projectToReference(195, 0.2, trackData, 1);

verifyGreaterThan(testCase, ref.s, 190);
verifyLessThan(testCase, abs(ref.lateralError), 0.5);
end

function stateLog = createStateLog()
stateLog = struct( ...
    'time', (1:6)', ...
    's', [10; 40; 51; 60; 80; 102], ...
    'lateralError', [0; 0.1; 0.2; 0.3; 0.4; 0.5], ...
    'trackLimitMargin', [1.5; 1.4; 1.3; 1.2; 1.1; 1.0], ...
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

function trackData = projectionTrackData()
points = [0 0; 10 0; 10 10; 20 10];
arcLen = [0; 10; 20; 30];
trackData = struct( ...
    'points', points, ...
    'arcLen', arcLen, ...
    'heading', [0; pi/2; 0; 0], ...
    'curvature', [0; 0.1; 0; 0], ...
    'mu', [1.2; 0.9; 1.1; 1.2], ...
    'length', arcLen(end), ...
    'trackWidth', 3.0, ...
    'trackHalfWidth', 1.5, ...
    'nPts', size(points, 1));
end

function trackData = longProjectionTrackData()
x = (0:200)';
points = [x, zeros(size(x))];
arcLen = x;
trackData = struct( ...
    'points', points, ...
    'arcLen', arcLen, ...
    'heading', zeros(size(x)), ...
    'curvature', zeros(size(x)), ...
    'mu', 1.2 * ones(size(x)), ...
    'length', arcLen(end), ...
    'trackWidth', 3.0, ...
    'trackHalfWidth', 1.5, ...
    'nPts', size(points, 1));
end
