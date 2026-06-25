function tests = DriverModelSkidpadTest
tests = functiontests(localfunctions);
end

function testRepeatedSkidpadLapsUseEquivalentSteering(testCase)
[driver, trackData, baseLength] = createSkidpadDriver(0);
speed = 8;
lapFraction = 0.25;

inputLap1 = sampleDriverAtS(driver, trackData, lapFraction * baseLength, speed, 0);
inputLap2 = sampleDriverAtS(driver, trackData, baseLength + lapFraction * baseLength, speed, 0);

verifyEqual(testCase, inputLap2.steer, inputLap1.steer, 'AbsTol', 1e-9);
verifyGreaterThan(testCase, abs(inputLap1.steer), 0.01);
end

function testSteadyCircleTargetsCenterline(testCase)
[driver, trackData, baseLength] = createSkidpadDriver(0);

input = sampleDriverAtS(driver, trackData, 0.3 * baseLength, 8, 0.4);

verifyTrue(testCase, isfield(input, 'targetLateralError'));
verifyEqual(testCase, input.targetLateralError, 0, 'AbsTol', 1e-12);
end

function testEdgeCorrectionSteersAwayFromTrackLimit(testCase)
[driver, trackData, baseLength] = createSkidpadDriver(0);
driver.inputProfile.steer(:) = 0;

leftEdgeInput = sampleDriverAtS(driver, trackData, 0.3 * baseLength, 8, 1.2);
rightEdgeInput = sampleDriverAtS(driver, trackData, 0.3 * baseLength, 8, -1.2);

verifyLessThan(testCase, leftEdgeInput.steer, 0);
verifyGreaterThan(testCase, rightEdgeInput.steer, 0);
end

function testDriveSlipLimitReducesThrottleWithoutBrake(testCase)
[driver, trackData, baseLength] = createSkidpadDriver(0.35);

input = sampleDriverAtS(driver, trackData, 0.2 * baseLength, 5, 0);

verifyEqual(testCase, input.throttle, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, input.brake, 0, 'AbsTol', 1e-12);
end

function testNormalRearSlipLeavesThrottleUnchanged(testCase)
[driver, trackData, baseLength] = createSkidpadDriver(0.05);

input = sampleDriverAtS(driver, trackData, 0.2 * baseLength, 5, 0);

verifyEqual(testCase, input.throttle, 1, 'AbsTol', 1e-12);
verifyEqual(testCase, input.brake, 0, 'AbsTol', 1e-12);
end

function [driver, trackData, baseLength] = createSkidpadDriver(rearSlip)
track = components.TestTrack('skidpad');
simulator = Simulator([], [], 0.001);
[points, curvature, mu, heading] = simulator.repeatClosedTrack( ...
    track.getTrackPoints(), ...
    track.getCurvature(), ...
    track.getSurfaceFriction(), ...
    track.getHeading(), ...
    track.getWarmupLaps() + track.getRecordedLaps());
arcLen = [0; cumsum(sqrt(diff(points(:,1)).^2 + diff(points(:,2)).^2))];
baseLength = track.getTotalLength();

vehicle = struct( ...
    'track', track, ...
    'wheelbase', 1.558, ...
    'tire', createTireState(rearSlip));
driver = DriverModel(vehicle);
driver.inputDt = 0.001;
driver.throttleRampTime = 0;
driver.brakeRampTime = 0;
driver.steeringRampTime = 0;
driver.pedalReductionHoldTime = 0;
driver.pedalReleaseFilterTime = 0;
driver.pedalSwitchHoldTime = 0;
driver.trackArcLen = arcLen;
driver.trackCurvature = curvature(:);
driver.inputPlanner = DriverInputPlanner([], driver);
driver.inputProfile = createOpenLoopProfile(arcLen, curvature(:), vehicle.wheelbase);

trackData = struct( ...
    'arcLen', arcLen, ...
    'curvature', curvature(:), ...
    'heading', heading(:), ...
    'mu', mu(:), ...
    'trackWidth', track.getTrackWidth(), ...
    'trackHalfWidth', track.getTrackWidth() / 2);
end

function tire = createTireState(rearSlip)
tire = struct( ...
    'RL', struct('slipRatio', rearSlip), ...
    'RR', struct('slipRatio', rearSlip));
end

function profile = createOpenLoopProfile(arcLen, curvature, wheelbase)
steer = atan(wheelbase * curvature);
profile = struct( ...
    's', arcLen(:), ...
    'vTarget', 10 * ones(size(arcLen(:))), ...
    'vLimit', 10 * ones(size(arcLen(:))), ...
    'axRef', ones(size(arcLen(:))), ...
    'throttle', ones(size(arcLen(:))), ...
    'brake', zeros(size(arcLen(:))), ...
    'steer', steer(:));
end

function input = sampleDriverAtS(driver, trackData, s, speed, lateralError)
ref = referenceAtS(trackData, s, lateralError);
state = VehicleState('s', s, 'speed', speed, 'yaw', ref.heading);
state.refHeading = ref.heading;
state.refCurvature = ref.curvature;
state.lateralError = lateralError;
state.mu = ref.mu;

input = driver.computeInput(state, ref);
end

function ref = referenceAtS(trackData, s, lateralError)
idx = find(trackData.arcLen <= s, 1, 'last');
if isempty(idx)
    idx = 1;
end
idx = max(1, min(idx, numel(trackData.curvature)));
ref = struct( ...
    'idx', idx, ...
    's', s, ...
    'heading', trackData.heading(idx), ...
    'curvature', trackData.curvature(idx), ...
    'mu', trackData.mu(idx), ...
    'lateralError', lateralError, ...
    'trackWidth', trackData.trackWidth, ...
    'trackHalfWidth', trackData.trackHalfWidth, ...
    'trackLimitMargin', trackData.trackHalfWidth - abs(lateralError), ...
    'onTrack', abs(lateralError) <= trackData.trackHalfWidth);
end
