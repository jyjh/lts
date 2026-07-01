function tests = DriverModelTest
% DRIVERMODELTEST Unit tests for the new preview-follow racing driver.
tests = functiontests(localfunctions);
end

% ============================================================
% These tests exercise the controller's sign behavior without a full
% simulator. They use a track with a long straight (so feedforward steer is
% ~0 and individual terms can be isolated) plus a left-hand corner. The car
% is placed ON its racing line (yaw = line heading, offset = line offset)
% and then perturbed to check each correction's sign.
% ============================================================

function [driver, trackData] = makeDriver(rearSlip)
if nargin < 1
    rearSlip = 0.0;
end
[trackData, vm] = mockTrackAndVehicle(rearSlip);
driver = DriverModel(vm);
driver.inputDt = 0.001;
driver.pedalFilterTime = 0;   % disable smoothing for deterministic tests

initialState = VehicleState('speed', 10);
driver = driver.prepareForSimulation(initialState, trackData, 0.001);
end

function [trackData, vm] = mockTrackAndVehicle(rearSlip)
% A long straight (idx 1..40) then a left-hand corner. The straight lets us
% isolate cross-track / edge / oversteer signs without feedforward curvature.
seg1 = [(0:1:40)', zeros(41,1)];                 % 40 m straight along +x
R = 20; cx = 40; cy = 20;
th = linspace(-pi/2, 0, 25)';
corner = [cx + R*cos(th), cy + R*sin(th)];
corner(1,:) = [];
pts = [seg1; corner];
closed = false;
curvature = components.Track.computeCurvature(pts, closed);
heading = components.Track.computeHeading(pts, closed);
arcLen = components.Track.cumulativeArcLength(pts, closed);
trackData = struct( ...
    'points', pts, 'arcLen', arcLen(:), ...
    'curvature', curvature(:), 'heading', heading(:), ...
    'mu', 1.2 * ones(size(pts,1), 1), ...
    'trackWidth', 6.0, 'trackHalfWidth', 3.0, 'closedLoop', closed);

vm = struct();
vm.totalMass = 256;
vm.g = 9.80665;
vm.wheelbase = 1.558;
vm.maxSpeed = 80;
vm.staticFrontWeight = 0.50;
vm.brakeBiasFront = 0.60;
vm.brakeForceCoefficient = 0.70;
vm.tire = struct('getPeakFriction', @(~)1.6, ...
    'RL', struct('slipRatio', rearSlip), ...
    'RR', struct('slipRatio', rearSlip));
vm.aero = struct('computeForces', @(~)struct( ...
    'F_drag', 0, 'Fz_front', 0, 'Fz_rear', 0));
vm.powertrain = struct('computeMaxDriveForce', @(~)3000);
end

function observation = obsAt(idx, lateralError)
halfWidth = 3.0;
observation = struct( ...
    'idx', idx, ...
    's', NaN, 'x', NaN, 'y', NaN, ...
    'heading', NaN, 'curvature', NaN, 'mu', 1.2, ...
    'lateralError', lateralError, ...
    'trackWidth', halfWidth * 2, ...
    'trackHalfWidth', halfWidth, ...
    'trackLimitMargin', halfWidth - abs(lateralError), ...
    'onTrack', abs(lateralError) <= halfWidth);
end

function state = onLineState(driver, idx, varargin)
% A VehicleState sitting on the racing line at idx (yaw = line heading),
% with optional overrides (e.g. bodySlipAngle for the oversteer test).
state = VehicleState('speed', 12, 'yaw', driver.racingLine.heading(idx), ...
    'bodySlipAngle', 0, 'yawRate', 0);
for k = 1:2:numel(varargin)
    state.(varargin{k}) = varargin{k+1};
end
state.vehicleManager = [];
end

function testCornerSteerPointsTowardApex(testCase)
% On-line in the corner, the feedforward + preview steer should be positive
% (left) for this left-hand corner.
[driver, ~] = makeDriver();
cornerIdx = 50;   % inside the corner
state = onLineState(driver, cornerIdx);
input = driver.computeInput(state, obsAt(cornerIdx, driver.racingLine.offsetW(cornerIdx)));
verifyTrue(testCase, isfinite(input.steer));
verifyGreaterThan(testCase, input.steer, 0.02);
end

function testCrossTrackSteersBackTowardLine(testCase)
% On the STRAIGHT (feedforward ~0): a car LEFT of its line should steer more
% right (smaller steer) than a car RIGHT of its line.
[driver, ~] = makeDriver();
straightIdx = 20;   % well inside the straight
lineOff = driver.racingLine.offsetW(straightIdx);   % ~0 on the straight

leftState = onLineState(driver, straightIdx);
rightState = onLineState(driver, straightIdx);
leftInput = driver.computeInput(leftState, obsAt(straightIdx, lineOff + 1.0));
rightInput = driver.computeInput(rightState, obsAt(straightIdx, lineOff - 1.0));
% Left-of-line (e>0) -> cross-track term is negative (steer right).
verifyLessThan(testCase, leftInput.steer, rightInput.steer);
end

function testOversteerTriggersCounterSteer(testCase)
% On the straight with zero feedforward, a large positive body slip (tail
% out to the right) should ADD negative (left/counter) steer vs no slip.
[driver, ~] = makeDriver();
straightIdx = 20;
lineOff = driver.racingLine.offsetW(straightIdx);
clean = onLineState(driver, straightIdx);
sliding = onLineState(driver, straightIdx, 'bodySlipAngle', 0.20);
cleanInput = driver.computeInput(clean, obsAt(straightIdx, lineOff));
slidingInput = driver.computeInput(sliding, obsAt(straightIdx, lineOff));
verifyLessThan(testCase, slidingInput.steer, cleanInput.steer);
end

function testTractionControlCutsThrottleOnWheelSpin(testCase)
% With high rear slip, throttle is cut well below the no-slip case.
driverClean = makeDriver(0.02);
driverSpin = makeDriver(0.30);
straightIdx = 20;
lineOff = driverClean.racingLine.offsetW(straightIdx);
state = VehicleState('speed', 8, 'yaw', 0, 'bodySlipAngle', 0, 'yawRate', 0);
stateSpin = state;
stateSpin.vehicleManager = driverSpin.vehicleManager;
cleanInput = driverClean.computeInput(state, obsAt(straightIdx, lineOff));
spinInput = driverSpin.computeInput(stateSpin, obsAt(straightIdx, lineOff));
verifyGreaterThan(testCase, cleanInput.throttle, 0.5);
verifyLessThan(testCase, spinInput.throttle, cleanInput.throttle - 0.3);
end

function testEdgeCorrectionSteersAwayFromLimit(testCase)
% On the straight, near the left edge steer should be more negative than near
% the right edge.
[driver, ~] = makeDriver();
straightIdx = 20;
baseState = onLineState(driver, straightIdx);
leftInput = driver.computeInput(baseState, obsAt(straightIdx, 2.6));
rightInput = driver.computeInput(baseState, obsAt(straightIdx, -2.6));
verifyLessThan(testCase, leftInput.steer, rightInput.steer);
end

function testUndersteerBreatheReducesThrottle(testCase)
% When the car understeers (yaw rate lagging the corner demand) on the
% throttle, the driver should breathe (reduce) throttle vs the grip case.
[driver, ~] = makeDriver();
cornerIdx = 50;
lineOff = driver.racingLine.offsetW(cornerIdx);
gripped = onLineState(driver, cornerIdx, 'yawRate', 0.6);
understeering = onLineState(driver, cornerIdx, 'yawRate', 0.05);
gInput = driver.computeInput(gripped, obsAt(cornerIdx, lineOff));
uInput = driver.computeInput(understeering, obsAt(cornerIdx, lineOff));
verifyLessThanOrEqual(testCase, uInput.throttle, gInput.throttle + 1e-9);
end
