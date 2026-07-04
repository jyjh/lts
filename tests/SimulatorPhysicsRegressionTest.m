function tests = SimulatorPhysicsRegressionTest
tests = functiontests(localfunctions);
end

function testChassisStepDoesNotCallAlgebraicSuspensionCorrection(testCase)
tire = components.Tire.PacejkaTire('43105_18x7.5_10_R25B_7.tir');
tire.relaxationLength = 0;
powertrain = SimulatorZeroPowertrain();
suspension = SimulatorChassisOnlySuspensionSpy(256 * 9.80665 / 4);
chassis = SimulatorChassisSpy();
aero = SimulatorZeroAero();

vehicle = VehicleManager(aero, suspension, powertrain, tire, [], chassis, []);
vehicle.totalMass = 256;
vehicle.wheelbase = 1.558;
vehicle.trackWidth = 1.21;
vehicle.cgHeight = 0.3;
vehicle.yawInertia = 130;
vehicle.staticFrontWeight = 0.5;
vehicle.brakeBiasFront = 0.6;
vehicle.brakeForceCoefficient = 0.7;
vehicle.maxSpeed = 80;

speed = 10;
initializeWheelSpeeds(tire, speed);
state = VehicleState('speed', speed, 'vx', speed, 'vy', 0, ...
    'yaw', 0, 'x', 0, 'y', 0, 'mu', 1.2);
state.vehicleManager = vehicle;

simulator = Simulator(vehicle, [], 0.001);
simulator.wheelSolveIterations = 1;
ref = struct( ...
    'heading', 0, ...
    'x', 0, ...
    'y', 0, ...
    'idx', 1, ...
    'trackData', straightTrackData());
input = struct('throttle', 0, 'brake', 0, 'steer', 0);

simulator.step(state, input, ref);

verifyEqual(testCase, suspension.algebraicCalls, 0);
verifyGreaterThanOrEqual(testCase, suspension.chassisCalls, 1);
end

function testPressureBrakeModeUsesLoggedAxlePressures(testCase)
tire = components.Tire.PacejkaTire('43105_18x7.5_10_R25B_7.tir');
tire.relaxationLength = 0;
powertrain = SimulatorZeroPowertrain();
suspension = SimulatorChassisOnlySuspensionSpy(256 * 9.80665 / 4);
chassis = SimulatorChassisSpy();
aero = SimulatorZeroAero();

vehicle = VehicleManager(aero, suspension, powertrain, tire, [], chassis, []);
vehicle.totalMass = 256;
vehicle.wheelbase = 1.558;
vehicle.trackWidth = 1.21;
vehicle.cgHeight = 0.3;
vehicle.yawInertia = 130;
vehicle.staticFrontWeight = 0.5;
vehicle.brakeBiasFront = 0.6;
vehicle.brakeForceCoefficient = 0.7;
vehicle.brakePressureFrontForcePerBar = 100;
vehicle.brakePressureRearForcePerBar = 60;
vehicle.maxSpeed = 80;

speed = 10;
initializeWheelSpeeds(tire, speed);
state = VehicleState('speed', speed, 'vx', speed, 'vy', 0, ...
    'yaw', 0, 'x', 0, 'y', 0, 'mu', 1.2);
state.vehicleManager = vehicle;

simulator = Simulator(vehicle, [], 0.001);
simulator.brakeMode = "pressure";
simulator.wheelSolveIterations = 1;
ref = struct( ...
    'heading', 0, ...
    'x', 0, ...
    'y', 0, ...
    'idx', 1, ...
    'trackData', straightTrackData());
input = struct( ...
    'throttle', 0, ...
    'brake', 0.2, ...
    'steer', 0, ...
    'brakePressureFrontBar', 10, ...
    'brakePressureRearBar', 5);

[~, forces] = simulator.step(state, input, ref);

expectedFrontTorque = 10 * vehicle.brakePressureFrontForcePerBar * ...
    tire.FL.wheelRadius / 2;
expectedRearTorque = 5 * vehicle.brakePressureRearForcePerBar * ...
    tire.RL.wheelRadius / 2;
verifyTrue(testCase, forces.brakePressureMode);
verifyEqual(testCase, forces.brakeCommand, 0.2, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.brakePressureFrontBar, 10, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.brakePressureRearBar, 5, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.brakeTorque_FL, expectedFrontTorque, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.brakeTorque_FR, expectedFrontTorque, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.brakeTorque_RL, expectedRearTorque, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.brakeTorque_RR, expectedRearTorque, 'AbsTol', 1e-12);
end

function testSimulatorCachesPersistAcrossMethodCalls(testCase)
vehicle = VehicleManager([], [], [], [], []);
simulator = Simulator(vehicle, [], 0.001);

tf = simulator.hasChassis();

verifyFalse(testCase, tf);
verifyFalse(testCase, isempty(simulator.cachedHasChassis));
verifyEqual(testCase, simulator.cachedHasChassis, false);
end

function initializeWheelSpeeds(tire, speed)
corners = {tire.FL, tire.FR, tire.RL, tire.RR};
for i = 1:numel(corners)
    corners{i}.angularVelocity = speed / corners{i}.wheelRadius;
end
end

function trackData = straightTrackData()
points = [0 0; 100 0];
trackData = struct( ...
    'points', points, ...
    'arcLen', [0; 100], ...
    'heading', [0; 0], ...
    'curvature', [0; 0], ...
    'mu', [1.2; 1.2], ...
    'length', 100, ...
    'trackWidth', 3, ...
    'trackHalfWidth', 1.5, ...
    'nPts', 2);
end
