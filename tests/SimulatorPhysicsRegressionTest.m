function tests = SimulatorPhysicsRegressionTest
tests = functiontests(localfunctions);
end

function testChassisStepDoesNotCallAlgebraicSuspensionCorrection(testCase)
tire = lts.components.Tire.PacejkaTire('43105_18x7.5_10_R25B_7.tir');
tire.relaxationLength = 0;
powertrain = SimulatorZeroPowertrain();
suspension = SimulatorChassisOnlySuspensionSpy(256 * 9.80665 / 4);
chassis = SimulatorChassisSpy();
aero = SimulatorZeroAero();

vehicle = lts.vehicle.VehicleManager(aero, suspension, powertrain, tire, [], chassis, []);
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
state = lts.simulation.VehicleState('speed', speed, 'vx', speed, 'vy', 0, ...
    'yaw', 0, 'x', 0, 'y', 0, 'mu', 1.2);
state.vehicleManager = vehicle;

simulator = lts.simulation.Simulator(vehicle, [], 0.001);
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
tire = lts.components.Tire.PacejkaTire('43105_18x7.5_10_R25B_7.tir');
tire.relaxationLength = 0;
powertrain = SimulatorZeroPowertrain();
suspension = SimulatorChassisOnlySuspensionSpy(256 * 9.80665 / 4);
chassis = SimulatorChassisSpy();
aero = SimulatorZeroAero();

vehicle = lts.vehicle.VehicleManager(aero, suspension, powertrain, tire, [], chassis, []);
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
state = lts.simulation.VehicleState('speed', speed, 'vx', speed, 'vy', 0, ...
    'yaw', 0, 'x', 0, 'y', 0, 'mu', 1.2);
state.vehicleManager = vehicle;

simulator = lts.simulation.Simulator(vehicle, [], 0.001);
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

function testMotorTorqueCommandModeUsesLoggedMotorTorque(testCase)
[vehicle, tire, powertrain] = directTorqueVehicle();
vehicle.powertrain.totalRatio = 3.4;
vehicle.powertrain.efficiency = 0.9;
powertrain = vehicle.powertrain;

speed = 10;
initializeWheelSpeeds(tire, speed);
state = lts.simulation.VehicleState('speed', speed, 'vx', speed, 'vy', 0, ...
    'yaw', 0, 'x', 0, 'y', 0, 'mu', 1.2);
state.vehicleManager = vehicle;

simulator = lts.simulation.Simulator(vehicle, [], 0.001);
simulator.powertrainMode = "motor_torque_command";
simulator.wheelSolveIterations = 1;
ref = struct( ...
    'heading', 0, ...
    'x', 0, ...
    'y', 0, ...
    'idx', 1, ...
    'trackData', straightTrackData());
input = struct('throttle', 0.7, 'brake', 0, 'steer', 0, ...
    'motorTorqueCommandNm', 50, ...
    'packVoltageV', 300, ...
    'packCurrentA', 1);

[~, forces] = simulator.step(state, input, ref);

expectedWheelTorque = 50 * powertrain.totalRatio * powertrain.efficiency;
verifyEqual(testCase, forces.driveTorqueTotal, expectedWheelTorque, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.coastdownTorqueTotal, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.motorTorque, 50, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.motorTorqueRequested, 50, 'AbsTol', 1e-12);
verifyTrue(testCase, isnan(forces.motorTorquePowerLimitNm));
verifyFalse(testCase, forces.motorTorquePowerLimitActive);
verifyEqual(testCase, forces.packPowerW, 300, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.wheelTorque, expectedWheelTorque, 'AbsTol', 1e-12);
end

function testMotorTorqueCommandModeCapsPositiveTorqueByPackPower(testCase)
[vehicle, tire, powertrain] = directTorqueVehicle();
vehicle.powertrain.totalRatio = 3.4;
vehicle.powertrain.efficiency = 0.9;
powertrain = vehicle.powertrain;

speed = 10;
initializeWheelSpeeds(tire, speed);
state = lts.simulation.VehicleState('speed', speed, 'vx', speed, 'vy', 0, ...
    'yaw', 0, 'x', 0, 'y', 0, 'mu', 1.2);
state.vehicleManager = vehicle;

simulator = lts.simulation.Simulator(vehicle, [], 0.001);
simulator.powertrainMode = "motor_torque_command";
simulator.limitMotorTorqueByPackPower = true;
simulator.wheelSolveIterations = 1;
ref = struct( ...
    'heading', 0, ...
    'x', 0, ...
    'y', 0, ...
    'idx', 1, ...
    'trackData', straightTrackData());
input = struct('throttle', 0.7, 'brake', 0, 'steer', 0, ...
    'motorTorqueCommandNm', 100, ...
    'packVoltageV', 300, ...
    'packCurrentA', 5);

[~, forces] = simulator.step(state, input, ref);

motorOmega = speed / tire.RL.wheelRadius * powertrain.totalRatio;
expectedMotorTorque = 1500 / motorOmega;
expectedWheelTorque = expectedMotorTorque * powertrain.totalRatio * powertrain.efficiency;
verifyEqual(testCase, forces.motorTorqueRequested, 100, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.motorTorque, expectedMotorTorque, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.motorTorquePowerLimitNm, expectedMotorTorque, 'AbsTol', 1e-12);
verifyTrue(testCase, forces.motorTorquePowerLimitActive);
verifyEqual(testCase, forces.packPowerW, 1500, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.driveTorqueTotal, expectedWheelTorque, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.coastdownTorqueTotal, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.wheelTorque, expectedWheelTorque, 'AbsTol', 1e-12);
end

function testWheelIterationFeedsCarrierSpeedBackIntoPackTorqueCap(testCase)
forcesOne = packLimitedMotoringStep(1);
forcesTwo = packLimitedMotoringStep(2);

% The first torque impulse raises carrier speed. The second fixed-point
% candidate must therefore use the lower constant-power torque limit.
verifyLessThan(testCase, forcesTwo.motorTorque, forcesOne.motorTorque);
verifyEqual(testCase, forcesTwo.motorTorquePowerLimitNm, ...
    forcesTwo.motorTorque, 'AbsTol', 1e-12);
verifyEqual(testCase, forcesTwo.driveTorqueTotal, ...
    forcesTwo.motorTorque * 3.4 * 0.9, 'AbsTol', 1e-12);
end

function testMotorTorqueCommandPowerCapUsesLoggedRpmBeforeSimulatedMotorSpeed(testCase)
[vehicle, tire, powertrain] = directTorqueVehicle();
vehicle.powertrain.totalRatio = 3.4;
vehicle.powertrain.efficiency = 0.9;
powertrain = vehicle.powertrain;

speed = 10;
initializeWheelSpeeds(tire, speed);
state = lts.simulation.VehicleState('speed', speed, 'vx', speed, 'vy', 0, ...
    'yaw', 0, 'x', 0, 'y', 0, 'mu', 1.2);
state.vehicleManager = vehicle;

simulator = lts.simulation.Simulator(vehicle, [], 0.001);
simulator.powertrainMode = "motor_torque_command";
simulator.limitMotorTorqueByPackPower = true;
simulator.wheelSolveIterations = 1;
ref = struct( ...
    'heading', 0, ...
    'x', 0, ...
    'y', 0, ...
    'idx', 1, ...
    'trackData', straightTrackData());
input = struct('throttle', 0.7, 'brake', 0, 'steer', 0, ...
    'motorTorqueCommandNm', 100, ...
    'motorRpm', 6000, ...
    'packVoltageV', 300, ...
    'packCurrentA', 5);

[~, forces] = simulator.step(state, input, ref);

motorOmega = 6000 * 2 * pi / 60;
expectedMotorTorque = 1500 / motorOmega;
expectedWheelTorque = expectedMotorTorque * powertrain.totalRatio * powertrain.efficiency;
verifyEqual(testCase, forces.motorTorque, expectedMotorTorque, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.wheelTorque, expectedWheelTorque, 'AbsTol', 1e-12);
verifyTrue(testCase, forces.motorTorquePowerLimitActive);
end

function testMotorTorqueCommandModeCutsPositiveTorqueAtRpmLimit(testCase)
[vehicle, tire, powertrain] = directTorqueVehicle();
vehicle.powertrain.totalRatio = 3.4;
vehicle.powertrain.efficiency = 0.9;
vehicle.powertrain.rpmLimitRPM = 1000;
powertrain = vehicle.powertrain;

speed = 20;
initializeWheelSpeeds(tire, speed);
state = lts.simulation.VehicleState('speed', speed, 'vx', speed, 'vy', 0, ...
    'yaw', 0, 'x', 0, 'y', 0, 'mu', 1.2);
state.vehicleManager = vehicle;

simulator = lts.simulation.Simulator(vehicle, [], 0.001);
simulator.powertrainMode = "motor_torque_command";
simulator.wheelSolveIterations = 1;
ref = struct( ...
    'heading', 0, ...
    'x', 0, ...
    'y', 0, ...
    'idx', 1, ...
    'trackData', straightTrackData());
input = struct('throttle', 0.7, 'brake', 0, 'steer', 0, ...
    'motorTorqueCommandNm', 50, ...
    'packVoltageV', 300, ...
    'packCurrentA', 100);

[~, forces] = simulator.step(state, input, ref);

verifyGreaterThan(testCase, powertrain.state.motorRPM, powertrain.rpmLimitRPM);
verifyEqual(testCase, forces.motorTorqueRequested, 50, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.motorTorque, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.wheelTorque, 0, 'AbsTol', 1e-12);
verifyTrue(testCase, forces.rpmLimitActive);
end

function testMotorTorqueCommandModeAllowsNegativeTorqueAtRpmLimit(testCase)
[vehicle, tire, powertrain] = directTorqueVehicle();
vehicle.powertrain.totalRatio = 3.4;
vehicle.powertrain.efficiency = 0.9;
vehicle.powertrain.rpmLimitRPM = 1000;
powertrain = vehicle.powertrain;

speed = 20;
initializeWheelSpeeds(tire, speed);
state = lts.simulation.VehicleState('speed', speed, 'vx', speed, 'vy', 0, ...
    'yaw', 0, 'x', 0, 'y', 0, 'mu', 1.2);
state.vehicleManager = vehicle;

simulator = lts.simulation.Simulator(vehicle, [], 0.001);
simulator.powertrainMode = "motor_torque_command";
simulator.wheelSolveIterations = 1;
ref = struct( ...
    'heading', 0, ...
    'x', 0, ...
    'y', 0, ...
    'idx', 1, ...
    'trackData', straightTrackData());
input = struct('throttle', 0.3, 'brake', 0, 'steer', 0, ...
    'motorTorqueCommandNm', -20);

[~, forces] = simulator.step(state, input, ref);

expectedWheelTorque = -20 * powertrain.totalRatio / powertrain.efficiency;
verifyGreaterThan(testCase, powertrain.state.motorRPM, powertrain.rpmLimitRPM);
verifyEqual(testCase, forces.motorTorque, -20, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.wheelTorque, expectedWheelTorque, 'AbsTol', 1e-12);
verifyFalse(testCase, forces.rpmLimitActive);
end

function testMotorTorqueCommandModeUsesNegativeTorqueAsCoastdown(testCase)
[vehicle, tire, powertrain] = directTorqueVehicle();
vehicle.powertrain.totalRatio = 3.4;
vehicle.powertrain.efficiency = 0.9;
powertrain = vehicle.powertrain;

speed = 10;
initializeWheelSpeeds(tire, speed);
state = lts.simulation.VehicleState('speed', speed, 'vx', speed, 'vy', 0, ...
    'yaw', 0, 'x', 0, 'y', 0, 'mu', 1.2);
state.vehicleManager = vehicle;

simulator = lts.simulation.Simulator(vehicle, [], 0.001);
simulator.powertrainMode = "motor_torque_command";
simulator.wheelSolveIterations = 1;
ref = struct( ...
    'heading', 0, ...
    'x', 0, ...
    'y', 0, ...
    'idx', 1, ...
    'trackData', straightTrackData());
input = struct('throttle', 0.3, 'brake', 0, 'steer', 0, ...
    'motorTorqueCommandNm', -20);

[~, forces] = simulator.step(state, input, ref);

expectedWheelTorque = -20 * powertrain.totalRatio / powertrain.efficiency;
verifyEqual(testCase, forces.driveTorqueTotal, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.coastdownTorqueTotal, expectedWheelTorque, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.motorTorque, -20, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.motorTorqueRequested, -20, 'AbsTol', 1e-12);
verifyFalse(testCase, forces.motorTorquePowerLimitActive);
verifyEqual(testCase, forces.wheelTorque, expectedWheelTorque, 'AbsTol', 1e-12);
end

function testMotorTorqueCommandModeUsesSeparateRegenEfficiency(testCase)
[vehicle, tire, powertrain] = directTorqueVehicle();
vehicle.powertrain.totalRatio = 3.4;
vehicle.powertrain.efficiency = 0.8;
vehicle.powertrain.regenEfficiency = 0.95;
powertrain = vehicle.powertrain;

speed = 10;
initializeWheelSpeeds(tire, speed);
state = lts.simulation.VehicleState('speed', speed, 'vx', speed, 'vy', 0, ...
    'yaw', 0, 'x', 0, 'y', 0, 'mu', 1.2);
state.vehicleManager = vehicle;

simulator = lts.simulation.Simulator(vehicle, [], 0.001);
simulator.powertrainMode = "motor_torque_command";
simulator.wheelSolveIterations = 1;
ref = struct( ...
    'heading', 0, ...
    'x', 0, ...
    'y', 0, ...
    'idx', 1, ...
    'trackData', straightTrackData());
input = struct('throttle', 0.3, 'brake', 0, 'steer', 0, ...
    'motorTorqueCommandNm', -20);

[~, forces] = simulator.step(state, input, ref);

expectedWheelTorque = -20 * powertrain.totalRatio / powertrain.regenEfficiency;
verifyEqual(testCase, forces.coastdownTorqueTotal, expectedWheelTorque, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.wheelTorque, expectedWheelTorque, 'AbsTol', 1e-12);
verifyEqual(testCase, vehicle.powertrain.state.drivetrainEfficiency, ...
    powertrain.regenEfficiency, 'AbsTol', 1e-12);
end

function testMotorTorqueCommandModeCapsNegativeTorqueByRegenPackPower(testCase)
[vehicle, tire, powertrain] = directTorqueVehicle();
vehicle.powertrain.totalRatio = 3.4;
vehicle.powertrain.efficiency = 0.9;
powertrain = vehicle.powertrain;

speed = 10;
initializeWheelSpeeds(tire, speed);
state = lts.simulation.VehicleState('speed', speed, 'vx', speed, 'vy', 0, ...
    'yaw', 0, 'x', 0, 'y', 0, 'mu', 1.2);
state.vehicleManager = vehicle;

simulator = lts.simulation.Simulator(vehicle, [], 0.001);
simulator.powertrainMode = "motor_torque_command";
simulator.limitMotorTorqueByPackPower = true;
simulator.wheelSolveIterations = 1;
ref = struct( ...
    'heading', 0, ...
    'x', 0, ...
    'y', 0, ...
    'idx', 1, ...
    'trackData', straightTrackData());
input = struct('throttle', 0.1, 'brake', 0, 'steer', 0, ...
    'motorTorqueCommandNm', -100, ...
    'packVoltageV', 300, ...
    'packCurrentA', -5);

[~, forces] = simulator.step(state, input, ref);

motorOmega = speed / tire.RL.wheelRadius * powertrain.totalRatio;
expectedMotorTorque = -1500 / motorOmega;
expectedWheelTorque = expectedMotorTorque * powertrain.totalRatio / powertrain.efficiency;
verifyEqual(testCase, forces.motorTorqueRequested, -100, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.motorTorque, expectedMotorTorque, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.motorTorquePowerLimitNm, expectedMotorTorque, 'AbsTol', 1e-12);
verifyTrue(testCase, forces.motorTorquePowerLimitActive);
verifyEqual(testCase, forces.packPowerW, -1500, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.driveTorqueTotal, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.coastdownTorqueTotal, expectedWheelTorque, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.wheelTorque, expectedWheelTorque, 'AbsTol', 1e-12);
end

function testMotorTorqueCommandModeUsesRegenCommandWhenPackIsCharging(testCase)
[vehicle, tire, powertrain] = directTorqueVehicle();
vehicle.powertrain.totalRatio = 3.4;
vehicle.powertrain.efficiency = 0.9;
powertrain = vehicle.powertrain;

speed = 10;
initializeWheelSpeeds(tire, speed);
state = lts.simulation.VehicleState('speed', speed, 'vx', speed, 'vy', 0, ...
    'yaw', 0, 'x', 0, 'y', 0, 'mu', 1.2);
state.vehicleManager = vehicle;

simulator = lts.simulation.Simulator(vehicle, [], 0.001);
simulator.powertrainMode = "motor_torque_command";
simulator.wheelSolveIterations = 1;
ref = struct( ...
    'heading', 0, ...
    'x', 0, ...
    'y', 0, ...
    'idx', 1, ...
    'trackData', straightTrackData());
input = struct('throttle', 0.1, 'brake', 0, 'steer', 0, ...
    'motorTorqueCommandNm', 10, ...
    'regenTorqueNm', -20, ...
    'packVoltageV', 300, ...
    'packCurrentA', -1000);

[~, forces] = simulator.step(state, input, ref);

expectedWheelTorque = -20 * powertrain.totalRatio / powertrain.efficiency;
verifyEqual(testCase, forces.motorTorqueRequested, -20, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.motorTorque, -20, 'AbsTol', 1e-12);
verifyFalse(testCase, forces.motorTorquePowerLimitActive);
verifyEqual(testCase, forces.driveTorqueTotal, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.coastdownTorqueTotal, expectedWheelTorque, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.wheelTorque, expectedWheelTorque, 'AbsTol', 1e-12);
end

function testMotorTorqueCommandModeRequiresReplayCommand(testCase)
[vehicle, tire, ~] = directTorqueVehicle();

speed = 10;
initializeWheelSpeeds(tire, speed);
state = lts.simulation.VehicleState('speed', speed, 'vx', speed, 'vy', 0, ...
    'yaw', 0, 'x', 0, 'y', 0, 'mu', 1.2);
state.vehicleManager = vehicle;

simulator = lts.simulation.Simulator(vehicle, [], 0.001);
simulator.powertrainMode = "motor_torque_command";
ref = struct( ...
    'heading', 0, ...
    'x', 0, ...
    'y', 0, ...
    'idx', 1, ...
    'trackData', straightTrackData());
input = struct('throttle', 0.3, 'brake', 0, 'steer', 0);

verifyError(testCase, @() simulator.step(state, input, ref), ...
    'lts_simulation_Simulator:MissingMotorTorqueCommand');
end

function testSimulatorRejectsObsoleteNoChassisPath(testCase)
vehicle = lts.vehicle.VehicleManager([], [], [], [], []);
simulator = lts.simulation.Simulator(vehicle, [], 0.001);
state = lts.simulation.VehicleState('speed', 1, 'vx', 1, ...
    'yaw', 0, 'x', 0, 'y', 0);
state.vehicleManager = vehicle;
input = struct('throttle', 0, 'brake', 0, 'steer', 0);
ref = struct('heading', 0, 'x', 0, 'y', 0);

verifyError(testCase, @() simulator.step(state, input, ref), ...
    'lts_simulation_Simulator:ChassisRequired');
end

function testWheelIterationsDoNotMultiplyAppliedImpulse(testCase)
dOmegaOne = zeroGripDrivenWheelDelta(1);
dOmegaThree = zeroGripDrivenWheelDelta(3);

verifyGreaterThan(testCase, dOmegaOne, 0);
% Fixed-point corrections may change the converged contact force, but must
% not scale angular impulse in proportion to the iteration count.
verifyEqual(testCase, dOmegaThree, dOmegaOne, 'RelTol', 0.15);
end

function testTireAligningMomentsContributeToYawMoment(testCase)
[vehicle, tire, ~] = directTorqueVehicle();
tire.relaxationLength = 0;
tire.rollingResistanceCoeff = 0;
speed = 10;
initializeWheelSpeeds(tire, speed);
state = lts.simulation.VehicleState('speed', speed, 'vx', speed, 'vy', 1, ...
    'yaw', 0, 'x', 0, 'y', 0, 'mu', 1.2, 'steer', 0);
state.vehicleManager = vehicle;
simulator = lts.simulation.Simulator(vehicle, [], 0.001);
kin = simulator.getCornerKinematics(0);
contact = simulator.computePlanarTireContactData(state, kin);
load = vehicle.totalMass * 9.80665 / 4;
loads = struct('FL', load, 'FR', load, 'RL', load, 'RR', load);

tireData = simulator.updatePlanarTireForces( ...
    state, loads, 0, true, 'steady', contact);
states = {tire.FL, tire.FR, tire.RL, tire.RR};
forceMoment = 0;
aligningMoment = 0;
for i = 1:4
    fxBody = states{i}.Fx * contact.cosWheelHeading(i) - ...
        states{i}.Fy * contact.sinWheelHeading(i);
    fyBody = states{i}.Fx * contact.sinWheelHeading(i) + ...
        states{i}.Fy * contact.cosWheelHeading(i);
    forceMoment = forceMoment + contact.xPositions(i) * fyBody - ...
        contact.yPositions(i) * fxBody;
    aligningMoment = aligningMoment + states{i}.Mz;
end

verifyNotEqual(testCase, aligningMoment, 0);
verifyEqual(testCase, tireData.yawMoment, forceMoment + aligningMoment, ...
    'AbsTol', 1e-9);
end

function testPlanarDynamicsReportsAxleSpecificLateralAcceleration(testCase)
vehicle = lts.vehicle.VehicleManager([], [], [], [], []);
vehicle.totalMass = 256;
vehicle.wheelbase = 1.6;
vehicle.staticFrontWeight = 0.45;
vehicle.yawInertia = 100;
simulator = lts.simulation.Simulator(vehicle, [], 0.001);
state = lts.simulation.VehicleState('speed', 10, 'vx', 10, 'vy', 0);
state.vehicleManager = vehicle;
tireData = struct( ...
    'sumFxBody', 0, ...
    'sumFyBody', 512, ...
    'yawMoment', 300);

dynamics = simulator.computePlanarDynamics(state, tireData, 0);

frontArm = vehicle.wheelbase * (1 - vehicle.staticFrontWeight);
rearArm = vehicle.wheelbase * vehicle.staticFrontWeight;
verifyEqual(testCase, dynamics.ay, 2, 'AbsTol', 1e-12);
verifyEqual(testCase, dynamics.yawAccel, 3, 'AbsTol', 1e-12);
verifyEqual(testCase, dynamics.frontAxleAy, ...
    dynamics.ay + dynamics.yawAccel * frontArm, 'AbsTol', 1e-12);
verifyEqual(testCase, dynamics.rearAxleAy, ...
    dynamics.ay - dynamics.yawAccel * rearArm, 'AbsTol', 1e-12);
end

function testPlanarKinematicsPreservesSteadyCircle(testCase)
speed = 10;
yawRate = 1;
dt = 0.001;
state = lts.simulation.VehicleState('speed', speed, 'vx', speed, 'vy', 0, ...
    'yaw', 0, 'yawRate', yawRate, 'x', 0, 'y', 0);
dynamics = struct('ax', 0, 'ay', speed * yawRate, 'yawAccel', 0);
simulator = lts.simulation.Simulator(lts.vehicle.VehicleManager([], [], [], [], []), [], dt);

for i = 1:round(10 / dt)
    kinematics = simulator.integratePlanarKinematics(state, dynamics, dt);
    state.vx = kinematics.vx;
    state.vy = kinematics.vy;
    state.speed = hypot(state.vx, state.vy);
    state.bodySlipAngle = atan2(state.vy, state.vx);
    state.yawRate = kinematics.yawRate;
    state.yaw = kinematics.yaw;
    state.x = kinematics.x;
    state.y = kinematics.y;
end

verifyEqual(testCase, state.speed, speed, 'AbsTol', 1e-3);
verifyLessThan(testCase, abs(state.bodySlipAngle), 1e-5);
end

function testLeanTelemetryAndMotecExportIncludeAxleAccelerations(testCase)
simulator = lts.simulation.Simulator(lts.vehicle.VehicleManager([], [], [], [], []), [], 0.001);
stateLog = simulator.createLeanStateLog(2);
verifyTrue(testCase, isfield(stateLog, 'frontAxleAy'));
verifyTrue(testCase, isfield(stateLog, 'rearAxleAy'));
verifyTrue(testCase, isfield(stateLog, 'frontRollRate'));
verifyTrue(testCase, isfield(stateLog, 'rearRollRate'));

stateLog.time = [0; 0.001];
stateLog.s = [0; 0.01];
stateLog.speed = [10; 10];
stateLog.speedKmh = stateLog.speed * 3.6;
stateLog.tireSpeed_FL = [11; 12];
stateLog.ax = [0; 0];
stateLog.ay = [1; 1.1];
stateLog.frontAxleAy = [1.2; 1.3];
stateLog.rearAxleAy = [0.8; 0.9];
stateLog.rollRate = [0.10; 0.11];
stateLog.frontRollRate = [0.12; 0.13];
stateLog.rearRollRate = [0.08; 0.09];
stateLog.twistRate = stateLog.frontRollRate - stateLog.rearRollRate;
stateLog.replayRegenTorqueNm = [-5; -6];
stateLog.replayMotorTorqueCommandNm = [10; -3];
stateLog.replayMotorRpm = [1000; 1100];
stateLog.replayPackVoltageV = [300; 301];
stateLog.replayPackCurrentA = [12; -4];
stateLog.replayPackPowerW = stateLog.replayPackVoltageV .* stateLog.replayPackCurrentA;
stateLog.motorTorque = [8; -2];
stateLog.motorTorqueRequested = [10; -3];
stateLog.motorTorquePowerLimitNm = [8; -2];
stateLog.motorTorquePowerLimitActive = [true; false];
stateLog.packVoltageV = stateLog.replayPackVoltageV;
stateLog.packCurrentA = stateLog.replayPackCurrentA;
stateLog.packPowerW = stateLog.replayPackPowerW;

csvFile = [tempname '.csv'];
cleanup = onCleanup(@() deleteIfExists(csvFile)); %#ok<NASGU>
lts.telemetry.TelemetryExporter.writeToMoTeCFormat(stateLog, csvFile);
header = firstCsvLine(csvFile);
headers = strsplit(header, ',');
data = readmatrix(csvFile, 'NumHeaderLines', 1);
simSpeedCol = find(strcmp(headers, 'Simulation Vehicle Speed Value (km/h)'), 1);

verifyTrue(testCase, any(strcmp(headers, 'Simulation Vehicle Speed Value (km/h)')));
verifyEqual(testCase, sum(strcmp(headers, 'Simulation Vehicle Speed Value (km/h)')), 1);
verifyFalse(testCase, any(strcmp(headers, 'Vehicle Speed Value (km/h)')));
verifyEqual(testCase, data(:, simSpeedCol), stateLog.tireSpeed_FL * 3.6, 'AbsTol', 1e-12);
verifyNotEqual(testCase, data(1, simSpeedCol), stateLog.speedKmh(1));
verifyTrue(testCase, contains(header, 'Front Axle Lat Accel Raw'));
verifyTrue(testCase, contains(header, 'Rear Axle Lat Accel Raw'));
verifyTrue(testCase, contains(header, 'G Sensor Front Axle Acceleration Lateral'));
verifyTrue(testCase, contains(header, 'G Sensor Rear Axle Acceleration Lateral'));
verifyTrue(testCase, contains(header, 'Roll Rate Front (deg/s)'));
verifyTrue(testCase, contains(header, 'Roll Rate Rear (deg/s)'));
verifyTrue(testCase, contains(header, 'Chassis Twist Rate (deg/s)'));
verifyTrue(testCase, contains(header, 'Replay Regen Torque (Nm)'));
verifyTrue(testCase, contains(header, 'Replay Motor Torque Command (Nm)'));
verifyTrue(testCase, contains(header, 'Replay Motor RPM (rpm)'));
verifyTrue(testCase, contains(header, 'Replay Pack Voltage (V)'));
verifyTrue(testCase, contains(header, 'Replay Pack Current (A)'));
verifyTrue(testCase, contains(header, 'Replay Pack Power (W)'));
verifyTrue(testCase, contains(header, 'Requested Motor Torque Command (Nm)'));
verifyTrue(testCase, contains(header, 'Pack Power Motor Torque Limit (Nm)'));
verifyTrue(testCase, contains(header, 'Pack Power Torque Limit Active (bool)'));
verifyTrue(testCase, contains(header, 'Pack Voltage (V)'));
verifyTrue(testCase, contains(header, 'Pack Current (A)'));
verifyTrue(testCase, contains(header, 'Pack Power (W)'));
end

function testMotecExportIncludesTireDiagnostics(testCase)
stateLog.time = [0; 0.001];
stateLog.s = [0; 0.01];
stateLog.speed = [10; 10];
stateLog.speedKmh = stateLog.speed * 3.6;
stateLog.slipAngle_FL = [0.01; 0.02];
stateLog.peakMu_FL = [1.5; 1.4];
stateLog.tireUtilization_FL = [0.8; 0.9];

csvFile = [tempname '.csv'];
cleanup = onCleanup(@() deleteIfExists(csvFile)); %#ok<NASGU>
lts.telemetry.TelemetryExporter.writeToMoTeCFormat(stateLog, csvFile);
header = firstCsvLine(csvFile);

verifyTrue(testCase, contains(header, 'Slip Angle FL (deg)'));
verifyTrue(testCase, contains(header, 'Peak MU FL (ratio)'));
verifyTrue(testCase, contains(header, 'Tire Utilization FL (%)'));
end

function [vehicle, tire, powertrain] = directTorqueVehicle()
tire = lts.components.Tire.PacejkaTire('43105_18x7.5_10_R25B_7.tir');
tire.relaxationLength = 0;
powertrain = SimulatorZeroPowertrain();
suspension = SimulatorChassisOnlySuspensionSpy(256 * 9.80665 / 4);
chassis = SimulatorChassisSpy();
aero = SimulatorZeroAero();

vehicle = lts.vehicle.VehicleManager(aero, suspension, powertrain, tire, [], chassis, []);
vehicle.totalMass = 256;
vehicle.wheelbase = 1.558;
vehicle.trackWidth = 1.21;
vehicle.cgHeight = 0.3;
vehicle.yawInertia = 130;
vehicle.staticFrontWeight = 0.5;
vehicle.brakeBiasFront = 0.6;
vehicle.brakeForceCoefficient = 0.7;
vehicle.maxSpeed = 80;
end

function testPowertrainModeCacheTracksRuntimePolicyChanges(testCase)
[vehicle, ~, ~] = directTorqueVehicle();
simulator = lts.simulation.Simulator(vehicle, [], 0.001);
state = lts.simulation.VehicleState('speed', 10, 'vx', 10, 'vy', 0);

simulator.powertrainMode = "throttle";
[driveTorque, coastTorque] = simulator.computePowertrainTorques( ...
    state, struct(), 0.5);
verifyEqual(testCase, driveTorque, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, coastTorque, 0, 'AbsTol', 1e-12);

simulator.powertrainMode = "motor_torque_command";
input = struct('motorTorqueCommandNm', -20);
[driveTorque, coastTorque] = simulator.computePowertrainTorques( ...
    state, input, 0);
verifyEqual(testCase, driveTorque, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, coastTorque, -20, 'AbsTol', 1e-12);
end

function testFullRunResetClearsComponentDynamics(testCase)
[vehicle, tire, powertrain] = directTorqueVehicle();
tire.FL.angularVelocity = 100;
tire.RR.slipRatio = -0.8;
tire.RR.Fx = -500;
powertrain.state.updateFromDrivenWheels(100, 1);
vehicle.chassis.state.pitchAngle = 0.2;

simulator = lts.simulation.Simulator(vehicle, [], 0.001);
simulator.resetForSimulation();

verifyEqual(testCase, tire.FL.angularVelocity, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, tire.RR.slipRatio, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, tire.RR.Fx, 0, 'AbsTol', 1e-12);
verifyFalse(testCase, powertrain.state.motorSpeedInitialized);
verifyEqual(testCase, vehicle.chassis.state.pitchAngle, 0, 'AbsTol', 1e-12);
end

function testLateralDragAtLongitudinalOffsetCreatesYawMoment(testCase)
vehicle = lts.vehicle.VehicleManager([], [], [], [], []);
vehicle.totalMass = 100;
vehicle.yawInertia = 50;
vehicle.wheelbase = 1.6;
vehicle.staticFrontWeight = 0.5;
simulator = lts.simulation.Simulator(vehicle, [], 0.001);
state = lts.simulation.VehicleState( ...
    'speed', hypot(10, 10), 'vx', 10, 'vy', 10);
tireData = struct('sumFxBody', 0, 'sumFyBody', 0, 'yawMoment', 0);
aeroForces = struct('F_drag', 100, 'dragXPosition', 1);

dynamics = simulator.computePlanarDynamics(state, tireData, aeroForces);

expectedFy = -100 / sqrt(2);
verifyEqual(testCase, dynamics.dragForceY, expectedFy, 'AbsTol', 1e-12);
verifyEqual(testCase, dynamics.yawMoment, expectedFy, 'AbsTol', 1e-12);
verifyEqual(testCase, dynamics.yawAccel, expectedFy / 50, 'AbsTol', 1e-12);
end

function testOutboardAeroCenterPreservesPitchMoment(testCase)
vehicle = lts.vehicle.VehicleManager([], [], [], [], []);
vehicle.wheelbase = 1.6;
vehicle.staticFrontWeight = 0.5;
vehicle.cgHeight = 0.3;
vehicle.airDensity = 1.2;
frontArm = vehicle.wheelbase * (1 - vehicle.staticFrontWeight);
rearArm = vehicle.wheelbase * vehicle.staticFrontWeight;
xPosition = frontArm + 0.2;
aero = lts.components.Aero.WholeCarAero(xPosition, 0.3, 1, 0, 0);
state = lts.simulation.VehicleState('speed', 10, 'vx', 10);
state.vehicleManager = vehicle;

forces = aero.computeForces(state);
totalDownforce = aero.computeDownforce(state);

verifyGreaterThan(testCase, forces.Fz_front, totalDownforce);
verifyLessThan(testCase, forces.Fz_rear, 0);
verifyEqual(testCase, forces.Fz_front + forces.Fz_rear, ...
    totalDownforce, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.Fz_front * frontArm - ...
    forces.Fz_rear * rearArm, totalDownforce * xPosition, 'AbsTol', 1e-12);
end

function dOmega = zeroGripDrivenWheelDelta(iterations)
[vehicle, tire, ~] = directTorqueVehicle();
% Isolate the applied driveline impulse with an unloaded contact patch.
% Surface mu is deliberately not a force-scaling control anymore.
vehicle.suspension.loadPerCorner = 0;
tire.relaxationLength = 0;
tire.rollingResistanceCoeff = 0;
speed = 10;
initializeWheelSpeeds(tire, speed);
state = lts.simulation.VehicleState('speed', speed, 'vx', speed, 'vy', 0, ...
    'yaw', 0, 'x', 0, 'y', 0, 'mu', 1);
state.vehicleManager = vehicle;
simulator = lts.simulation.Simulator(vehicle, [], 0.001);
simulator.powertrainMode = "motor_torque_command";
simulator.wheelSolveIterations = iterations;
ref = struct('heading', 0, 'x', 0, 'y', 0, 's', 0, ...
    'mu', 1, 'curvature', 0, 'referenceMode', 'free');
input = struct('throttle', 0.5, 'brake', 0, 'steer', 0, ...
    'motorTorqueCommandNm', 50);
omega0 = tire.RL.angularVelocity;

simulator.step(state, input, ref);
dOmega = tire.RL.angularVelocity - omega0;
end

function forces = packLimitedMotoringStep(iterations)
[vehicle, tire, ~] = directTorqueVehicle();
vehicle.powertrain.totalRatio = 3.4;
vehicle.powertrain.efficiency = 0.9;
speed = 10;
initializeWheelSpeeds(tire, speed);
state = lts.simulation.VehicleState('speed', speed, 'vx', speed, 'vy', 0, ...
    'yaw', 0, 'x', 0, 'y', 0, 'mu', 1.2);
state.vehicleManager = vehicle;
simulator = lts.simulation.Simulator(vehicle, [], 0.001);
simulator.powertrainMode = "motor_torque_command";
simulator.limitMotorTorqueByPackPower = true;
simulator.wheelSolveIterations = iterations;
ref = struct('heading', 0, 'x', 0, 'y', 0, 'idx', 1, ...
    'trackData', straightTrackData());
input = struct('throttle', 0.7, 'brake', 0, 'steer', 0, ...
    'motorTorqueCommandNm', 100, 'packVoltageV', 300, 'packCurrentA', 5);

[~, forces] = simulator.step(state, input, ref);
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

function line = firstCsvLine(fileName)
fid = fopen(fileName, 'r');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
line = fgetl(fid);
end

function deleteIfExists(fileName)
if exist(fileName, 'file')
    delete(fileName);
end
end
