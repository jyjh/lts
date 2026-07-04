function tests = ChassisLoadTransferTest
tests = functiontests(localfunctions);
end

function testPositiveLongitudinalAccelCreatesRearwardLoadShift(testCase)
[vehicle, suspension, chassis] = createVehicleWithChassis();
zeroAero = zeroAeroForces();
dt = 0.001;

for idx = 1:500
    chassis.updateFromAccelerations(4, 0, zeroAero, dt);
    loads = suspension.computeCornerLoadsFromChassis(chassis, 0, dt);
end

frontLoad = loads.FL + loads.FR;
rearLoad = loads.RL + loads.RR;
staticFront = vehicle.totalMass * 9.81 * vehicle.staticFrontWeight;
staticRear = vehicle.totalMass * 9.81 * (1 - vehicle.staticFrontWeight);

verifyGreaterThan(testCase, chassis.getPitchAngle(), 0);
verifyLessThan(testCase, frontLoad, staticFront);
verifyGreaterThan(testCase, rearLoad, staticRear);
end

function testPositiveLateralAccelCreatesRightSideLoadShift(testCase)
[~, suspension, chassis] = createVehicleWithChassis();
zeroAero = zeroAeroForces();
dt = 0.001;

for idx = 1:500
    chassis.updateFromAccelerations(0, 6, zeroAero, dt);
    loads = suspension.computeCornerLoadsFromChassis(chassis, 0, dt);
end

leftLoad = loads.FL + loads.RL;
rightLoad = loads.FR + loads.RR;

verifyGreaterThan(testCase, chassis.getRollAngle(), 0);
verifyGreaterThan(testCase, rightLoad, leftLoad);
end

function testYawAccelerationCreatesOpposedAxleRollExcitation(testCase)
[~, ~, chassis] = createVehicleWithChassis();
zeroAero = zeroAeroForces();
dt = 0.001;

chassis.updateFromAccelerations(0, 0, zeroAero, dt, 3);

verifyGreaterThan(testCase, chassis.state.frontRollAccel, 0);
verifyLessThan(testCase, chassis.state.rearRollAccel, 0);
end

function testZeroYawAccelerationMatchesScalarLateralRoll(testCase)
[~, ~, scalarChassis] = createVehicleWithChassis();
[~, ~, explicitChassis] = createVehicleWithChassis();
zeroAero = zeroAeroForces();
dt = 0.001;

scalarChassis.updateFromAccelerations(0, 6, zeroAero, dt);
explicitChassis.updateFromAccelerations(0, 6, zeroAero, dt, 0);

verifyEqual(testCase, explicitChassis.state.frontRollAccel, ...
    scalarChassis.state.frontRollAccel, 'AbsTol', 1e-12);
verifyEqual(testCase, explicitChassis.state.rearRollAccel, ...
    scalarChassis.state.rearRollAccel, 'AbsTol', 1e-12);
verifyEqual(testCase, explicitChassis.getFrontRollAngle(), ...
    scalarChassis.getFrontRollAngle(), 'AbsTol', 1e-12);
verifyEqual(testCase, explicitChassis.getRearRollAngle(), ...
    scalarChassis.getRearRollAngle(), 'AbsTol', 1e-12);
end

function testDragAboveCgCreatesPositivePitchMoment(testCase)
[~, ~, chassis] = createVehicleWithChassis();
aeroForces = zeroAeroForces();
aeroForces.F_drag = 100;
aeroForces.dragHeight = 0.5;

chassis.updateFromAccelerations(0, 0, aeroForces, 0.001);

verifyEqual(testCase, chassis.state.dragPitchMoment, 50, 'AbsTol', 1e-12);
verifyEqual(testCase, chassis.state.aeroPitchMoment, 50, 'AbsTol', 1e-12);
verifyGreaterThan(testCase, chassis.getPitchAngle(), 0);
end

function testVehicleConfigBuildLinksChassisAndUsesSprungMass(testCase)
config = lts.vehicles.baseline();
vehicle = lts.vehicle.VehicleManager.fromConfig(config, lts.components.TestTrack('straight10'), 0.001);

expectedSprungMass = config.totalMass - 4 * config.unsprungMass;
verifyEqual(testCase, vehicle.chassis.sprungMass, expectedSprungMass, 'AbsTol', 1e-12);
verifyFalse(testCase, isempty(vehicle.chassis.suspension));
verifyFalse(testCase, isempty(vehicle.suspension.chassis));
end

function testTwistUsesSeparateFrontAndRearRollForCornerKinematics(testCase)
state = lts.components.Chassis.ChassisState();
wheelbase = 1.6;
trackWidth = 1.2;
state.frontRollAngle = 0.10;
state.rearRollAngle = 0.02;
state.frontRollRate = 0.30;
state.rearRollRate = 0.05;

state.updateCornerKinematics(wheelbase, trackWidth, 0.5);

frontRollDisplacement = state.cornerDisplacement.FR - state.cornerDisplacement.FL;
rearRollDisplacement = state.cornerDisplacement.RR - state.cornerDisplacement.RL;
verifyEqual(testCase, frontRollDisplacement, state.frontRollAngle * trackWidth, 'AbsTol', 1e-12);
verifyEqual(testCase, rearRollDisplacement, state.rearRollAngle * trackWidth, 'AbsTol', 1e-12);
verifyNotEqual(testCase, frontRollDisplacement, rearRollDisplacement);
end

function testAlgebraicSuspensionFallbackStillComputesLoads(testCase)
config = lts.vehicles.baseline();
vehicle = lts.vehicle.VehicleManager([], [], [], [], []);
vehicle.totalMass = config.totalMass;
vehicle.wheelbase = config.wheelbase;
vehicle.trackWidth = config.trackWidth;
vehicle.cgHeight = config.cgHeight;
vehicle.staticFrontWeight = config.staticFrontWeight;
geometry = lts.components.Suspension.SuspensionGeometry.fromConfig( ...
    config.suspension.geometry, vehicle);
suspension = createSuspension(vehicle, config.suspension, config.unsprungMass, geometry);
vehicle.suspension = suspension;
suspension.warmup(vehicle.totalMass, 0.001);
state = lts.simulation.VehicleState('speed', 20);
state.vehicleManager = vehicle;
state.ax = 3;
state.ay = 6;

loads = suspension.computeCornerLoads(state, 120, 80, vehicle.totalMass, 0.001);
loadValues = [loads.FL; loads.FR; loads.RL; loads.RR];

verifyTrue(testCase, all(isfinite(loadValues)));
verifyTrue(testCase, all(loadValues >= 0));
end

function testAlgebraicFallbackUsesAxleSpecificLateralAcceleration(testCase)
config = lts.vehicles.baseline();
vehicle = lts.vehicle.VehicleManager([], [], [], [], []);
vehicle.totalMass = config.totalMass;
vehicle.wheelbase = config.wheelbase;
vehicle.trackWidth = config.trackWidth;
vehicle.cgHeight = config.cgHeight;
vehicle.staticFrontWeight = config.staticFrontWeight;
geometry = lts.components.Suspension.SuspensionGeometry.fromConfig( ...
    config.suspension.geometry, vehicle);
suspension = createSuspension(vehicle, config.suspension, config.unsprungMass, geometry);
vehicle.suspension = suspension;
suspension.warmup(vehicle.totalMass, 0.001);
state = lts.simulation.VehicleState('speed', 20);
state.vehicleManager = vehicle;
state.ax = 0;
state.ay = 0;
state.frontAxleAy = 5;
state.rearAxleAy = -5;

loads = suspension.estimateCornerLoads(state, 0, 0, vehicle.totalMass);

verifyGreaterThan(testCase, loads.FR, loads.FL);
verifyGreaterThan(testCase, loads.RL, loads.RR);
end

function [vehicle, suspension, chassis] = createVehicleWithChassis()
% Minimal chassis+suspension fixture for unit tests. Sources all tuning
% values from the baseline car config (single source of truth) rather than
% re-declaring literals here, so these tests stay in sync with the config.
config = lts.vehicles.baseline();
vehicle = lts.vehicle.VehicleManager([], [], [], [], []);
vehicle.totalMass = config.totalMass;
vehicle.wheelbase = config.wheelbase;
vehicle.trackWidth = config.trackWidth;
vehicle.cgHeight = config.cgHeight;
vehicle.staticFrontWeight = config.staticFrontWeight;

unsprungMass = config.unsprungMass;
sprungMass = vehicle.totalMass - 4 * unsprungMass;
chassis = lts.components.Chassis.SimpleChassis(vehicle, sprungMass);
chassis = applyChassisTuning(chassis, config.chassis);
vehicle.chassis = chassis;
geometry = lts.components.Suspension.SuspensionGeometry.fromConfig( ...
    config.suspension.geometry, vehicle);
suspension = createSuspension(vehicle, config.suspension, unsprungMass, geometry);
vehicle.suspension = suspension;
suspension.warmup(vehicle.totalMass, 0.001);
end

function suspension = createSuspension(vehicle, suspCfg, unsprungMass, geometry)
suspension = lts.components.Suspension.SuspensionManager( ...
    vehicle, ...
    suspCfg.rollStiffnessOverride, ...
    suspCfg.front.springRate, suspCfg.front.dampingCoeff, suspCfg.front.reboundCoeff, ...
    suspCfg.rear.springRate,  suspCfg.rear.dampingCoeff,  suspCfg.rear.reboundCoeff, ...
    suspCfg.motionRatio, ...
    suspCfg.bumpStopLength, ...
    suspCfg.bumpStopRate, ...
    suspCfg.tireSpringRate, ...
    unsprungMass, ...
    geometry);
end

function chassis = applyChassisTuning(chassis, chassisCfg)
% APPLYCHASSISTUNIS Apply configured platform stiffness/damping to a chassis.
% Returns the chassis (SimpleChassis is a value class, so the modified copy
% must be returned to the caller).
chassis.heaveStiffness    = chassisCfg.heaveStiffness;
chassis.heaveDamping      = chassisCfg.heaveDamping;
chassis.pitchStiffness    = chassisCfg.pitchStiffness;
chassis.pitchDamping      = chassisCfg.pitchDamping;
chassis.rollStiffness     = chassisCfg.rollStiffness;
chassis.rollDamping       = chassisCfg.rollDamping;
chassis.torsionalRigidity = chassisCfg.torsionalRigidity;
chassis.torsionalDamping  = chassisCfg.torsionalDamping;
end

function aeroForces = zeroAeroForces()
aeroForces = struct( ...
    'Fz_front', 0, ...
    'Fz_rear', 0, ...
    'F_drag', 0, ...
    'dragHeight', 0);
end
