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

function testLinkedChassisConservesSteadyVerticalForce(testCase)
config = lts.vehicles.R25();
[vehicle, suspension, chassis] = createVehicleWithChassis(config);
dt = 0.001;
aeroForces = struct( ...
    'Fz_front', 1500, ...
    'Fz_rear', 1600, ...
    'F_drag', 0, ...
    'dragHeight', 0);

for idx = 1:6000
    loads = suspension.computeCornerLoadsFromChassis(chassis, 0, dt);
    chassis.updateFromAccelerations(0, 0, aeroForces, dt, 0);
end

travel = [suspension.frontLeft.state.damperPosition, ...
    suspension.frontRight.state.damperPosition, ...
    suspension.rearLeft.state.damperPosition, ...
    suspension.rearRight.state.damperPosition];
verifyTrue(testCase, all(travel < config.suspension.bumpStopLength));
verifyEqual(testCase, totalNormalLoad(loads), ...
    vehicle.totalMass * lts.vehicle.VehicleManager.g + ...
    aeroForces.Fz_front + aeroForces.Fz_rear, 'AbsTol', 2);
end

function testLinkedChassisVerticalForceIdentityIncludesUnsprungInertia(testCase)
[vehicle, suspension, chassis] = createVehicleWithChassis(lts.vehicles.R25());
dt = 0.001;
aeroForces = struct( ...
    'Fz_front', 3000, ...
    'Fz_rear', 3000, ...
    'F_drag', 0, ...
    'dragHeight', 0);

for idx = 1:1000
    loads = suspension.computeCornerLoadsFromChassis(chassis, 0, dt);
    chassis.updateFromAccelerations(0, 0, aeroForces, dt, 0);
end

suspensionForces = [ ...
    suspension.frontLeft.state.suspensionForce, ...
    suspension.frontRight.state.suspensionForce, ...
    suspension.rearLeft.state.suspensionForce, ...
    suspension.rearRight.state.suspensionForce];
tireForces = [loads.FL, loads.FR, loads.RL, loads.RR];
travel = [suspension.frontLeft.state.damperPosition, ...
    suspension.frontRight.state.damperPosition, ...
    suspension.rearLeft.state.damperPosition, ...
    suspension.rearRight.state.damperPosition];
verifyTrue(testCase, any(travel > suspension.frontLeft.bumpStopLength));

% Downward-positive point-mass balance. For each unsprung mass,
% m_u*z_u_ddot = F_suspension - F_tire; adding that to the chassis heave
% inertia must equal the net external vertical force exactly.
totalVerticalInertia = chassis.sprungMass * chassis.state.heaveAccel + ...
    sum(suspensionForces - tireForces);
netExternalDownwardForce = ...
    vehicle.totalMass * lts.vehicle.VehicleManager.g + ...
    aeroForces.Fz_front + aeroForces.Fz_rear - sum(tireForces);
verifyEqual(testCase, totalVerticalInertia, netExternalDownwardForce, ...
    'AbsTol', 1e-9);
end

function testGeometricTransferAtWheelLiftConservesAxleLoad(testCase)
[vehicle, suspension, chassis] = createVehicleWithChassis(lts.vehicles.R25());
frontTotal = vehicle.totalMass * lts.vehicle.VehicleManager.g * ...
    vehicle.staticFrontWeight;
rearTotal = vehicle.totalMass * lts.vehicle.VehicleManager.g * ...
    (1 - vehicle.staticFrontWeight);
chassis.state.frontGeometricLateralLoadTransfer = frontTotal;
chassis.state.rearGeometricLateralLoadTransfer = rearTotal;

loads = suspension.computeCornerLoadsFromChassis(chassis, 0, 0);

verifyEqual(testCase, loads.FL, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, loads.RL, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, loads.FL + loads.FR, frontTotal, 'AbsTol', 1e-9);
verifyEqual(testCase, loads.RL + loads.RR, rearTotal, 'AbsTol', 1e-9);
end

function testYawAccelerationCreatesOpposedAxleRollExcitation(testCase)
[~, ~, chassis] = createVehicleWithChassis();
zeroAero = zeroAeroForces();
dt = 0.001;

chassis.updateFromAccelerations(0, 0, zeroAero, dt, 3);

verifyGreaterThan(testCase, chassis.state.frontRollAccel, 0);
verifyLessThan(testCase, chassis.state.rearRollAccel, 0);
end

function testLinkedSuspensionLetsFrontAndRearRollRatesSeparate(testCase)
[~, suspension, chassis] = createVehicleWithChassis();
zeroAero = zeroAeroForces();
dt = 0.001;

[KwF, KwR] = suspension.getAxleRollStiffness();
verifyGreaterThan(testCase, abs(KwF - KwR), 1e-6);

chassis.torsionalRigidity = 5000;
chassis.torsionalDamping = 250;
for idx = 1:100
    chassis.updateFromAccelerations(0, 6, zeroAero, dt, 0);
end

verifyGreaterThan(testCase, abs(chassis.getFrontRollRate() - ...
    chassis.getRearRollRate()), 1e-7);
verifyGreaterThan(testCase, abs(chassis.getTwistRate()), 1e-7);
verifyGreaterThan(testCase, abs(chassis.getFrontRollAngle() - ...
    chassis.getRearRollAngle()), 1e-8);
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
aeroForces.F_drag_longitudinal = 100;
aeroForces.dragHeight = 0.5;

chassis.updateFromAccelerations(0, 0, aeroForces, 0.001);

expectedMoment = aeroForces.F_drag_longitudinal * ...
    (aeroForces.dragHeight - chassis.cgHeight);
verifyEqual(testCase, chassis.state.dragPitchMoment, expectedMoment, 'AbsTol', 1e-12);
verifyEqual(testCase, chassis.state.aeroPitchMoment, expectedMoment, 'AbsTol', 1e-12);
verifyGreaterThan(testCase, chassis.getPitchAngle(), 0);
end

function testDragThroughCgDoesNotCreatePitch(testCase)
[~, ~, chassis] = createVehicleWithChassis();
drag = 500;
aeroForces = zeroAeroForces();
aeroForces.F_drag = drag;
aeroForces.F_drag_longitudinal = drag;
aeroForces.dragHeight = chassis.cgHeight;
dragOnlyAx = -drag / chassis.totalMass;

chassis.updateFromAccelerations(dragOnlyAx, 0, aeroForces, 0.001);

verifyEqual(testCase, chassis.state.dragPitchMoment, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, chassis.state.pitchAccel, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, chassis.getPitchAngle(), 0, 'AbsTol', 1e-12);
end

function testLateralDragThroughCgDoesNotCreateRoll(testCase)
[~, ~, chassis] = createVehicleWithChassis();
drag = 400;
aeroForces = zeroAeroForces();
aeroForces.F_drag = drag;
aeroForces.F_drag_longitudinal = 0;
aeroForces.F_drag_lateral = drag;
aeroForces.dragHeight = chassis.cgHeight;
aeroForces.dragXPosition = 0;
dragOnlyAy = -drag / chassis.totalMass;

chassis.updateFromAccelerations(0, dragOnlyAy, aeroForces, 0.001, 0);

verifyEqual(testCase, chassis.state.frontRollAccel, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, chassis.state.rearRollAccel, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, chassis.getRollAngle(), 0, 'AbsTol', 1e-12);
end

function testAeroResultantUsesAbsoluteHeightAndWeightedXPosition(testCase)
vehicle = lts.vehicle.VehicleManager([], [], [], [], []);
vehicle.wheelbase = 1.6;
vehicle.staticFrontWeight = 0.5;
vehicle.cgHeight = 0.3;
vehicle.airDensity = 1.2;
state = lts.simulation.VehicleState('speed', 10, 'vx', 10);
state.vehicleManager = vehicle;

front = lts.components.Aero.WholeCarAero(0.4, 0.2, 0, 1, 0);
rear = lts.components.Aero.WholeCarAero(-0.2, 0.5, 0, 3, 0);
aero = lts.components.Aero.AeroManager();
aero = aero.addComponent(front);
aero = aero.addComponent(rear);
forces = aero.computeForces(state);

verifyEqual(testCase, forces.dragHeight, 0.425, 'AbsTol', 1e-12);
verifyEqual(testCase, forces.dragXPosition, -0.05, 'AbsTol', 1e-12);
end

function testWingHeightSensitivityIsFractionPerCentimeter(testCase)
vehicle = lts.vehicle.VehicleManager([], [], [], [], []);
vehicle.airDensity = 1.2;
state = lts.simulation.VehicleState('speed', 10, 'vx', 10);
state.vehicleManager = vehicle;
front = lts.components.Aero.FrontWing(0.5, 0.2, 1, 0, 0, 0.3);
rear = lts.components.Aero.RearWing(-0.5, 0.2, 1, 0, 0, 0.15);

state.rideHeight = 0;
frontNominal = front.computeDownforce(state);
rearNominal = rear.computeDownforce(state);
state.rideHeight = 0.01;

verifyEqual(testCase, front.computeDownforce(state) / frontNominal, ...
    0.7, 'AbsTol', 1e-12);
verifyEqual(testCase, rear.computeDownforce(state) / rearNominal, ...
    0.85, 'AbsTol', 1e-12);
end

function testInfiniteTorsionalRigidityEnforcesExactConstraint(testCase)
[~, ~, chassis] = createVehicleWithChassis();
chassis.torsionalRigidity = Inf;
chassis.state.frontRollAngle = 0.1;
chassis.state.rearRollAngle = -0.04;
chassis.state.frontRollRate = 0.3;
chassis.state.rearRollRate = -0.2;
zeroAero = zeroAeroForces();

for idx = 1:200
    chassis.updateFromAccelerations(0, 6, zeroAero, 0.001, 2);
    verifyEqual(testCase, chassis.state.frontRollAngle, ...
        chassis.state.rearRollAngle, 'AbsTol', 0);
    verifyEqual(testCase, chassis.state.frontRollRate, ...
        chassis.state.rearRollRate, 'AbsTol', 0);
end

verifyEqual(testCase, chassis.getTwistAngle(), 0, 'AbsTol', 0);
verifyEqual(testCase, chassis.getTwistRate(), 0, 'AbsTol', 0);
verifyTrue(testCase, isfinite(chassis.getRollAngle()));
verifyLessThan(testCase, abs(chassis.getRollAngle()), 1);
end

function testChassisRollContributesOppositeRoadFrameCamber(testCase)
config = lts.vehicles.baseline();
config.suspension.geometry.front.camberCurve = [0 0 0];
config.suspension.geometry.rear.camberCurve = [0 0 0];
config.suspension.geometry.front.toeCurve = [0 0 0];
config.suspension.geometry.rear.toeCurve = [0 0 0];
[~, suspension, chassis] = createVehicleWithChassis(config);
chassis.state.frontRollAngle = 0.05;
chassis.state.rearRollAngle = -0.02;

suspension.updateGeometry(0);
kin = suspension.getCornerKinematics();

verifyEqual(testCase, kin.FL.camberAngle, -0.05, 'AbsTol', 1e-12);
verifyEqual(testCase, kin.FR.camberAngle, 0.05, 'AbsTol', 1e-12);
verifyEqual(testCase, kin.RL.camberAngle, 0.02, 'AbsTol', 1e-12);
verifyEqual(testCase, kin.RR.camberAngle, -0.02, 'AbsTol', 1e-12);
end

function testVehicleConfigBuildLinksChassisAndUsesSprungMass(testCase)
    assumeTrue(testCase, tireDataAvailable(), 'TTC tire data not present: see src/+lts/+components/+Tire/README.md');
config = lts.vehicles.baseline();
vehicle = lts.vehicle.VehicleManager.fromConfig(config, lts.components.TestTrack('straight10'), 0.001);

expectedSprungMass = config.totalMass - 4 * config.unsprungMass;
verifyEqual(testCase, vehicle.chassis.sprungMass, expectedSprungMass, 'AbsTol', 1e-12);
verifyFalse(testCase, isempty(vehicle.chassis.suspension));
verifyFalse(testCase, isempty(vehicle.suspension.chassis));
end

function testSimpleChassisRequiresExplicitSprungMass(testCase)
config = lts.vehicles.baseline();
vehicle = minimalVehicleManager(config);

verifyError(testCase, ...
    @() lts.components.Chassis.SimpleChassis(vehicle), ...
    'lts_chassis_SimpleChassis:MissingSprungMass');
verifyError(testCase, ...
    @() lts.components.Chassis.SimpleChassis(vehicle, vehicle.totalMass + 1), ...
    'lts_chassis_SimpleChassis:InvalidSprungMass');
end

function testTorsionalRigidityAcceptsRigidConstraintAndRejectsInvalidValues(testCase)
config = lts.vehicles.baseline();
vehicle = minimalVehicleManager(config);
sprungMass = config.totalMass - 4 * config.unsprungMass;
chassis = lts.components.Chassis.SimpleChassis(vehicle, sprungMass);

chassis.torsionalRigidity = Inf;
verifyEqual(testCase, chassis.torsionalRigidity, Inf);
verifyError(testCase, @() setTorsionalRigidity(chassis, NaN), ...
    'lts_chassis_SimpleChassis:InvalidTorsionalRigidity');
verifyError(testCase, @() setTorsionalRigidity(chassis, -1), ...
    'lts_chassis_SimpleChassis:InvalidTorsionalRigidity');
verifyError(testCase, @() setTorsionalRigidity(chassis, -Inf), ...
    'lts_chassis_SimpleChassis:InvalidTorsionalRigidity');
end

function testLinkedSuspensionBypassesLegacyPlatformCoefficients(testCase)
configA = lts.vehicles.baseline();
configB = configA;
fallbackFields = {'heaveStiffness', 'heaveDamping', ...
    'pitchStiffness', 'pitchDamping', 'rollStiffness', 'rollDamping'};
for idx = 1:numel(fallbackFields)
    configA.chassis.(fallbackFields{idx}) = 0;
    configB.chassis.(fallbackFields{idx}) = 1e12;
end
[~, suspensionA, chassisA] = createVehicleWithChassis(configA);
[~, suspensionB, chassisB] = createVehicleWithChassis(configB);
aero = zeroAeroForces();
aero.Fz_front = 300;
aero.Fz_rear = 500;
dt = 0.001;

for idx = 1:50
    suspensionA.computeCornerLoadsFromChassis(chassisA, 0, dt);
    suspensionB.computeCornerLoadsFromChassis(chassisB, 0, dt);
    chassisA.updateFromAccelerations(2, 4, aero, dt, 0.5);
    chassisB.updateFromAccelerations(2, 4, aero, dt, 0.5);
end

verifyEqual(testCase, chassisA.state.heave, chassisB.state.heave, 'AbsTol', 0);
verifyEqual(testCase, chassisA.state.pitchAngle, chassisB.state.pitchAngle, 'AbsTol', 0);
verifyEqual(testCase, chassisA.state.frontRollAngle, ...
    chassisB.state.frontRollAngle, 'AbsTol', 0);
verifyEqual(testCase, chassisA.state.rearRollAngle, ...
    chassisB.state.rearRollAngle, 'AbsTol', 0);
end

function testR25WheelRateMatchesSpecSheetRollRate(testCase)
config = lts.vehicles.R25();
expectedRollRateNmPerDeg = 671.26;
expectedWheelRate = expectedRollRateNmPerDeg * 180 / pi * ...
    2 / config.trackWidth^2;

verifyEqual(testCase, config.suspension.front.springRate, expectedWheelRate, 'AbsTol', 0.1);
verifyEqual(testCase, config.suspension.rear.springRate, expectedWheelRate, 'AbsTol', 0.1);

frontSpringRollRate = config.suspension.front.springRate * ...
    config.suspension.motionRatio^2 * config.trackWidth^2 / 2 * pi / 180;
rearSpringRollRate = config.suspension.rear.springRate * ...
    config.suspension.motionRatio^2 * config.trackWidth^2 / 2 * pi / 180;

verifyEqual(testCase, frontSpringRollRate, expectedRollRateNmPerDeg, 'AbsTol', 1.0);
verifyEqual(testCase, rearSpringRollRate, expectedRollRateNmPerDeg, 'AbsTol', 1.0);

frontBar = lts.components.Suspension.AntiRollBar( ...
    config.suspension.frontArb.stiffness, ...
    config.suspension.frontArb.motionRatio, ...
    config.suspension.frontArb.leverArm, ...
    config.suspension.frontArb.enabled);
rearBar = lts.components.Suspension.AntiRollBar( ...
    config.suspension.rearArb.stiffness, ...
    config.suspension.rearArb.motionRatio, ...
    config.suspension.rearArb.leverArm, ...
    config.suspension.rearArb.enabled);

verifyFalse(testCase, config.suspension.frontArb.enabled);
verifyFalse(testCase, config.suspension.rearArb.enabled);
verifyEqual(testCase, frontBar.getWheelRateStiffness(), 0, 'AbsTol', 1e-12);
verifyEqual(testCase, rearBar.getWheelRateStiffness(), 0, 'AbsTol', 1e-12);
verifyEqual(testCase, frontSpringRollRate + frontBar.getWheelRateStiffness() * ...
    config.trackWidth^2 / 2 * pi / 180, expectedRollRateNmPerDeg, 'AbsTol', 1.0);
verifyEqual(testCase, rearSpringRollRate + rearBar.getWheelRateStiffness() * ...
    config.trackWidth^2 / 2 * pi / 180, expectedRollRateNmPerDeg, 'AbsTol', 1.0);
end

function testR25DampingMatchesSpecSheetPercentCritical(testCase)
config = lts.vehicles.R25();
sprungMass = config.totalMass - 4 * config.unsprungMass;
frontSprungCornerMass = sprungMass * config.staticFrontWeight / 2;
rearSprungCornerMass = sprungMass * (1 - config.staticFrontWeight) / 2;
frontCritical = 2 * sqrt(config.suspension.front.springRate * frontSprungCornerMass);
rearCritical = 2 * sqrt(config.suspension.rear.springRate * rearSprungCornerMass);

verifyEqual(testCase, config.suspension.front.dampingCoeff, 4.00 * frontCritical, 'AbsTol', 0.2);
verifyEqual(testCase, config.suspension.rear.dampingCoeff, 3.00 * rearCritical, 'AbsTol', 0.2);
verifyEqual(testCase, config.suspension.front.reboundCoeff, 0.80 * frontCritical, 'AbsTol', 0.2);
verifyEqual(testCase, config.suspension.rear.reboundCoeff, 0.90 * rearCritical, 'AbsTol', 0.2);
end

function testR25SuspensionGeometryMatchesSpecSheetStatics(testCase)
config = lts.vehicles.R25();
front = config.suspension.geometry.front;
rear = config.suspension.geometry.rear;

verifyEqual(testCase, config.suspension.bumpStopLength, 0.0254, 'AbsTol', 1e-12);
verifyEqual(testCase, front.travelGrid, [-0.0254 0 0.0254], 'AbsTol', 1e-12);
verifyEqual(testCase, rear.travelGrid, [-0.0254 0 0.0254], 'AbsTol', 1e-12);
verifyEqual(testCase, front.motionRatioCurve, [1 1 1], 'AbsTol', 1e-12);
verifyEqual(testCase, rear.motionRatioCurve, [1 1 1], 'AbsTol', 1e-12);
verifyEqual(testCase, front.rollCenterLateral, 0.013469, 'AbsTol', 1e-12);
verifyEqual(testCase, rear.rollCenterLateral, 0.024413, 'AbsTol', 1e-12);

frontCamberDeg = front.camberCurve * 180 / pi;
rearCamberDeg = rear.camberCurve * 180 / pi;
frontCamberSlope = (frontCamberDeg(3) - frontCamberDeg(1)) / ...
    (front.travelGrid(3) - front.travelGrid(1));
rearCamberSlope = (rearCamberDeg(3) - rearCamberDeg(1)) / ...
    (rear.travelGrid(3) - rear.travelGrid(1));

verifyEqual(testCase, frontCamberDeg(2), -0.7, 'AbsTol', 1e-6);
verifyEqual(testCase, rearCamberDeg(2), -0.6, 'AbsTol', 1e-6);
verifyEqual(testCase, frontCamberSlope, -49.9, 'AbsTol', 1e-6);
verifyEqual(testCase, rearCamberSlope, -66.8, 'AbsTol', 1e-6);
verifyEqual(testCase, front.toeCurve * 180 / pi, [0.75 0.75 0.75], 'AbsTol', 1e-12);
verifyEqual(testCase, rear.toeCurve * 180 / pi, [0 0 0], 'AbsTol', 1e-12);

vehicle = lts.vehicle.VehicleManager([], [], [], [], []);
vehicle.wheelbase = config.wheelbase;
vehicle.trackWidth = config.trackWidth;
vehicle.staticFrontWeight = config.staticFrontWeight;
geometry = lts.components.Suspension.SuspensionGeometry.fromConfig( ...
    config.suspension.geometry, vehicle);
verifyEqual(testCase, geometry.frontRollCenterLateral, front.rollCenterLateral, 'AbsTol', 1e-12);
verifyEqual(testCase, geometry.rearRollCenterLateral, rear.rollCenterLateral, 'AbsTol', 1e-12);

frontKin = geometry.computeCornerKinematics('FL', 0, 0);
rearKin = geometry.computeCornerKinematics('RL', 0, 0);
verifyEqual(testCase, frontKin.rollCenterLateral, front.rollCenterLateral, 'AbsTol', 1e-12);
verifyEqual(testCase, rearKin.rollCenterLateral, rear.rollCenterLateral, 'AbsTol', 1e-12);
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
[vehicle, suspension] = createAlgebraicVehicle(config);
state = lts.simulation.VehicleState('speed', 20);
state.vehicleManager = vehicle;
state.ax = 3;
state.ay = 6;

loads = suspension.computeCornerLoads(state, 120, 80, vehicle.totalMass, 0.001);
loadValues = [loads.FL; loads.FR; loads.RL; loads.RR];

verifyTrue(testCase, all(isfinite(loadValues)));
verifyTrue(testCase, all(loadValues >= 0));
end

function testAlgebraicFallbackZeroRollCenterPreservesLateralMoment(testCase)
config = setRollCenters(lts.vehicles.baseline(), 0, 0, 0, 0);
[vehicle, suspension] = createAlgebraicVehicle(config);
state = lts.simulation.VehicleState('speed', 20);
state.vehicleManager = vehicle;
state.ax = 0;
state.ay = 6;

loads = suspension.estimateCornerLoads(state, 0, 0, vehicle.totalMass);

expectedRightMinusLeft = 2 * vehicle.totalMass * state.ay * ...
    vehicle.cgHeight / vehicle.trackWidth;
verifyEqual(testCase, rightMinusLeft(loads), expectedRightMinusLeft, ...
    'AbsTol', 1e-9);
verifyEqual(testCase, totalNormalLoad(loads), ...
    vehicle.totalMass * lts.vehicle.VehicleManager.g, 'AbsTol', 1e-9);
end

function testAlgebraicFallbackRollCenterHeightChangesAxleSplit(testCase)
ay = 6;
flatConfig = setRollCenters(lts.vehicles.baseline(), 0, 0, 0, 0);
raisedConfig = setRollCenters(lts.vehicles.baseline(), 0.12, 0.03, 0, 0);
[flatVehicle, flatSuspension] = createAlgebraicVehicle(flatConfig);
[raisedVehicle, raisedSuspension] = createAlgebraicVehicle(raisedConfig);
flatState = lts.simulation.VehicleState('speed', 20);
flatState.vehicleManager = flatVehicle;
flatState.ax = 0;
flatState.ay = ay;
raisedState = lts.simulation.VehicleState('speed', 20);
raisedState.vehicleManager = raisedVehicle;
raisedState.ax = 0;
raisedState.ay = ay;

flatLoads = flatSuspension.estimateCornerLoads( ...
    flatState, 0, 0, flatVehicle.totalMass);
raisedLoads = raisedSuspension.estimateCornerLoads( ...
    raisedState, 0, 0, raisedVehicle.totalMass);

expectedRightMinusLeft = 2 * raisedVehicle.totalMass * ay * ...
    raisedVehicle.cgHeight / raisedVehicle.trackWidth;
flatFrontDiff = flatLoads.FR - flatLoads.FL;
raisedFrontDiff = raisedLoads.FR - raisedLoads.FL;

verifyEqual(testCase, rightMinusLeft(raisedLoads), expectedRightMinusLeft, ...
    'AbsTol', 1e-9);
verifyGreaterThan(testCase, abs(raisedFrontDiff - flatFrontDiff), 1e-6);
verifyEqual(testCase, totalNormalLoad(raisedLoads), ...
    raisedVehicle.totalMass * lts.vehicle.VehicleManager.g, 'AbsTol', 1e-9);
end

function testAlgebraicFallbackUsesAxleSpecificLateralAcceleration(testCase)
config = setRollCenters(lts.vehicles.baseline(), 0.10, 0.10, 0, 0);
[vehicle, suspension] = createAlgebraicVehicle(config);
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

function testRollCenterHeightReducesChassisRollResponse(testCase)
lowConfig = setRollCenters(lts.vehicles.baseline(), 0, 0, 0, 0);
highConfig = setRollCenters(lts.vehicles.baseline(), 0.15, 0.15, 0, 0);
[~, ~, lowChassis] = createVehicleWithChassis(lowConfig);
[~, ~, highChassis] = createVehicleWithChassis(highConfig);
zeroAero = zeroAeroForces();
dt = 0.001;

lowChassis.updateFromAccelerations(0, 6, zeroAero, dt, 0);
highChassis.updateFromAccelerations(0, 6, zeroAero, dt, 0);

verifyLessThan(testCase, abs(highChassis.state.frontRollAccel), ...
    abs(lowChassis.state.frontRollAccel));
verifyLessThan(testCase, abs(highChassis.state.rearRollAccel), ...
    abs(lowChassis.state.rearRollAccel));

for idx = 1:1000
    lowChassis.updateFromAccelerations(0, 6, zeroAero, dt, 0);
    highChassis.updateFromAccelerations(0, 6, zeroAero, dt, 0);
end

verifyLessThan(testCase, abs(highChassis.getRollAngle()), ...
    abs(lowChassis.getRollAngle()));
end

function testChassisLoadsChangeWithRollCenterHeight(testCase)
lowConfig = setRollCenters(lts.vehicles.baseline(), 0, 0, 0, 0);
highConfig = setRollCenters(lts.vehicles.baseline(), 0.15, 0.15, 0, 0);
[~, lowSuspension, lowChassis] = createVehicleWithChassis(lowConfig);
[~, highSuspension, highChassis] = createVehicleWithChassis(highConfig);
zeroAero = zeroAeroForces();
dt = 0.001;

lowChassis.updateFromAccelerations(0, 6, zeroAero, dt, 0);
highChassis.updateFromAccelerations(0, 6, zeroAero, dt, 0);
lowLoads = lowSuspension.computeCornerLoadsFromChassis(lowChassis, 0, dt);
highLoads = highSuspension.computeCornerLoadsFromChassis(highChassis, 0, dt);

verifyGreaterThan(testCase, ...
    abs(rightMinusLeft(highLoads) - rightMinusLeft(lowLoads)), 10);
end

function testRollCenterLateralContributesSignedChassisRollMoment(testCase)
zeroConfig = setRollCenters(lts.vehicles.baseline(), 0, 0, 0, 0);
lateralConfig = setRollCenters(lts.vehicles.baseline(), 0, 0, 0.05, 0.05);
[~, ~, zeroPositive] = createVehicleWithChassis(zeroConfig);
[~, ~, lateralPositive] = createVehicleWithChassis(lateralConfig);
[~, ~, zeroNegative] = createVehicleWithChassis(zeroConfig);
[~, ~, lateralNegative] = createVehicleWithChassis(lateralConfig);
zeroAero = zeroAeroForces();
dt = 0.001;
g = lts.vehicle.VehicleManager.g;

zeroPositive.updateFromAccelerations(0, g, zeroAero, dt, 0);
lateralPositive.updateFromAccelerations(0, g, zeroAero, dt, 0);
zeroNegative.updateFromAccelerations(0, -g, zeroAero, dt, 0);
lateralNegative.updateFromAccelerations(0, -g, zeroAero, dt, 0);

verifyGreaterThan(testCase, lateralPositive.state.frontRollAccel, ...
    zeroPositive.state.frontRollAccel);
verifyGreaterThan(testCase, lateralPositive.state.rearRollAccel, ...
    zeroPositive.state.rearRollAccel);
verifyLessThan(testCase, lateralNegative.state.frontRollAccel, ...
    zeroNegative.state.frontRollAccel);
verifyLessThan(testCase, lateralNegative.state.rearRollAccel, ...
    zeroNegative.state.rearRollAccel);
verifyEqual(testCase, lateralPositive.state.frontRollCenterLateral, 0.05, ...
    'AbsTol', 1e-12);
verifyEqual(testCase, lateralNegative.state.frontRollCenterLateral, -0.05, ...
    'AbsTol', 1e-12);
end

function testRollCenterLateralVisibleThroughManagerKinematics(testCase)
config = setRollCenters(lts.vehicles.baseline(), 0.03, 0.04, 0.011, 0.022);
[~, suspension] = createAlgebraicVehicle(config);

kin = suspension.getCornerKinematics();

verifyEqual(testCase, kin.FL.rollCenterLateral, 0.011, 'AbsTol', 1e-12);
verifyEqual(testCase, kin.FR.rollCenterLateral, 0.011, 'AbsTol', 1e-12);
verifyEqual(testCase, kin.RL.rollCenterLateral, 0.022, 'AbsTol', 1e-12);
verifyEqual(testCase, kin.RR.rollCenterLateral, 0.022, 'AbsTol', 1e-12);
end

function testChassisPathUsesTotalMassForLongitudinalTransfer(testCase)
[vehicle, suspension, chassis] = createVehicleWithChassis(lts.vehicles.R25());
zeroAero = zeroAeroForces();
dt = 0.001;
ax = 4;

for idx = 1:6000
    loads = suspension.computeCornerLoadsFromChassis(chassis, 0, dt);
    chassis.updateFromAccelerations(ax, 0, zeroAero, dt, 0);
end

staticFrontLoad = vehicle.totalMass * lts.vehicle.VehicleManager.g * ...
    vehicle.staticFrontWeight;
expectedTransfer = vehicle.totalMass * ax * vehicle.cgHeight / vehicle.wheelbase;
actualTransfer = staticFrontLoad - (loads.FL + loads.FR);
verifyEqual(testCase, actualTransfer, expectedTransfer, 'AbsTol', 2);
verifyEqual(testCase, totalNormalLoad(loads), ...
    vehicle.totalMass * lts.vehicle.VehicleManager.g, 'AbsTol', 2);
end

function testChassisPathUsesTotalMassForLateralTransfer(testCase)
config = setRollCenters(lts.vehicles.baseline(), 0, 0, 0, 0);
[vehicle, suspension, chassis] = createVehicleWithChassis(config);
zeroAero = zeroAeroForces();
dt = 0.001;
ay = 6;

for idx = 1:6000
    loads = suspension.computeCornerLoadsFromChassis(chassis, 0, dt);
    chassis.updateFromAccelerations(0, ay, zeroAero, dt, 0);
end

expectedRightMinusLeft = 2 * vehicle.totalMass * ay * ...
    vehicle.cgHeight / vehicle.trackWidth;
verifyEqual(testCase, rightMinusLeft(loads), expectedRightMinusLeft, 'AbsTol', 2);
verifyEqual(testCase, totalNormalLoad(loads), ...
    vehicle.totalMass * lts.vehicle.VehicleManager.g, 'AbsTol', 2);
end

function testOverAckermannIsNotClippedToIdeal(testCase)
config = lts.vehicles.R25();
vehicle = lts.vehicle.VehicleManager([], [], [], [], []);
vehicle.wheelbase = config.wheelbase;
vehicle.trackWidth = config.trackWidth;
vehicle.staticFrontWeight = config.staticFrontWeight;
overGeometry = lts.components.Suspension.SuspensionGeometry.fromConfig( ...
    config.suspension.geometry, vehicle);
idealGeometry = overGeometry;
idealGeometry.ackermann = 1;

overSteer = overGeometry.computeSteeringAngles(0.2);
idealSteer = idealGeometry.computeSteeringAngles(0.2);

verifyGreaterThan(testCase, overSteer.FL, idealSteer.FL);
verifyLessThan(testCase, overSteer.FR, idealSteer.FR);
end

function [vehicle, suspension, chassis] = createVehicleWithChassis(config)
% Minimal chassis+suspension fixture for unit tests. Sources all tuning
% values from the baseline car config (single source of truth) rather than
% re-declaring literals here, so these tests stay in sync with the config.
if nargin < 1 || isempty(config)
    config = lts.vehicles.baseline();
end
vehicle = minimalVehicleManager(config);

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
chassis = chassis.setSuspension(suspension);
suspension.chassis = chassis;
vehicle.chassis = chassis;
vehicle.suspension = suspension;
end

function vehicle = minimalVehicleManager(config)
vehicle = lts.vehicle.VehicleManager([], [], [], [], []);
vehicle.totalMass = config.totalMass;
vehicle.wheelbase = config.wheelbase;
vehicle.trackWidth = config.trackWidth;
vehicle.cgHeight = config.cgHeight;
vehicle.staticFrontWeight = config.staticFrontWeight;
end

function chassis = setTorsionalRigidity(chassis, value)
chassis.torsionalRigidity = value;
end

function [vehicle, suspension] = createAlgebraicVehicle(config)
vehicle = lts.vehicle.VehicleManager([], [], [], [], []);
vehicle.totalMass = config.totalMass;
vehicle.wheelbase = config.wheelbase;
vehicle.trackWidth = config.trackWidth;
vehicle.cgHeight = config.cgHeight;
vehicle.staticFrontWeight = config.staticFrontWeight;
geometry = lts.components.Suspension.SuspensionGeometry.fromConfig( ...
    config.suspension.geometry, vehicle);
suspension = createSuspension(vehicle, config.suspension, ...
    config.unsprungMass, geometry);
vehicle.suspension = suspension;
suspension.warmup(vehicle.totalMass, 0.001);
end

function config = setRollCenters(config, frontHeight, rearHeight, ...
        frontLateral, rearLateral)
config.suspension.geometry.front.rollCenterHeight = frontHeight;
config.suspension.geometry.rear.rollCenterHeight = rearHeight;
config.suspension.geometry.front.rollCenterLateral = frontLateral;
config.suspension.geometry.rear.rollCenterLateral = rearLateral;
end

function value = rightMinusLeft(loads)
value = (loads.FR + loads.RR) - (loads.FL + loads.RL);
end

function value = totalNormalLoad(loads)
value = loads.FL + loads.FR + loads.RL + loads.RR;
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
    'F_drag_longitudinal', 0, ...
    'F_drag_lateral', 0, ...
    'dragHeight', 0, ...
    'dragXPosition', 0);
end
