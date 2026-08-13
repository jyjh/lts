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

function testDerivedHeaveStiffnessMatchesSuspensionSprings(testCase)
% When heaveStiffness is NaN (derive), the chassis should settle to the
% same heave equilibrium as if the derived value were set explicitly:
% at steady state with constant downforce, heave_eq = Fz / Kheave.
config = lts.vehicles.baseline();
[~, suspension, chassis] = createVehicleWithChassis(config);
zeroAero = zeroAeroForces();
dt = 0.001;

% Symmetric downforce step (equal front/rear to minimize pitch coupling).
FzStep = 2000;
aeroStep = struct('Fz_front', FzStep / 2, 'Fz_rear', FzStep / 2, ...
    'F_drag', 0, 'dragHeight', 0);

for idx = 1:5000
    chassis.updateFromAccelerations(0, 0, aeroStep, dt, 0);
end

% Compute expected Kheave from the suspension springs + motion ratios.
susp = suspension;
mrF = susp.frontLeft.motionRatio;
if susp.frontLeft.state.motionRatioEffective > 0
    mrF = susp.frontLeft.state.motionRatioEffective;
end
mrR = susp.rearLeft.motionRatio;
if susp.rearLeft.state.motionRatioEffective > 0
    mrR = susp.rearLeft.state.motionRatioEffective;
end
expectedKheave = 2 * (susp.frontLeft.springRate * mrF^2 + ...
                      susp.rearLeft.springRate * mrR^2);

expectedHeave = FzStep / expectedKheave;
verifyGreaterThan(testCase, expectedKheave, 0);
verifyTrue(testCase, isfinite(chassis.state.heave));
verifyEqual(testCase, chassis.state.heave, expectedHeave, 'RelTol', 0.02);
end

function testExplicitHeaveStiffnessOverrideIsRespected(testCase)
% A finite heaveStiffness override must be used instead of the derived
% value. A very stiff override should produce less heave than the derived.
configDerived = lts.vehicles.baseline();
configStiff = lts.vehicles.baseline();
configStiff.chassis.heaveStiffness = 1e7;

[~, ~, derivedChassis] = createVehicleWithChassis(configDerived);
[~, ~, stiffChassis] = createVehicleWithChassis(configStiff);

FzStep = 2000;
aeroStep = struct('Fz_front', FzStep / 2, 'Fz_rear', FzStep / 2, ...
    'F_drag', 0, 'dragHeight', 0);
dt = 0.001;

for idx = 1:5000
    derivedChassis.updateFromAccelerations(0, 0, aeroStep, dt, 0);
    stiffChassis.updateFromAccelerations(0, 0, aeroStep, dt, 0);
end

verifyLessThan(testCase, abs(stiffChassis.state.heave), ...
    abs(derivedChassis.state.heave));
end

function testInfTorsionalRigidityIsRejected(testCase)
config = lts.vehicles.baseline();
config.chassis.torsionalRigidity = Inf;
[~, ~, chassis] = createVehicleWithChassis(config);

verifyError(testCase, @() chassis.updateFromAccelerations( ...
    0, 0, zeroAeroForces(), 0.001, 0), ...
    'lts_components_Chassis_SimpleChassis:InfTorsionalRigidity');
end

function [vehicle, suspension, chassis] = createVehicleWithChassis(config)
% Minimal chassis+suspension fixture for unit tests. Sources all tuning
% values from the baseline car config (single source of truth) rather than
% re-declaring literals here, so these tests stay in sync with the config.
if nargin < 1 || isempty(config)
    config = lts.vehicles.baseline();
end
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
chassis = chassis.setSuspension(suspension);
suspension.chassis = chassis;
vehicle.chassis = chassis;
vehicle.suspension = suspension;
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
    'dragHeight', 0);
end
