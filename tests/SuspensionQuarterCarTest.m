function tests = SuspensionQuarterCarTest
tests = functiontests(localfunctions);
end

function testStaticEquilibrium(testCase)
[~, suspension, chassis] = createChassisCoupledSuspension(1);
dt = 0.001;

loads = runCoupledSteps(suspension, chassis, 0, 0, 0, 50, dt);
loadValues = loadVector(loads);

verifyEqual(testCase, sum(loadValues), ...
    chassis.sprungMass * 9.80665 + 4 * suspension.frontLeft.unsprungMass * 9.80665, ...
    'AbsTol', 1e-6);
verifyLessThan(testCase, abs(suspension.computePitchAngle()), 1e-12);
verifyEqual(testCase, suspension.frontLeft.state.damperPosition, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, suspension.rearLeft.state.damperPosition, 0, 'AbsTol', 1e-12);
end

function testSpringDamperRatesAffectTransientNormalLoad(testCase)
[~, baseSuspension, baseChassis] = createChassisCoupledSuspension(1);
[~, stiffSuspension, stiffChassis] = createChassisCoupledSuspension(2);
dt = 0.001;

loadsBase = runCoupledSteps(baseSuspension, baseChassis, 4, 8, 0, 40, dt);
loadsStiff = runCoupledSteps(stiffSuspension, stiffChassis, 4, 8, 0, 40, dt);

loadDelta = norm(loadVector(loadsBase) - loadVector(loadsStiff));
verifyGreaterThan(testCase, loadDelta, 1e-3);
end

function testDigressiveDamperReducesHighSpeedForce(testCase)
% Below the configured knee the digressive damper matches the linear slope
% exactly; above it the force grows at the reduced high-speed ratio.
% Validated through the public quarter-car update path (computeDamperForce
% is private), comparing a linear corner (knee=Inf, ratio=1) against an
% otherwise identical digressive corner (knee=0.05, ratio=0.25).
cfg = lts.vehicles.baseline();
vm = lts.vehicle.VehicleManager([], [], [], [], []);
vm.totalMass = cfg.totalMass;
vm.wheelbase = cfg.wheelbase;
vm.trackWidth = cfg.trackWidth;
vm.cgHeight = cfg.cgHeight;
vm.staticFrontWeight = cfg.staticFrontWeight;
sprungCornerMass = (cfg.totalMass - 4 * cfg.unsprungMass) ...
    * cfg.staticFrontWeight / 2;
staticLoad = sprungCornerMass * 9.80665;

% --- Low shaft speed (below the knee): the two laws coincide. ---
lowSpeed = 0.02;   % < knee
linearLow = buildDamperCorner(vm, cfg, sprungCornerMass, Inf, 1.0);
digressiveLow = buildDamperCorner(vm, cfg, sprungCornerMass, 0.05, 0.25);
linearLow.initializeStaticLoad(linearLow.state, staticLoad);
digressiveLow.initializeStaticLoad(digressiveLow.state, staticLoad);
linearLow.updateCornerFromChassis(linearLow.state, 0, lowSpeed, 0.001, 0);
digressiveLow.updateCornerFromChassis(digressiveLow.state, 0, lowSpeed, 0.001, 0);
verifyEqual(testCase, digressiveLow.state.suspensionForce, ...
    linearLow.state.suspensionForce, 'AbsTol', 1e-6);

% --- High shaft speed (well beyond the knee): digressive force is bounded
%     well below the linear extrapolation. ---
highSpeed = 0.25;  % >> knee
linearHigh = buildDamperCorner(vm, cfg, sprungCornerMass, Inf, 1.0);
digressiveHigh = buildDamperCorner(vm, cfg, sprungCornerMass, 0.05, 0.25);
linearHigh.initializeStaticLoad(linearHigh.state, staticLoad);
digressiveHigh.initializeStaticLoad(digressiveHigh.state, staticLoad);
linearHigh.updateCornerFromChassis(linearHigh.state, 0, highSpeed, 0.001, 0);
digressiveHigh.updateCornerFromChassis(digressiveHigh.state, 0, highSpeed, 0.001, 0);

linearDamper = linearHigh.state.suspensionForce - linearHigh.state.staticLoad;
digressiveDamper = digressiveHigh.state.suspensionForce - digressiveHigh.state.staticLoad;
verifyGreaterThan(testCase, linearDamper, 0);
verifyLessThan(testCase, digressiveDamper, linearDamper);
% At 0.25 m/s with knee=0.05, ratio=0.25 the ideal force ratio is ~0.44;
% allow margin for the semi-implicit unsprung-mass integration.
verifyLessThan(testCase, digressiveDamper, 0.6 * linearDamper);
end

function corner = buildDamperCorner(vm, cfg, sprungCornerMass, knee, ratio)
corner = lts.components.Suspension.SimpleSuspension( ...
    vm, ...
    cfg.suspension.front.springRate, ...
    cfg.suspension.front.dampingCoeff, cfg.suspension.front.reboundCoeff, ...
    cfg.suspension.motionRatio, ...
    cfg.suspension.bumpStopLength, cfg.suspension.bumpStopRate, ...
    cfg.suspension.tireSpringRate, cfg.unsprungMass, sprungCornerMass, ...
    knee, ratio);
end

function testAntiRollBarCouplesLeftRightWheelTravel(testCase)
[~, suspension, chassis] = createChassisCoupledSuspension(1, 100000, 60000);

runCoupledSteps(suspension, chassis, 0, 8, 0, 80, 0.001);

frontForces = [
    suspension.frontLeft.state.antiRollBarForce
    suspension.frontRight.state.antiRollBarForce
];
rearForces = [
    suspension.rearLeft.state.antiRollBarForce
    suspension.rearRight.state.antiRollBarForce
];

verifyLessThan(testCase, frontForces(1), 0);
verifyGreaterThan(testCase, frontForces(2), 0);
verifyLessThan(testCase, rearForces(1), 0);
verifyGreaterThan(testCase, rearForces(2), 0);
verifyEqual(testCase, sum(frontForces), 0, 'AbsTol', 1e-9);
verifyEqual(testCase, sum(rearForces), 0, 'AbsTol', 1e-9);
end

function testPitchUsesSprungBodyPositionNotDamperDeflection(testCase)
[vehicle, suspension] = createSuspension(1);

suspension.frontLeft.state.sprungPosition = 0;
suspension.frontRight.state.sprungPosition = 0;
suspension.rearLeft.state.sprungPosition = 0.010;
suspension.rearRight.state.sprungPosition = 0.010;

% Keep suspension deflection at zero. The old damper-based pitch
% calculation would return zero for this state.
suspension.frontLeft.state.damperPosition = 0;
suspension.frontRight.state.damperPosition = 0;
suspension.rearLeft.state.damperPosition = 0;
suspension.rearRight.state.damperPosition = 0;

expectedPitch = atan2(0.010, vehicle.wheelbase);
verifyEqual(testCase, suspension.computePitchAngle(), expectedPitch, 'AbsTol', 1e-12);
end

function testExtremeUnloadIsFiniteAndNonnegative(testCase)
[~, suspension, chassis] = createChassisCoupledSuspension(1);

loads = runCoupledSteps(suspension, chassis, -25, 35, 0, 300, 0.001);
values = loadVector(loads);
verifyTrue(testCase, all(isfinite(values)));
verifyTrue(testCase, all(values >= 0));

stateValues = [
    suspension.frontLeft.state.sprungPosition
    suspension.frontLeft.state.sprungVelocity
    suspension.frontLeft.state.unsprungPosition
    suspension.frontLeft.state.unsprungVelocity
    suspension.frontRight.state.sprungPosition
    suspension.frontRight.state.sprungVelocity
    suspension.frontRight.state.unsprungPosition
    suspension.frontRight.state.unsprungVelocity
    suspension.rearLeft.state.sprungPosition
    suspension.rearLeft.state.sprungVelocity
    suspension.rearLeft.state.unsprungPosition
    suspension.rearLeft.state.unsprungVelocity
    suspension.rearRight.state.sprungPosition
    suspension.rearRight.state.sprungVelocity
    suspension.rearRight.state.unsprungPosition
    suspension.rearRight.state.unsprungVelocity
];
verifyTrue(testCase, all(isfinite(stateValues)));
end

function testGeometryTelemetryStillUpdates(testCase)
[~, suspension, chassis] = createChassisCoupledSuspension(1);

runCoupledSteps(suspension, chassis, 2, 4, 0.25, 20, 0.001);

kin = suspension.getCornerKinematics();
values = [
    kin.FL.wheelTravel
    kin.FR.wheelTravel
    kin.RL.wheelTravel
    kin.RR.wheelTravel
    kin.FL.camberAngle
    kin.FR.camberAngle
    kin.FL.toeAngle
    kin.FR.toeAngle
    kin.FL.steerAngle
    kin.FR.steerAngle
    kin.FL.motionRatio
    kin.FR.motionRatio
];

verifyTrue(testCase, all(isfinite(values)));
verifyGreaterThan(testCase, kin.FL.motionRatio, 0);
verifyGreaterThan(testCase, abs(kin.FL.steerAngle), 0);
end

function testSteeringAxisGeometryAffectsCamberAndContactPatch(testCase)
[~, suspension] = createSuspension(1);

geometry = suspension.geometry;
geometry.frontCasterAngle = 8 * pi / 180;
geometry.frontKingpinInclination = 10 * pi / 180;
geometry.frontMechanicalTrail = 0.035;
geometry.frontScrubRadius = 0.020;
geometry.frontKingpinOffset = 0.020;
suspension.geometry = geometry;

suspension.updateGeometry(0);
kinZero = suspension.getCornerKinematics();
suspension.updateGeometry(0.25);
kin = suspension.getCornerKinematics();

verifyGreaterThan(testCase, kin.FL.camberAngle, kinZero.FL.camberAngle);
verifyLessThan(testCase, kin.FR.camberAngle, kinZero.FR.camberAngle);
verifyGreaterThan(testCase, hypot( ...
    kin.FL.xPosition - kinZero.FL.xPosition, ...
    kin.FL.yPosition - kinZero.FL.yPosition), 1e-4);
verifyEqual(testCase, kinZero.FL.xPosition, kinZero.FL.wheelCenterXPosition, ...
    'AbsTol', 1e-12);
verifyEqual(testCase, kinZero.FL.yPosition, kinZero.FL.wheelCenterYPosition, ...
    'AbsTol', 1e-12);
verifyEqual(testCase, kin.FL.mechanicalTrail, 0.035, 'AbsTol', 1e-12);
verifyEqual(testCase, kin.FL.scrubRadius, 0.020, 'AbsTol', 1e-12);
verifyEqual(testCase, kin.FL.kingpinInclination, 10 * pi / 180, ...
    'AbsTol', 1e-12);
end

function testQuarterCarTelemetryExportsToCsv(testCase)
stateLog = createTelemetryStateLog();
testDir = fileparts(mfilename('fullpath'));
csvFile = fullfile(testDir, 'telemetry_export_test.csv');
cleanup = onCleanup(@() deleteIfExists(csvFile));

lts.telemetry.TelemetryExporter.writeToMoTeCFormat(stateLog, csvFile);
header = string(readlines(csvFile));
header = header(1);

verifyTrue(testCase, contains(header, "Suspension Force FL (N)"));
verifyTrue(testCase, contains(header, "Anti Roll Bar Force FL (N)"));
verifyTrue(testCase, contains(header, "Suspension Demand FR (N)"));
verifyTrue(testCase, contains(header, "Tire Deflection RL (mm)"));
verifyTrue(testCase, contains(header, "Sprung Position RR (mm)"));
verifyTrue(testCase, contains(header, "Unsprung Position FL (mm)"));
verifyTrue(testCase, contains(header, "Sprung Vel FR (mm/s)"));
verifyTrue(testCase, contains(header, "Unsprung Vel RL (mm/s)"));
verifyTrue(testCase, contains(header, "Body Slip Angle (deg)"));
verifyTrue(testCase, contains(header, "Engine RPM (rpm)"));
verifyTrue(testCase, contains(header, "Motor RPM (rpm)"));
verifyTrue(testCase, contains(header, "Wheel Speed Front Left Sensor Linear (m/s)"));

headers = split(header, ",");
tireSpeedFLCol = find(headers == "Wheel Speed Front Left Sensor Linear (m/s)", 1);
verifyNotEmpty(testCase, tireSpeedFLCol);
data = readmatrix(csvFile, 'NumHeaderLines', 1);
expectedTireSpeed = stateLog.omega_FL(2) * stateLog.wheelRadius_FL(2);
verifyEqual(testCase, data(2, tireSpeedFLCol), expectedTireSpeed, 'AbsTol', 1e-8);
end

function testDynamicBumpStopContributesToRollStiffness(testCase)
config = lts.vehicles.R25();
vehicle = lts.vehicle.VehicleManager([], [], [], [], []);
vehicle.totalMass = config.totalMass;
vehicle.wheelbase = config.wheelbase;
vehicle.trackWidth = config.trackWidth;
vehicle.cgHeight = config.cgHeight;
vehicle.staticFrontWeight = config.staticFrontWeight;
geometry = lts.components.Suspension.SuspensionGeometry.fromConfig( ...
    config.suspension.geometry, vehicle);
frontArb = config.suspension.frontArb;
rearArb = config.suspension.rearArb;
geometry.frontAntiRollBar = lts.components.Suspension.AntiRollBar( ...
    frontArb.stiffness, frontArb.motionRatio, frontArb.leverArm, frontArb.enabled);
geometry.rearAntiRollBar = lts.components.Suspension.AntiRollBar( ...
    rearArb.stiffness, rearArb.motionRatio, rearArb.leverArm, rearArb.enabled);

suspension = lts.components.Suspension.SuspensionManager( ...
    vehicle, ...
    config.suspension.front.springRate, config.suspension.front.dampingCoeff, config.suspension.front.reboundCoeff, ...
    config.suspension.rear.springRate,  config.suspension.rear.dampingCoeff,  config.suspension.rear.reboundCoeff, ...
    config.suspension.motionRatio, ...
    config.suspension.bumpStopLength, ...
    config.suspension.bumpStopRate, ...
    config.suspension.tireSpringRate, ...
    config.unsprungMass, ...
    geometry);
vehicle.suspension = suspension;
suspension.warmup(vehicle.totalMass, 0.001);

frontNoStop = config.suspension.front.springRate + ...
    2 * geometry.frontAntiRollBar.getWheelRateStiffness();
rearNoStop = config.suspension.rear.springRate + ...
    2 * geometry.rearAntiRollBar.getWheelRateStiffness();

[KwF, KwR] = suspension.getAxleRollStiffness();
verifyEqual(testCase, KwF, frontNoStop, 'AbsTol', 1e-9);
verifyEqual(testCase, KwR, rearNoStop, 'AbsTol', 1e-9);

% bumpStopLength is free travel from static ride height. Static spring
% compression alone must not engage it; dynamic damper travel must cross it.
engagedTravel = config.suspension.bumpStopLength + 1e-3;
suspension.frontLeft.state.damperPosition = engagedTravel;
suspension.rearLeft.state.damperPosition = engagedTravel;
[KwF, KwR] = suspension.getAxleRollStiffness();

verifyEqual(testCase, KwF, ...
    frontNoStop + 0.5 * config.suspension.bumpStopRate, 'AbsTol', 1e-9);
verifyEqual(testCase, KwR, ...
    rearNoStop + 0.5 * config.suspension.bumpStopRate, 'AbsTol', 1e-9);
end

function testAxleRollStiffnessMatchesAntiRollBarForceMoment(testCase)
config = lts.vehicles.baseline();
[vehicle, suspension] = createSuspension(1, 0, 0, config);
[baseKwF, ~] = suspension.getAxleRollStiffness();

barRate = 1000;
suspension.frontAntiRollBarWheelRate = barRate;
[barKwF, ~] = suspension.getAxleRollStiffness();
verifyEqual(testCase, barKwF - baseKwF, 2 * barRate, 'AbsTol', 1e-12);

phi = 0.02;
halfTravel = vehicle.trackWidth * phi / 2;
suspension.frontLeft.state.damperPosition = -halfTravel;
suspension.frontRight.state.damperPosition = halfTravel;
barForces = suspension.getAntiRollBarForces();
actualMoment = (barForces.FR - barForces.FL) * vehicle.trackWidth / 2;
equivalentMoment = (barKwF - baseKwF) * vehicle.trackWidth^2 / 2 * phi;
verifyEqual(testCase, actualMoment, equivalentMoment, 'AbsTol', 1e-10);
end

function testWheelDomainTravelIsNotDividedByMotionRatioTwice(testCase)
config = lts.vehicles.baseline();
config.suspension.motionRatio = 0.5;
config.suspension.geometry.front.motionRatioCurve = [0.5 0.5 0.5];
config.suspension.geometry.rear.motionRatioCurve = [0.5 0.5 0.5];
[~, suspension] = createSuspension(1, 0, 0, config);
suspension.frontLeft.state.damperPosition = 0.01;
suspension.frontRight.state.damperPosition = 0;
suspension.frontAntiRollBarWheelRate = 1000;

suspension.updateGeometry(0);
barForces = suspension.getAntiRollBarForces();

verifyEqual(testCase, suspension.frontLeft.state.wheelTravel, ...
    0.01, 'AbsTol', 1e-12);
verifyEqual(testCase, barForces.FL, 10, 'AbsTol', 1e-12);
verifyEqual(testCase, barForces.FR, -10, 'AbsTol', 1e-12);
end

function testReboundKneeOverrideAffectsReboundOnly(testCase)
% The rebound knee override must change only the rebound side: at a
% rebound speed beyond both knees the higher-knee corner keeps its
% low-speed slope longer (more force), while compression remains shared.
cfg = lts.vehicles.baseline();
vm = lts.vehicle.VehicleManager([], [], [], [], []);
vm.totalMass = cfg.totalMass;
vm.wheelbase = cfg.wheelbase;
vm.trackWidth = cfg.trackWidth;
vm.cgHeight = cfg.cgHeight;
vm.staticFrontWeight = cfg.staticFrontWeight;
sprungCornerMass = (cfg.totalMass - 4 * cfg.unsprungMass) ...
    * cfg.staticFrontWeight / 2;
staticLoad = sprungCornerMass * 9.80665;

lowReboundKnee = buildDamperCorner(vm, cfg, sprungCornerMass, 0.05, 0.25);
highReboundKnee = buildDamperCorner(vm, cfg, sprungCornerMass, 0.05, 0.25);
lowReboundKnee.dampingReboundKneeSpeed = 0.05;
highReboundKnee.dampingReboundKneeSpeed = 0.30;

lowReboundKnee.initializeStaticLoad(lowReboundKnee.state, staticLoad);
highReboundKnee.initializeStaticLoad(highReboundKnee.state, staticLoad);
lowReboundKnee.updateCornerFromChassis(lowReboundKnee.state, 0, -0.25, 0.001, 0);
highReboundKnee.updateCornerFromChassis(highReboundKnee.state, 0, -0.25, 0.001, 0);

lowReboundForce = abs(lowReboundKnee.state.suspensionForce - staticLoad);
highReboundForce = abs(highReboundKnee.state.suspensionForce - staticLoad);
verifyGreaterThan(testCase, highReboundForce, lowReboundForce, ...
    'higher rebound knee must sustain more low-speed-slope force in rebound');

lowReboundKnee.initializeStaticLoad(lowReboundKnee.state, staticLoad);
highReboundKnee.initializeStaticLoad(highReboundKnee.state, staticLoad);
lowReboundKnee.updateCornerFromChassis(lowReboundKnee.state, 0, 0.25, 0.001, 0);
highReboundKnee.updateCornerFromChassis(highReboundKnee.state, 0, 0.25, 0.001, 0);
verifyEqual(testCase, highReboundKnee.state.suspensionForce, ...
    lowReboundKnee.state.suspensionForce, 'AbsTol', 1e-9, ...
    'compression side must ignore the rebound knee override');
end

function testForceElementAddsItsCurveToSuspensionForce(testCase)
% A pluggable TravelCurveElement must add exactly its lookup value to the
% corner's suspension force, and an empty element list must leave the
% built-in force law untouched.
cfg = lts.vehicles.baseline();
vm = lts.vehicle.VehicleManager([], [], [], [], []);
vm.totalMass = cfg.totalMass;
vm.wheelbase = cfg.wheelbase;
vm.trackWidth = cfg.trackWidth;
vm.cgHeight = cfg.cgHeight;
vm.staticFrontWeight = cfg.staticFrontWeight;
sprungCornerMass = (cfg.totalMass - 4 * cfg.unsprungMass) ...
    * cfg.staticFrontWeight / 2;
staticLoad = sprungCornerMass * 9.80665;

plain = buildDamperCorner(vm, cfg, sprungCornerMass, Inf, 1.0);
withElement = buildDamperCorner(vm, cfg, sprungCornerMass, Inf, 1.0);
element = lts.components.Suspension.TravelCurveElement( ...
    'travelGrid', [-0.05 0 0.020 0.030 0.05], ...
    'forceGrid', [0 0 0 600 1800]);
withElement.forceElements = {element};

plain.initializeStaticLoad(plain.state, staticLoad);
withElement.initializeStaticLoad(withElement.state, staticLoad);

% Impose 25 mm compression with a very small step: after one semi-implicit
% update the unsprung velocity (and therefore the damper force) scales
% with dt, so the force difference isolates the element's curve value.
% At 25 mm the element is halfway up its 20-30 mm ramp: 300 N.
plain.updateCornerFromChassis(plain.state, 0.025, 0, 1e-5, 0);
withElement.updateCornerFromChassis(withElement.state, 0.025, 0, 1e-5, 0);
verifyEqual(testCase, withElement.state.suspensionForce - ...
    plain.state.suspensionForce, 300, 'AbsTol', 2);

% At 10 mm compression (before engagement) the element adds nothing.
plain.initializeStaticLoad(plain.state, staticLoad);
withElement.initializeStaticLoad(withElement.state, staticLoad);
plain.updateCornerFromChassis(plain.state, 0.010, 0, 1e-5, 0);
withElement.updateCornerFromChassis(withElement.state, 0.010, 0, 1e-5, 0);
verifyEqual(testCase, withElement.state.suspensionForce, ...
    plain.state.suspensionForce, 'AbsTol', 1e-6);
end

function [vehicle, suspension] = createSuspension(rateScale, ...
        frontAntiRollBarRate, rearAntiRollBarRate, config)
if nargin < 2
    frontAntiRollBarRate = 0;
end
if nargin < 3
    rearAntiRollBarRate = 0;
end

% Source geometry + vehicle constants from the baseline config so the
% fixture stays in sync with the car definition.
if nargin < 4 || isempty(config)
    config = lts.vehicles.baseline();
end
vehicle = lts.vehicle.VehicleManager([], [], [], [], []);
vehicle.totalMass = config.totalMass;
vehicle.wheelbase = config.wheelbase;
vehicle.trackWidth = config.trackWidth;
vehicle.cgHeight = config.cgHeight;
vehicle.staticFrontWeight = config.staticFrontWeight;
geometry = lts.components.Suspension.SuspensionGeometry.fromConfig( ...
    config.suspension.geometry, vehicle);
suspension = lts.components.Suspension.SuspensionManager( ...
    vehicle, ...
    config.suspension.front.springRate * rateScale, config.suspension.front.dampingCoeff * rateScale, config.suspension.front.reboundCoeff * rateScale, ...
    config.suspension.rear.springRate * rateScale,  config.suspension.rear.dampingCoeff * rateScale,  config.suspension.rear.reboundCoeff * rateScale, ...
    config.suspension.motionRatio, ...
    config.suspension.bumpStopLength, ...
    config.suspension.bumpStopRate, ...
    config.suspension.tireSpringRate, ...
    config.unsprungMass, ...
    geometry, ...
    frontAntiRollBarRate, ...
    rearAntiRollBarRate);
vehicle.suspension = suspension;
suspension.warmup(vehicle.totalMass, 0.001);
end

function [vehicle, suspension, chassis] = createChassisCoupledSuspension( ...
        rateScale, frontAntiRollBarRate, rearAntiRollBarRate, config)
% Chassis-coupled fixture mirroring the Simulator's load loop: the chassis
% resolves sprung attitude, the suspension reacts through the corner
% spring/damper/ARB forces.
if nargin < 2
    frontAntiRollBarRate = 0;
end
if nargin < 3
    rearAntiRollBarRate = 0;
end
if nargin < 4 || isempty(config)
    config = lts.vehicles.baseline();
end

[vehicle, suspension] = createSuspension(rateScale, ...
    frontAntiRollBarRate, rearAntiRollBarRate, config);
sprungMass = vehicle.totalMass - 4 * config.unsprungMass;
chassis = lts.components.Chassis.SimpleChassis(vehicle, sprungMass);
chassis.heaveStiffness = config.chassis.heaveStiffness;
chassis.heaveDamping = config.chassis.heaveDamping;
chassis.pitchStiffness = config.chassis.pitchStiffness;
chassis.pitchDamping = config.chassis.pitchDamping;
chassis.rollStiffness = config.chassis.rollStiffness;
chassis.rollDamping = config.chassis.rollDamping;
chassis.torsionalRigidity = config.chassis.torsionalRigidity;
chassis.torsionalDamping = config.chassis.torsionalDamping;
chassis = chassis.setSuspension(suspension);
suspension.chassis = chassis;
vehicle.chassis = chassis;
end

function loads = runCoupledSteps(suspension, chassis, ax, ay, steer, nSteps, dt)
% Advance the coupled chassis+suspension loop (order matches
% ChassisLoadTransferTest: chassis attitude first, corner loads from it).
aeroForces = zeroAeroForces();
loads = [];
for idx = 1:nSteps
    chassis.updateFromAccelerations(ax, ay, aeroForces, dt, 0);
    loads = suspension.computeCornerLoadsFromChassis(chassis, steer, dt);
end
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

function values = loadVector(loads)
values = [loads.FL; loads.FR; loads.RL; loads.RR];
end

function stateLog = createTelemetryStateLog()
n = 3;
stateLog = struct();
stateLog.time = (0:n-1)' * 0.001;
stateLog.s = (0:n-1)';
stateLog.speedKmh = [0; 10; 20];
stateLog.bodySlipAngle = [0; 0.02; -0.03];
stateLog.motorRPM = [1000; 1100; 1200];
stateLog.drivenWheelRPM = [300; 320; 340];
stateLog.Fz_FL = [600; 610; 620];
stateLog.Fz_FR = [600; 590; 580];
stateLog.Fz_RL = [700; 710; 720];
stateLog.Fz_RR = [700; 690; 680];

corners = {'FL', 'FR', 'RL', 'RR'};
for idx = 1:numel(corners)
    corner = corners{idx};
    stateLog.(sprintf('suspensionForce_%s', corner)) = (idx:idx+n-1)' * 100;
    stateLog.(sprintf('antiRollBarForce_%s', corner)) = (idx:idx+n-1)' * 10;
    stateLog.(sprintf('suspensionDemand_%s', corner)) = (idx:idx+n-1)' * 110;
    stateLog.(sprintf('tireDeflection_%s', corner)) = (idx:idx+n-1)' * 0.001;
    stateLog.(sprintf('damperPos_%s', corner)) = (idx:idx+n-1)' * 0.002;
    stateLog.(sprintf('damperVel_%s', corner)) = (idx:idx+n-1)' * 0.01;
    stateLog.(sprintf('sprungPosition_%s', corner)) = (idx:idx+n-1)' * 0.003;
    stateLog.(sprintf('unsprungPosition_%s', corner)) = (idx:idx+n-1)' * 0.004;
    stateLog.(sprintf('sprungVelocity_%s', corner)) = (idx:idx+n-1)' * 0.02;
    stateLog.(sprintf('unsprungVelocity_%s', corner)) = (idx:idx+n-1)' * 0.03;
    stateLog.(sprintf('wheelTravel_%s', corner)) = (idx:idx+n-1)' * 0.005;
    stateLog.(sprintf('camber_%s', corner)) = (idx:idx+n-1)' * 0.01;
    stateLog.(sprintf('toe_%s', corner)) = (idx:idx+n-1)' * 0.001;
    stateLog.(sprintf('wheelSteer_%s', corner)) = (idx:idx+n-1)' * 0.02;
    stateLog.(sprintf('omega_%s', corner)) = (idx:idx+n-1)' * 10;
    stateLog.(sprintf('wheelRadius_%s', corner)) = ones(n, 1) * (0.24 + idx * 0.01);
end
end

function deleteIfExists(filepath)
if exist(filepath, 'file')
    delete(filepath);
end
end
