function tests = SuspensionQuarterCarTest
tests = functiontests(localfunctions);
end

function testStaticEquilibrium(testCase)
[vehicle, suspension] = createSuspension(1);
state = createState(0, 0, 0);

loads = suspension.computeCornerLoads(state, 0, 0, vehicle.totalMass, 0.001);
loadValues = loadVector(loads);

verifyEqual(testCase, sum(loadValues), vehicle.totalMass * vehicle.g, 'AbsTol', 1e-9);
verifyLessThan(testCase, abs(suspension.computePitchAngle()), 1e-12);
verifyEqual(testCase, suspension.frontLeft.state.damperPosition, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, suspension.rearLeft.state.damperPosition, 0, 'AbsTol', 1e-12);
end

function testSpringDamperRatesAffectTransientNormalLoad(testCase)
[vehicleBase, baseSuspension] = createSuspension(1);
[vehicleStiff, stiffSuspension] = createSuspension(2);
stateBase = createState(4, 8, 0.2);
stateStiff = createState(4, 8, 0.2);

for idx = 1:40
    loadsBase = baseSuspension.computeCornerLoads( ...
        stateBase, 100, 150, vehicleBase.totalMass, 0.001);
    loadsStiff = stiffSuspension.computeCornerLoads( ...
        stateStiff, 100, 150, vehicleStiff.totalMass, 0.001);
end

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
    vm, 0.5, ...
    cfg.suspension.front.springRate, ...
    cfg.suspension.front.dampingCoeff, cfg.suspension.front.reboundCoeff, ...
    cfg.suspension.motionRatio, ...
    cfg.suspension.bumpStopLength, cfg.suspension.bumpStopRate, ...
    cfg.suspension.tireSpringRate, cfg.unsprungMass, sprungCornerMass, ...
    knee, ratio);
end

function testAntiRollBarCouplesLeftRightWheelTravel(testCase)
[vehicle, suspension] = createSuspension(1, 100000, 60000);
state = createState(0, 8, 0.1);

for idx = 1:80
    suspension.computeCornerLoads(state, 0, 0, vehicle.totalMass, 0.001);
end

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

function testConstantDemandSettlesToLoadTransferTarget(testCase)
[vehicle, suspension] = createSuspension(1);
state = createState(3, 6, 0.15);
targetLoads = suspension.estimateCornerLoads(state, 120, 80, vehicle.totalMass);

for idx = 1:6000
    loads = suspension.computeCornerLoads(state, 120, 80, vehicle.totalMass, 0.001);
end

verifyEqual(testCase, loadVector(loads), loadVector(targetLoads), 'AbsTol', 5);
end

function testExtremeUnloadIsFiniteAndNonnegative(testCase)
[vehicle, suspension] = createSuspension(1);
state = createState(-25, 35, 0.3);

for idx = 1:300
    loads = suspension.computeCornerLoads(state, 0, 0, vehicle.totalMass, 0.001);
    values = loadVector(loads);
    verifyTrue(testCase, all(isfinite(values)));
    verifyTrue(testCase, all(values >= 0));
end

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
[vehicle, suspension] = createSuspension(1);
state = createState(2, 4, 0.25);

for idx = 1:20
    suspension.computeCornerLoads(state, 80, 100, vehicle.totalMass, 0.001);
end

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
    config.suspension.rollStiffnessOverride, ...
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
    config.suspension.rollStiffnessOverride, ...
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

function state = createState(ax, ay, steer)
state = lts.simulation.VehicleState('speed', 20);
state.ax = ax;
state.ay = ay;
state.steer = steer;
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
