function tests = SuspensionQuarterCarTest
tests = functiontests(localfunctions);
end

function testStaticEquilibrium(testCase)
[vehicle, suspension] = createSuspension(1);
state = createState(0, 0, 0);

loads = suspension.computeCornerLoads(state, 0, 0, vehicle.totalMass, 0.001);
loadValues = loadVector(loads);

verifyEqual(testCase, sum(loadValues), vehicle.totalMass * 9.81, 'AbsTol', 1e-9);
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

function testQuarterCarTelemetryExportsToCsv(testCase)
stateLog = createTelemetryStateLog();
testDir = fileparts(mfilename('fullpath'));
csvFile = fullfile(testDir, 'telemetry_export_test.csv');
cleanup = onCleanup(@() deleteIfExists(csvFile));

TelemetryExporter.writeToMoTeCFormat(stateLog, csvFile);
header = string(readlines(csvFile));
header = header(1);

verifyTrue(testCase, contains(header, "Suspension Force FL (N)"));
verifyTrue(testCase, contains(header, "Suspension Demand FR (N)"));
verifyTrue(testCase, contains(header, "Tire Deflection RL (mm)"));
verifyTrue(testCase, contains(header, "Sprung Position RR (mm)"));
verifyTrue(testCase, contains(header, "Unsprung Position FL (mm)"));
verifyTrue(testCase, contains(header, "Sprung Vel FR (mm/s)"));
verifyTrue(testCase, contains(header, "Unsprung Vel RL (mm/s)"));
verifyTrue(testCase, contains(header, "Body Slip Angle (deg)"));
end

function [vehicle, suspension] = createSuspension(rateScale)
vehicle = VehicleManager([], [], [], [], []);
geometry = components.Suspension.SuspensionGeometry.fromPreset('baseline', vehicle);
suspension = components.Suspension.SuspensionManager( ...
    vehicle, ...
    0.55, ...
    45000 * rateScale, 3000 * rateScale, 4500 * rateScale, ...
    42000 * rateScale, 2800 * rateScale, 4200 * rateScale, ...
    0.95, ...
    0.025, ...
    200000, ...
    200000, ...
    25, ...
    geometry);
vehicle.suspension = suspension;
suspension.warmup(vehicle.totalMass, 0.001);
end

function state = createState(ax, ay, steer)
state = VehicleState('speed', 20);
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
stateLog.Fz_FL = [600; 610; 620];
stateLog.Fz_FR = [600; 590; 580];
stateLog.Fz_RL = [700; 710; 720];
stateLog.Fz_RR = [700; 690; 680];

corners = {'FL', 'FR', 'RL', 'RR'};
for idx = 1:numel(corners)
    corner = corners{idx};
    stateLog.(sprintf('suspensionForce_%s', corner)) = (idx:idx+n-1)' * 100;
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
end
end

function deleteIfExists(filepath)
if exist(filepath, 'file')
    delete(filepath);
end
end
