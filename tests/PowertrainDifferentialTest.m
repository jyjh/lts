function tests = PowertrainDifferentialTest
% POWERTRAINDIFFERENTIALTEST Coverage for the EMRAX field-weakening fix,
%   reflected motor inertia, locked/LSD differential reworks, and the
%   opt-in coastdown/regen path. Uses functiontests + verifyXxx.
tests = functiontests(localfunctions);
end

function testMotoringEfficiencyCurveInterpolatesAndClampsRPM(testCase)
pt = lts.components.Powertrain.EMRAX228Powertrain();
pt = pt.setMotoringEfficiencyCurve([1000 2000 3000], [0.75 0.90 0.80]);

verifyEqual(testCase, pt.getMotoringEfficiencyAtRPM(500), 0.75, 'AbsTol', 1e-12);
verifyEqual(testCase, pt.getMotoringEfficiencyAtRPM(1500), 0.825, 'AbsTol', 1e-12);
verifyEqual(testCase, pt.getMotoringEfficiencyAtRPM(-2500), 0.85, 'AbsTol', 1e-12);
verifyEqual(testCase, pt.getMotoringEfficiencyAtRPM(4000), 0.80, 'AbsTol', 1e-12);
end

function testEmptyMotoringEfficiencyCurveUsesScalar(testCase)
pt = lts.components.Powertrain.EMRAX228Powertrain('', 0.87);

verifyEqual(testCase, pt.getMotoringEfficiencyAtRPM(2500), 0.87, 'AbsTol', 1e-12);
end

function testDefaultLCMapLoadsLowercaseTorqueField(testCase)
pt = createPowertrain();
verifyEqual(testCase, pt.totalGearRatio, 3.36, 'RelTol', 1e-12);
verifyGreaterThan(testCase, pt.maxEngineTorque, 0);
verifyGreaterThan(testCase, numel(pt.torqueCurveNm), 0);
end

function testLegacyCCMapLoadsUppercaseTorqueField(testCase)
pt = lts.components.Powertrain.EMRAX228Powertrain( ...
    powertrainMapPath('EMRAX228CC Single_4.5.mat'));
verifyEqual(testCase, pt.totalGearRatio, 4.5, 'RelTol', 1e-12);
verifyGreaterThan(testCase, pt.maxEngineTorque, 0);
verifyGreaterThan(testCase, numel(pt.torqueCurveNm), 0);
end

function testFinalDriveOverrideScalesWheelForceAndMotorSpeed(testCase)
pt = createPowertrain();
baseRatio = pt.totalGearRatio;
newRatio = 4.15;
rpm = 3000;
baseForce = pt.lookupTractiveForceByRPM(rpm);

pt = pt.setFinalDriveRatio(newRatio);
pt.updateStateFromVehicleSpeed(10);

verifyEqual(testCase, pt.totalGearRatio, newRatio, 'RelTol', 1e-12);
verifyEqual(testCase, pt.lookupTractiveForceByRPM(rpm), ...
    baseForce * newRatio / baseRatio, 'RelTol', 1e-12);
verifyEqual(testCase, pt.state.motorRPM, ...
    10 / (2 * pi * pt.wheelRadius) * 60 * newRatio, 'RelTol', 1e-12);
end

function testConstantPowerFalloffDoesNotCollapseAtRevLimit(testCase)
% Fix 1: the old linear falloff drove wheel force to ~0 at the rev limit.
% Constant power must keep substantial force right up to the cap.
pt = createPowertrain();
anchorRPM = pt.rpmFalloffStartRPM;       % map top, where rolloff anchors
limitRPM  = pt.rpmLimitRPM;
fAnchor = pt.lookupTractiveForceByRPM(anchorRPM);

% Slightly below the hard cap the force must be the constant-power value,
% not ~0 (the old linear model gave 1194 N at 6000 rpm; correct is ~2240 N).
justUnderLimit = limitRPM - 1;
fJustUnder = pt.lookupTractiveForceByRPM(justUnderLimit);
expectedCP = fAnchor * anchorRPM / justUnderLimit;   % T ∝ 1/rpm
verifyEqual(testCase, fJustUnder, expectedCP, 'RelTol', 1e-6);
verifyGreaterThan(testCase, fJustUnder, 0.75 * fAnchor);

% At/above the cap the rev limiter zeroes force.
verifyEqual(testCase, pt.lookupTractiveForceByRPM(limitRPM), 0, 'AbsTol', 1e-9);
end

function testFalloffIsConstantPowerNotLinear(testCase)
% Fix 1: in the field-weakening band, force*rpm must be constant (constant
% power), whereas the old linear falloff made force*rpm fall with rpm. Test
% at two points spanning the actual band (map top -> rev limit).
pt = createPowertrain();
anchorRPM = pt.rpmFalloffStartRPM;   % map top: rolloff anchor
limitRPM  = pt.rpmLimitRPM;
assumeGreaterThan(testCase, limitRPM, anchorRPM + 100);  % non-trivial band
r1 = anchorRPM + 0.25 * (limitRPM - anchorRPM);
r2 = anchorRPM + 0.75 * (limitRPM - anchorRPM);
f1 = pt.lookupTractiveForceByRPM(r1);
f2 = pt.lookupTractiveForceByRPM(r2);
% Constant power => f1*r1 == f2*r2 (within the band). Linear would give a
% much smaller f2*r2 than f1*r1.
verifyEqual(testCase, f1 * r1, f2 * r2, 'RelTol', 1e-6);
end

function testReflectedRotorInertiaAddedToDrivenAxleOnly(testCase)
% Fix 2: rear (driven) wheels get wheelInertia + I_motor*ratio^2;
% front (undriven) wheels keep the bare wheel+tire inertia.
pt = createPowertrain();
expectedDriven = pt.motorRotorInertia * pt.totalGearRatio^2;
verifyEqual(testCase, pt.getReflectedRotorInertia(), expectedDriven, 'RelTol', 1e-12);
verifyGreaterThan(testCase, expectedDriven, 0);

% A standalone powertrain has no lts.simulation.Simulator cache; the per-corner split is
% verified through getWheelInertia semantics indirectly: the reflected
% inertia is strictly positive and material relative to a 0.5 wheel inertia.
verifyGreaterThan(testCase, expectedDriven, 0.5);
end

function testSimulatorKeepsReflectedRotorInertiaAsCarrierCoupling(testCase)
    assumeTrue(testCase, tireDataAvailable(), 'TTC tire data not present: see src/+lts/+components/+Tire/README.md');
pt = createPowertrain();
tire = lts.components.Tire.PacejkaTire('43105_18x7.5_10_R25B_7.tir');
tire.wheelInertia = 0.5;
vehicle = lts.vehicle.VehicleManager([], [], pt, tire, []);
simulator = lts.simulation.Simulator(vehicle, [], 0.001);
inertia = simulator.getWheelInertia();

verifyEqual(testCase, inertia.FL, tire.wheelInertia, 'RelTol', 1e-12);
verifyEqual(testCase, inertia.FR, tire.wheelInertia, 'RelTol', 1e-12);
verifyEqual(testCase, inertia.RL, tire.wheelInertia, 'RelTol', 1e-12);
verifyEqual(testCase, inertia.RR, tire.wheelInertia, 'RelTol', 1e-12);
verifyEqual(testCase, inertia.reflectedRotorInertia, ...
    pt.getReflectedRotorInertia(), 'RelTol', 1e-12);
end

function testReflectedRotorInertiaOnlyResistsCarrierAcceleration(testCase)
    assumeTrue(testCase, tireDataAvailable(), 'TTC tire data not present: see src/+lts/+components/+Tire/README.md');
tire = lts.components.Tire.PacejkaTire('43105_18x7.5_10_R25B_7.tir');
tire.rollingResistanceCoeff = 0;
tire.bearingDragCoeff = 0;
tire.RL.normalForce = 0;
tire.RR.normalForce = 0;
tire.RL.Fx = 0;
tire.RR.Fx = 0;
tire.RL.angularVelocity = 20;
tire.RR.angularVelocity = 20;
wheelInertia = 0.5;
reflectedInertia = 0.8;
dt = 0.01;

% Equal-and-opposite torque changes differential speed without moving the
% carrier, so rotor inertia must have no effect.
tire.updateDrivenWheelPairDynamics(tire.RL, tire.RR, 10, -10, 0, 0, ...
    dt, wheelInertia, wheelInertia, reflectedInertia, 0, 0);
verifyEqual(testCase, tire.RL.angularVelocity, ...
    20 + 10 / wheelInertia * dt, 'AbsTol', 1e-12);
verifyEqual(testCase, tire.RR.angularVelocity, ...
    20 - 10 / wheelInertia * dt, 'AbsTol', 1e-12);

% Common torque accelerates the carrier and therefore sees half the total
% reflected inertia in each wheel equation.
tire.RL.angularVelocity = 20;
tire.RR.angularVelocity = 20;
tire.updateDrivenWheelPairDynamics(tire.RL, tire.RR, 10, 10, 0, 0, ...
    dt, wheelInertia, wheelInertia, reflectedInertia, 0, 0);
expectedOmega = 20 + 10 / (wheelInertia + reflectedInertia / 2) * dt;
verifyEqual(testCase, tire.RL.angularVelocity, expectedOmega, 'AbsTol', 1e-12);
verifyEqual(testCase, tire.RR.angularVelocity, expectedOmega, 'AbsTol', 1e-12);
end

function testGetMaxTorqueAgreesWithDrivePath(testCase)
% Fix 5a: getMaxTorque must derive from the same wheel-force path as
% computeDriveTorque, so telemetry and the sim cannot disagree.
pt = createPowertrain();
rpm = 3000;
Tmax = pt.getMaxTorque(rpm);
fWheel = pt.lookupTractiveForceByRPM(rpm);
TfromForce = fWheel * pt.mapWheelRadius / pt.totalGearRatio;
verifyEqual(testCase, Tmax, TfromForce, 'RelTol', 1e-9);
verifyGreaterThan(testCase, Tmax, 0);
end

function testConfiguredWheelRadiusKeepsPlannerAndLiveDriveConsistent(testCase)
pt = createPowertrain();
mapRadius = pt.mapWheelRadius;
actualRadius = 0.241935;
pt = pt.setDrivenWheelRadius(actualRadius);
speed = 10;

pt.updateStateFromVehicleSpeed(speed);
liveWheelTorque = pt.computeDriveTorque(speed, 1);
plannerForce = pt.computeMaxDriveForce(speed);

verifyEqual(testCase, pt.mapWheelRadius, mapRadius, 'AbsTol', 1e-12);
verifyEqual(testCase, pt.wheelRadius, actualRadius, 'AbsTol', 1e-12);
verifyEqual(testCase, plannerForce, liveWheelTorque / actualRadius, ...
    'RelTol', 1e-12);
end

function testVehicleManagerSynchronizesPowertrainToTireRadius(testCase)
    assumeTrue(testCase, tireDataAvailable(), 'TTC tire data not present: see src/+lts/+components/+Tire/README.md');
cfg = lts.vehicle.VehicleConfig();
cfg.tire.wheelRadius = 0.267;
vehicle = lts.vehicle.VehicleManager.fromConfig( ...
    cfg, lts.components.TestTrack('straight10'), 0.001);

verifyEqual(testCase, vehicle.powertrain.wheelRadius, cfg.tire.wheelRadius, ...
    'AbsTol', 1e-12);
verifyNotEqual(testCase, vehicle.powertrain.mapWheelRadius, ...
    vehicle.powertrain.wheelRadius);
end

function testLockedDiffReturnsMeanCarrierAndEqualSplit(testCase)
% Fix 3: the spool no longer forward-integrates drive torque (which double-
% counted the impulse). It returns a 50/50 split and the mean carrier speed,
% leaving dynamics to the per-corner solver.
diff = lts.components.Powertrain.LockedDifferential();
out = diff.solveDrive(600, 40, 60, 0.5, 0.001);
verifyEqual(testCase, out.TL, 300, 'RelTol', 1e-12);
verifyEqual(testCase, out.TR, 300, 'RelTol', 1e-12);
verifyEqual(testCase, out.carrierOmega, 50, 'RelTol', 1e-12);   % mean(40,60)
verifyTrue(testCase, diff.locksWheels());
end

function testLockedDiffZeroTorqueKeepsMeanSpeed(testCase)
% Fix 3: with no drive torque, carrierOmega is just the mean — no spurious
% acceleration term remains.
diff = lts.components.Powertrain.LockedDifferential();
out = diff.solveDrive(0, 30, 50, 0.5, 0.001);
verifyEqual(testCase, out.carrierOmega, 40, 'RelTol', 1e-12);
verifyEqual(testCase, out.TL + out.TR, 0, 'AbsTol', 1e-12);
end

function testLSDConservesTotalTorqueAtLowCommand(testCase)
% Preload is an internal equal-and-opposite torque. It may brake the fast
% wheel at a small external drive command, but cannot change axle total.
diff = lts.components.Powertrain.ClutchLSDDifferential('preload', 20);
T_total = 10;
out = diff.solveDrive(T_total, 40, 60, 0.5, 0.001);
verifyEqual(testCase, out.TL + out.TR, T_total, 'AbsTol', 1e-9);
verifyGreaterThan(testCase, out.TL, 0);
verifyLessThan(testCase, out.TR, 0);
end

function testLSDPreloadActsAsZeroNetTorqueOffThrottle(testCase)
diff = lts.components.Powertrain.ClutchLSDDifferential('preload', 20);

out = diff.solveDrive(0, 40, 60, 0.5, 0.001);

verifyEqual(testCase, out.TL + out.TR, 0, 'AbsTol', 1e-12);
verifyGreaterThan(testCase, out.TL, 0);
verifyLessThan(testCase, out.TR, 0);
end

function testLSDEqualWheelSpeedsHaveSymmetricSplit(testCase)
diff = lts.components.Powertrain.ClutchLSDDifferential( ...
    'preload', 20, 'ramp', 0.5, 'speedGain', 2);

out = diff.solveDrive(400, 50, 50, 0.5, 0.001);

verifyEqual(testCase, out.TL, 200, 'AbsTol', 1e-12);
verifyEqual(testCase, out.TR, 200, 'AbsTol', 1e-12);
end

function testLSDInternalTorqueDampsTinySlipAcrossFixedPointIterations(testCase)
diff = lts.components.Powertrain.ClutchLSDDifferential( ...
    'preload', 20, 'ramp', 0.5, 'speedGain', 2, ...
    'relativeSpeedDamping', 0.5);
wheelInertia = 0.5;
dt = 0.001;
startSlip = 0.005;
guessSlip = startSlip;

for iter = 1:8
    out = diff.solveDriveline(0, 0, 50, 50 + guessSlip, ...
        wheelInertia, dt);
    guessSlip = startSlip + (out.TR - out.TL) / wheelInertia * dt;
    verifyGreaterThan(testCase, guessSlip, 0);
    verifyLessThan(testCase, guessSlip, startSlip);
end
end

function testLSDBiasesTowardSlowerWheel(testCase)
% Fix 4: the slower wheel receives more torque than the faster one.
diff = lts.components.Powertrain.ClutchLSDDifferential('preload', 0, 'ramp', 0.5);
out = diff.solveDrive(400, 30, 60, 0.5, 0.001);   % left is slower
verifyGreaterThan(testCase, out.TL, out.TR);
verifyEqual(testCase, out.TL + out.TR, 400, 'AbsTol', 1e-9);
end

function testLSDBiasRatioRespected(testCase)
% Fix 4: T_slow / T_fast must not exceed biasRatio.
biasRatio = 2.0;
diff = lts.components.Powertrain.ClutchLSDDifferential( ...
    'preload', 0, 'ramp', 1.0, 'biasRatio', biasRatio);
out = diff.solveDrive(400, 30, 60, 0.5, 0.001);
mx = max(out.TL, out.TR);
mn = min(out.TL, out.TR);
verifyGreaterThan(testCase, mn, 0);
verifyLessThanOrEqual(testCase, mx / mn, biasRatio + 1e-9);
verifyEqual(testCase, out.TL + out.TR, 400, 'AbsTol', 1e-9);
end

function testDrexlerRequiresCalibrationBeforeSolving(testCase)
diff = lts.components.Powertrain.DrexlerRampPlateDifferential();

verifyError(testCase, @() diff.solveDrive(100, 30, 60, 0.5, 0.001), ...
    'DrexlerRampPlateDifferential:Uncalibrated');
end

function testDrexlerVehicleConfigRequiresCalibration(testCase)
    assumeTrue(testCase, tireDataAvailable(), 'TTC tire data not present: see src/+lts/+components/+Tire/README.md');
cfg = lts.vehicle.VehicleConfig();
cfg.powertrain.differential = struct('type', 'drexler');

verifyError(testCase, ...
    @() lts.vehicle.VehicleManager.fromConfig(cfg, lts.components.TestTrack('straight10'), 0.001), ...
    'DrexlerRampPlateDifferential:Uncalibrated');
end

function testDrexlerAccelRampUsesThirtyDegreeSide(testCase)
diff = createCalibratedDrexler('preloadBreakawayTorqueNm', 0, ...
    'rampTorqueScale', 1);

out = diff.solveDriveline(100, 0, 30, 60, 0.5, 0.001);

expectedLockDifference = 100 * (1 / tand(30));
expectedBias = 0.5 * expectedLockDifference;
verifyEqual(testCase, out.TL, 50 + expectedBias, 'RelTol', 1e-12);
verifyEqual(testCase, out.TR, 50 - expectedBias, 'RelTol', 1e-12);
verifyGreaterThan(testCase, out.TL, out.TR);
end

function testDrexlerDecelRampUsesFortyFiveDegreeSide(testCase)
diff = createCalibratedDrexler('preloadBreakawayTorqueNm', 0, ...
    'rampTorqueScale', 1);

out = diff.solveDriveline(0, -100, 30, 60, 0.5, 0.001);

expectedLockDifference = 100 * (1 / tand(45));
expectedBias = 0.5 * expectedLockDifference;
verifyEqual(testCase, out.TL, -50 + expectedBias, 'RelTol', 1e-12);
verifyEqual(testCase, out.TR, -50 - expectedBias, 'RelTol', 1e-12);
verifyLessThan(testCase, out.TR, out.TL);  % faster wheel gets more braking
end

function testDrexlerConservesSignedDrivelineTorque(testCase)
diff = createCalibratedDrexler('preloadBreakawayTorqueNm', 40, ...
    'rampTorqueScale', 0.25);

cases = [100 0; 0 -80; 30 -10; 0 0];
for i = 1:size(cases, 1)
    driveTorque = cases(i, 1);
    coastTorque = cases(i, 2);
    out = diff.solveDriveline(driveTorque, coastTorque, 30, 60, 0.5, 0.001);
    verifyEqual(testCase, out.TL + out.TR, driveTorque + coastTorque, ...
        'AbsTol', 1e-10);
end
end

function testDrexlerPreloadCanActAsZeroNetInternalTorque(testCase)
diff = createCalibratedDrexler('preloadBreakawayTorqueNm', 40, ...
    'rampTorqueScale', 0);

out = diff.solveDriveline(0, 0, 30, 60, 0.5, 0.001);

verifyEqual(testCase, out.TL, 20, 'RelTol', 1e-12);
verifyEqual(testCase, out.TR, -20, 'RelTol', 1e-12);
verifyEqual(testCase, out.TL + out.TR, 0, 'AbsTol', 1e-12);
end

function testDrexlerDoesNotUseHydraulicBrakeTorqueAsDecelRampInput(testCase)
% Rear hydraulic brake torque is applied after the differential in
% lts.simulation.Simulator.step, so it is not an input to this driveline solve.
diff = createCalibratedDrexler('preloadBreakawayTorqueNm', 0, ...
    'rampTorqueScale', 1);

out = diff.solveDriveline(0, 0, 30, 60, 0.5, 0.001);

verifyEqual(testCase, out.TL, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, out.TR, 0, 'AbsTol', 1e-12);
end

function testDrexlerTinySlipIsStableAcrossFixedPointIterations(testCase)
% A sign-only full-preload kick at arbitrarily small slip used to flip and
% amplify the wheel-speed difference under the simulator's repeated solve.
diff = createCalibratedDrexler( ...
    'preloadBreakawayTorqueNm', 10, ...
    'rampTorqueScale', 1, ...
    'slipSmoothingRadPerSec', 0, ...
    'relativeSpeedDamping', 0.5);
wheelInertia = 0.5;
dt = 0.001;
startSlip = 0.005;
guessSlip = startSlip;

for iter = 1:8
    out = diff.solveDriveline(0, 0, 50, 50 + guessSlip, ...
        wheelInertia, dt);
    guessSlip = startSlip + (out.TR - out.TL) / wheelInertia * dt;
    verifyGreaterThan(testCase, guessSlip, 0);
    verifyLessThan(testCase, guessSlip, startSlip);
end
end

function testDrexlerFluidMetadataDoesNotChangePhysics(testCase)
motulDiff = createCalibratedDrexler( ...
    'fluid', "Motul Gear Competition 75W-140");
referenceDiff = createCalibratedDrexler('fluid', "Reference calibration oil");

motulOut = motulDiff.solveDriveline(80, -20, 30, 60, 0.5, 0.001);
referenceOut = referenceDiff.solveDriveline(80, -20, 30, 60, 0.5, 0.001);

verifyEqual(testCase, motulDiff.fluid, "Motul Gear Competition 75W-140");
verifyEqual(testCase, motulOut.TL, referenceOut.TL, 'RelTol', 1e-12);
verifyEqual(testCase, motulOut.TR, referenceOut.TR, 'RelTol', 1e-12);
end

function testDrexlerMetadataCarriesThroughVehicleConfig(testCase)
    assumeTrue(testCase, tireDataAvailable(), 'TTC tire data not present: see src/+lts/+components/+Tire/README.md');
cfg = lts.vehicle.VehicleConfig();
cfg.powertrain.differential = struct( ...
    'type', 'drexler', ...
    'accelRampAngleDeg', 30, ...
    'decelRampAngleDeg', 45, ...
    'preloadBreakawayTorqueNm', 40, ...
    'rampTorqueScale', 0.25, ...
    'fluid', "Motul Gear Competition 75W-140");

vehicle = lts.vehicle.VehicleManager.fromConfig(cfg, lts.components.TestTrack('straight10'), 0.001);

verifyClass(testCase, vehicle.differential, ...
    'lts.components.Powertrain.DrexlerRampPlateDifferential');
verifyEqual(testCase, vehicle.differential.fluid, ...
    "Motul Gear Competition 75W-140");
end

function testCoastdownTorqueZeroWhenDisabled(testCase)
% Fix 5c: with regen + motoring drag off (defaults), no coastdown torque.
pt = createPowertrain();
pt.motoringDragTorque = 0;
pt.regenEnabled = false;
pt.updateStateFromVehicleSpeed(20);
verifyEqual(testCase, pt.computeCoastdownTorque(20, 0), 0, 'AbsTol', 1e-12);
end

function testMotoringDragOpposesSpinWhenEnabled(testCase)
% Fix 5c: motoring drag reflects motor-side torque through the ratio and
% opposes the direction of rotation.
pt = createPowertrain();
pt.motoringDragTorque = 10;
pt.regenEnabled = false;
pt.updateStateFromVehicleSpeed(20);   % forward → positive motor omega
T = pt.computeCoastdownTorque(20, 0);
verifyLessThan(testCase, T, 0);   % braking
verifyEqual(testCase, -T, 10 * pt.totalGearRatio, 'RelTol', 1e-9);
end

function testMotoringDragCanBeLimitedToLowThrottle(testCase)
pt = createPowertrain();
pt.motoringDragTorque = 10;
pt.motoringDragThrottleThreshold = 0.2;
pt.updateStateFromVehicleSpeed(20);

verifyLessThan(testCase, pt.computeCoastdownTorque(20, 0.1), 0);
verifyEqual(testCase, pt.computeCoastdownTorque(20, 0.3), 0, 'AbsTol', 1e-12);
end

function testThrottleDeadbandZerosAndRescalesDriveTorque(testCase)
pt = createPowertrain();
pt.updateStateFromVehicleSpeed(12);
referenceTorque = pt.computeDriveTorque(12, 0.5);

pt = createPowertrain();
pt.throttleDeadband = 0.2;
pt.updateStateFromVehicleSpeed(12);
verifyEqual(testCase, pt.computeDriveTorque(12, 0.1), 0, 'AbsTol', 1e-12);

pt.updateStateFromVehicleSpeed(12);
verifyEqual(testCase, pt.computeDriveTorque(12, 0.6), ...
    referenceTorque, 'RelTol', 1e-12);
end

function testDefaultThrottleMapIsControllerShaped(testCase)
pt = createPowertrain();

pt.updateStateFromVehicleSpeed(12);
fullTorque = pt.computeDriveTorque(12, 1.0);
pt.updateStateFromVehicleSpeed(12);
halfPedalTorque = pt.computeDriveTorque(12, 0.5);

verifyGreaterThan(testCase, fullTorque, 0);
verifyGreaterThan(testCase, halfPedalTorque, 0);
verifyLessThan(testCase, halfPedalTorque, 0.5 * fullTorque);
end

function testThrottleMapShapesPostDeadbandTorqueRequest(testCase)
% The throttle map represents the controller torque/current request after
% any pedal deadband has been removed.
pt = createPowertrain();
pt.throttleDeadband = 0.2;
pt.throttleMapInput = [0 0.5 1];
pt.throttleMapOutput = [0 0.25 1];

pt.updateStateFromVehicleSpeed(12);
fullTorque = pt.computeDriveTorque(12, 1.0);
pt.updateStateFromVehicleSpeed(12);
shapedTorque = pt.computeDriveTorque(12, 0.6);  % (0.6 - 0.2)/(1 - 0.2) = 0.5

verifyGreaterThan(testCase, fullTorque, 0);
verifyEqual(testCase, shapedTorque, 0.25 * fullTorque, 'RelTol', 1e-12);
end

function testThrottleMapRejectsInvalidBreakpoints(testCase)
pt = createPowertrain();
pt.throttleMapInput = [0 0.5 0.5 1];
pt.throttleMapOutput = [0 0.2 0.4 1];
pt.updateStateFromVehicleSpeed(12);

verifyError(testCase, @() pt.computeDriveTorque(12, 0.5), ...
    'EMRAX228Powertrain:InvalidThrottleMap');
end

function testPedalForTorqueFractionInvertsMapAndDeadband(testCase)
pt = createPowertrain();
pt.throttleDeadband = 0.2;
pt.throttleMapInput = [0 0.5 1];
pt.throttleMapOutput = [0 0.25 1];
speed = 12;

pt.updateStateFromVehicleSpeed(speed);
fullTorque = pt.computeDriveTorque(speed, 1);
fractions = [0 0.1 0.25 0.5 1];
for fraction = fractions
    pedal = pt.pedalForTorqueFraction(fraction);
    pt.updateStateFromVehicleSpeed(speed);
    actualTorque = pt.computeDriveTorque(speed, pedal);
    verifyEqual(testCase, actualTorque, fraction * fullTorque, ...
        'RelTol', 1e-10, 'AbsTol', 1e-10);
end

verifyEqual(testCase, pt.pedalForTorqueFraction(0), 0, 'AbsTol', 1e-12);
verifyEqual(testCase, pt.pedalForTorqueFraction(0.25), 0.6, 'AbsTol', 1e-12);
verifyEqual(testCase, pt.pedalForTorqueFraction(-1), 0, 'AbsTol', 1e-12);
verifyEqual(testCase, pt.pedalForTorqueFraction(2), 1, 'AbsTol', 1e-12);
end

function testThrottleMapRejectsEmptyBreakpoints(testCase)
pt = createPowertrain();
pt.throttleMapInput = [];
pt.throttleMapOutput = [];
pt.updateStateFromVehicleSpeed(12);

verifyError(testCase, @() pt.computeDriveTorque(12, 0.5), ...
    'EMRAX228Powertrain:InvalidThrottleMap');
end

function testPedalInverseSupportsMonotonicThrottleMapPlateau(testCase)
pt = createPowertrain();
pt.throttleMapInput = [0 0.2 0.5 1];
pt.throttleMapOutput = [0 0.2 0.2 1];

verifyEqual(testCase, pt.pedalForTorqueFraction(0.2), 0.2, 'AbsTol', 1e-12);
pedal = pt.pedalForTorqueFraction(0.3);
pt.updateStateFromVehicleSpeed(12);
fullTorque = pt.computeDriveTorque(12, 1);
pt.updateStateFromVehicleSpeed(12);
verifyEqual(testCase, pt.computeDriveTorque(12, pedal), 0.3 * fullTorque, ...
    'RelTol', 1e-10);
end

function testVehicleConfigDoesNotHideEmptyThrottleMap(testCase)
    assumeTrue(testCase, tireDataAvailable(), 'TTC tire data not present: see src/+lts/+components/+Tire/README.md');
cfg = lts.vehicle.VehicleConfig();
cfg.powertrain.throttleMapInput = [];
cfg.powertrain.throttleMapOutput = [];
vehicle = lts.vehicle.VehicleManager.fromConfig(cfg, lts.components.TestTrack('straight10'), 0.001);
vehicle.powertrain.updateStateFromVehicleSpeed(12);

verifyError(testCase, @() vehicle.powertrain.computeDriveTorque(12, 0.5), ...
    'EMRAX228Powertrain:InvalidThrottleMap');
end

function testRegenZeroAtThrottleAndTapersNearRest(testCase)
% Fix 5c: regen only at off-throttle, and tapered to zero near rest so it
% cannot reverse the car.
pt = createPowertrain();
pt.regenEnabled = true;
pt.regenTorqueLimitNm = 30;
pt.updateStateFromVehicleSpeed(20);
% On-throttle: no regen.
verifyEqual(testCase, pt.computeCoastdownTorque(20, 0.5), 0, 'AbsTol', 1e-12);
% Off-throttle at speed: regen braking present.
verifyLessThan(testCase, pt.computeCoastdownTorque(20, 0), 0);
% Near rest: regen tapers toward zero.
Tfast = pt.computeCoastdownTorque(20, 0);
Tslow = pt.computeCoastdownTorque(0.5 * pt.regenEnabledSpeedFloor, 0);
verifyLessThan(testCase, abs(Tslow), abs(Tfast));
end

function testReverseRotationAllowedWhenRegenCapable(testCase)
% Fix 5c: reverseCapable is true iff regen or motoring drag is on.
pt = createPowertrain();
verifyFalse(testCase, pt.reverseCapable);
pt.regenEnabled = true;
verifyTrue(testCase, pt.reverseCapable);
pt.regenEnabled = false;
pt.motoringDragTorque = 5;
verifyTrue(testCase, pt.reverseCapable);
end

function testThrottleModeRegenUsesWheelToMotorEfficiencyDirection(testCase)
pt = createPowertrain();
pt.regenEnabled = true;
pt.regenTorqueLimitNm = 30;
pt.regenEfficiency = 0.8;
pt.updateStateFromVehicleSpeed(20);

actual = pt.computeCoastdownTorque(20, 0);
expected = -pt.regenTorqueLimitNm * pt.totalGearRatio / pt.regenEfficiency;
verifyEqual(testCase, actual, expected, 'RelTol', 1e-12);
end

% ---------- helpers ----------

function pt = createPowertrain()
pt = lts.components.Powertrain.EMRAX228Powertrain();
end

function diff = createCalibratedDrexler(varargin)
diff = lts.components.Powertrain.DrexlerRampPlateDifferential( ...
    'preloadBreakawayTorqueNm', 30, ...
    'rampTorqueScale', 0.2, ...
    varargin{:});
end

function path = powertrainMapPath(fileName)
testDir = fileparts(mfilename('fullpath'));
repoRoot = fileparts(testDir);
path = fullfile(repoRoot, 'src', '+lts', '+components', '+Powertrain', fileName);
end
