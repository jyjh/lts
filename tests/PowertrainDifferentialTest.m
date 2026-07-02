function tests = PowertrainDifferentialTest
% POWERTRAINDIFFERENTIALTEST Coverage for the EMRAX field-weakening fix,
%   reflected motor inertia, locked/LSD differential reworks, and the
%   opt-in coastdown/regen path. Uses functiontests + verifyXxx.
tests = functiontests(localfunctions);
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

% A standalone powertrain has no Simulator cache; the per-corner split is
% verified through getWheelInertia semantics indirectly: the reflected
% inertia is strictly positive and material relative to a 0.5 wheel inertia.
verifyGreaterThan(testCase, expectedDriven, 0.5);
end

function testSimulatorSplitsReflectedRotorInertiaAcrossRearWheels(testCase)
pt = createPowertrain();
tire = components.Tire.PacejkaTire('43105_18x7.5_10_R25B_7.tir');
tire.wheelInertia = 0.5;
vehicle = VehicleManager([], [], pt, tire, []);
simulator = Simulator(vehicle, [], 0.001);
inertia = simulator.getWheelInertia();

expectedRear = tire.wheelInertia + 0.5 * pt.getReflectedRotorInertia();
verifyEqual(testCase, inertia.FL, tire.wheelInertia, 'RelTol', 1e-12);
verifyEqual(testCase, inertia.FR, tire.wheelInertia, 'RelTol', 1e-12);
verifyEqual(testCase, inertia.RL, expectedRear, 'RelTol', 1e-12);
verifyEqual(testCase, inertia.RR, expectedRear, 'RelTol', 1e-12);
end

function testGetMaxTorqueAgreesWithDrivePath(testCase)
% Fix 5a: getMaxTorque must derive from the same wheel-force path as
% computeDriveTorque, so telemetry and the sim cannot disagree.
pt = createPowertrain();
rpm = 3000;
Tmax = pt.getMaxTorque(rpm);
fWheel = pt.lookupTractiveForceByRPM(rpm);
TfromForce = fWheel * pt.wheelRadius / (pt.totalGearRatio * pt.drivetrainEfficiency);
verifyEqual(testCase, Tmax, TfromForce, 'RelTol', 1e-9);
verifyGreaterThan(testCase, Tmax, 0);
end

function testLockedDiffReturnsMeanCarrierAndEqualSplit(testCase)
% Fix 3: the spool no longer forward-integrates drive torque (which double-
% counted the impulse). It returns a 50/50 split and the mean carrier speed,
% leaving dynamics to the per-corner solver.
diff = components.Powertrain.LockedDifferential();
out = diff.solveDrive(600, 40, 60, 0.5, 0.001);
verifyEqual(testCase, out.TL, 300, 'RelTol', 1e-12);
verifyEqual(testCase, out.TR, 300, 'RelTol', 1e-12);
verifyEqual(testCase, out.carrierOmega, 50, 'RelTol', 1e-12);   % mean(40,60)
verifyTrue(testCase, diff.locksWheels());
end

function testLockedDiffZeroTorqueKeepsMeanSpeed(testCase)
% Fix 3: with no drive torque, carrierOmega is just the mean — no spurious
% acceleration term remains.
diff = components.Powertrain.LockedDifferential();
out = diff.solveDrive(0, 30, 50, 0.5, 0.001);
verifyEqual(testCase, out.carrierOmega, 40, 'RelTol', 1e-12);
verifyEqual(testCase, out.TL + out.TR, 0, 'AbsTol', 1e-12);
end

function testLSDConservesTotalTorqueAtLowCommand(testCase)
% Fix 4: with a high preload relative to a small commanded torque, the old
% code drove the fast side negative and broke TL+TR == T_total after the
% non-negative clamp. The Tlock cap now prevents this.
diff = components.Powertrain.ClutchLSDDifferential('preload', 20);
T_total = 10;   % preload (20) >> base (5) → would invert the fast side pre-fix
out = diff.solveDrive(T_total, 40, 60, 0.5, 0.001);
verifyEqual(testCase, out.TL + out.TR, T_total, 'AbsTol', 1e-9);
verifyGreaterThanOrEqual(testCase, out.TL, 0);
verifyGreaterThanOrEqual(testCase, out.TR, 0);
end

function testLSDBiasesTowardSlowerWheel(testCase)
% Fix 4: the slower wheel receives more torque than the faster one.
diff = components.Powertrain.ClutchLSDDifferential('preload', 0, 'ramp', 0.5);
out = diff.solveDrive(400, 30, 60, 0.5, 0.001);   % left is slower
verifyGreaterThan(testCase, out.TL, out.TR);
verifyEqual(testCase, out.TL + out.TR, 400, 'AbsTol', 1e-9);
end

function testLSDBiasRatioRespected(testCase)
% Fix 4: T_slow / T_fast must not exceed biasRatio.
biasRatio = 2.0;
diff = components.Powertrain.ClutchLSDDifferential( ...
    'preload', 0, 'ramp', 1.0, 'biasRatio', biasRatio);
out = diff.solveDrive(400, 30, 60, 0.5, 0.001);
mx = max(out.TL, out.TR);
mn = min(out.TL, out.TR);
if mn > 0
    verifyLessThanOrEqual(testCase, mx / mn, biasRatio + 1e-9);
end
verifyEqual(testCase, out.TL + out.TR, 400, 'AbsTol', 1e-9);
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

% ---------- helpers ----------

function pt = createPowertrain()
pt = components.Powertrain.EMRAX228Powertrain();
end
