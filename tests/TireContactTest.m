function tests = TireContactTest
tests = functiontests(localfunctions);
end

function testFreeRollingWheelConvergesToRoadSpeed(testCase)
tire = createPacejkaTire();
corner = tire.FL;
Fz = 1000;
longSpeed = 18;
dt = 0.001;

corner.angularVelocity = 0;
for idx = 1:1000
    tire.solveWheelContact(corner, Fz, 0, 0, 1.2, longSpeed, 0, 0, dt);
end

verifyLessThan(testCase, abs(corner.slipRatio), 0.05);
verifyLessThan(testCase, abs(corner.angularVelocity * corner.wheelRadius - longSpeed), 1.0);
end

function testLargeBrakeTorqueCanIntegrateWheelSpeedThroughZero(testCase)
tire = createPacejkaTire();
corner = tire.FL;
longSpeed = 20;
dt = 0.001;
inertia = 0.5;
tire.rollingResistanceCoeff = 0;
tire.bearingDragCoeff = 0;
corner.normalForce = 0;
corner.Fx = 0;
corner.angularVelocity = longSpeed / corner.wheelRadius;
omegaBefore = corner.angularVelocity;
largeBrakeTorque = 2 * inertia * omegaBefore / dt;
expectedOmega = omegaBefore - largeBrakeTorque / inertia * dt;

tire.updateWheelDynamics( ...
    corner, 0, largeBrakeTorque, dt, inertia, longSpeed);

verifyGreaterThan(testCase, omegaBefore, 0);
verifyLessThan(testCase, corner.angularVelocity, 0);
verifyEqual(testCase, corner.angularVelocity, expectedOmega, 'AbsTol', 1e-12);
end

function testDrivenWheelProducesPositiveSlipAndForce(testCase)
tire = createPacejkaTire();
corner = tire.RL;
Fz = 1000;
longSpeed = 12;

corner.angularVelocity = longSpeed / corner.wheelRadius;
tire.solveWheelContact(corner, Fz, 0, 0, 1.2, longSpeed, 250, 0, 0.001);

verifyGreaterThan(testCase, corner.slipRatio, 0);
verifyGreaterThan(testCase, corner.Fx, 0);
end

function testPacejkaPeakMuCacheIncludesEvaluationSpeed(testCase)
tire = createPacejkaTire();
corner = tire.FL;
warningState = warning('query', 'Solver:Limits:Exceeded');
cleanup = onCleanup(@() warning(warningState.state, 'Solver:Limits:Exceeded'));
warning('error', 'Solver:Limits:Exceeded');

% updateCorner signature: (cornerState, Fz, alpha, kappa, gamma, dt, longSpeed).
% The trailing numeric is the contact-patch longitudinal speed [m/s], which
% the speed-keyed peak-Mu cache must distinguish.
tire.updateCorner(corner, 1000, 0.05, 0, 0, 0, 0.2);
keysAfterLowSpeed = tire.peakMuCache.keys;
tire.updateCorner(corner, 1000, 0.05, 0, 0, 0, 20);
keysAfterHighSpeed = tire.peakMuCache.keys;

verifyTrue(testCase, any(contains(keysAfterLowSpeed, '_1.0')));
verifyGreaterThan(testCase, numel(keysAfterHighSpeed), numel(keysAfterLowSpeed));
end

function testPacejkaIgnoresLegacySurfaceMuArguments(testCase)
tire = createPacejkaTire();
tire.relaxationLength = 0;
corner = tire.FL;

tire.updateCorner(corner, 1000, 0.04, 0.06, 0, 0, 20);
baselineFx = corner.Fx;
baselineFy = corner.Fy;
baselineMz = corner.Mz;
baselinePeakMu = corner.peakMu;

tire.updateCorner(corner, 1000, 0.04, 0.06, 0, 0.25, 0, 20);
alternateFx = corner.Fx;
alternateFy = corner.Fy;
alternateMz = corner.Mz;
alternatePeakMu = corner.peakMu;

verifyEqual(testCase, alternateFx, baselineFx, 'RelTol', 1e-12);
verifyEqual(testCase, alternateFy, baselineFy, 'RelTol', 1e-12);
verifyEqual(testCase, alternateMz, baselineMz, 'RelTol', 1e-12);
verifyEqual(testCase, alternatePeakMu, baselinePeakMu, 'RelTol', 1e-12);

tire.updateCorner(corner, 1000, 0.04, 0.06, 0, 2.0, 0, 20);
verifyEqual(testCase, corner.Fx, baselineFx, 'RelTol', 1e-12);
verifyEqual(testCase, corner.Fy, baselineFy, 'RelTol', 1e-12);
verifyEqual(testCase, corner.Mz, baselineMz, 'RelTol', 1e-12);
verifyEqual(testCase, corner.peakMu, baselinePeakMu, 'RelTol', 1e-12);
verifyEqual(testCase, tire.computeLongitudinalForce(1000, 0.06, 0.2), ...
    tire.computeLongitudinalForce(1000, 0.06, 2.0), 'RelTol', 1e-12);
verifyEqual(testCase, tire.computeLateralForce(1000, 0.04, 0.2), ...
    tire.computeLateralForce(1000, 0.04, 2.0), 'RelTol', 1e-12);
end

function testSlipRatioMatchesMFevalDriveConvention(testCase)
tire = createPacejkaTire();
corner = tire.RL;
corner.wheelRadius = 0.2;

corner.angularVelocity = 12 / corner.wheelRadius;
verifyEqual(testCase, ...
    tire.computeSlipRatioFromKinematics(corner, 10), 0.2, 'AbsTol', 1e-12);

% Positive drive slip is not artificially capped at one.
corner.angularVelocity = 22 / corner.wheelRadius;
verifyEqual(testCase, ...
    tire.computeSlipRatioFromKinematics(corner, 10), 1.2, 'AbsTol', 1e-12);

% A locked wheel is exactly -1 for a moving forward contact patch.
corner.angularVelocity = 0;
verifyEqual(testCase, ...
    tire.computeSlipRatioFromKinematics(corner, 10), -1, 'AbsTol', 1e-12);
end

function testTrueRestClearsStaleLongitudinalSlip(testCase)
tire = createPacejkaTire();
tire.relaxationLength = 0.30;
corner = tire.FL;
corner.angularVelocity = 0;
corner.slipRatio = -1;

kappa = tire.computeSlipRatioFromKinematics(corner, 0);
verifyEqual(testCase, kappa, 0, 'AbsTol', 1e-12);
tire.updateCorner(corner, 900, 0, kappa, 0, 1, 0.01, 0, true, 'advance');

verifyEqual(testCase, corner.slipRatio, 0, 'AbsTol', 1e-12);
expectedFx = tire.computeLongitudinalForce(900, 0, 1);
verifyEqual(testCase, corner.Fx, expectedFx, 'AbsTol', 1e-9);
end

function testHoldRelaxationUsesCommittedLaggedSlip(testCase)
tire = createPacejkaTire();
tire.relaxationLength = 0.30;
corner = tire.FL;
corner.slipAngle = 0;
corner.slipRatio = 0;

tire.updateCorner(corner, 1000, 0, 0.8, 0, 1.2, 0, 20, true, 'hold');

verifyEqual(testCase, corner.slipAngle, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, corner.slipRatio, 0, 'AbsTol', 1e-12);
verifyLessThan(testCase, abs(corner.Fx), 100);
end

function testZeroSpeedWheelBrakingFollowsTorqueBalance(testCase)
tire = createPacejkaTire();
corner = tire.FL;
corner.normalForce = 1000;
corner.angularVelocity = 0;
corner.Fx = -1000;
R = corner.wheelRadius;
dt = 0.001;
I = 0.5;
brakeTorque = 500;
longSpeed = 20;
expectedOmega = (-brakeTorque - corner.Fx * R ...
    - tire.rollingResistanceCoeff * corner.normalForce * R) / I * dt;

tire.updateWheelDynamics(corner, 0, brakeTorque, dt, I, longSpeed);

verifyEqual(testCase, corner.angularVelocity, expectedOmega, 'AbsTol', 1e-12);
verifyLessThan(testCase, corner.angularVelocity, 0);
end

function testPositiveSlipProducesRestoringAligningMoment(testCase)
tire = createPacejkaTire();
tire.relaxationLength = 0;
tire.updateCorner(tire.FL, 1000, 0.05, 0, 0, ...
    tire.surfaceMuReference, 0, 10, true, 'steady');

verifyGreaterThan(testCase, tire.FL.Fy, 0);
verifyLessThan(testCase, tire.FL.Mz, 0);
end

function testMirroredRightTiresCancelSymmetricLateralForces(testCase)
tire = createPacejkaTire();
tire.relaxationLength = 0;
normalLoad = 1000;
slipAngle = 0.04;
camber = -0.01;
speed = 15;

tire.updateAllCorners(normalLoad, normalLoad, normalLoad, normalLoad, ...
    slipAngle, -slipAngle, slipAngle, -slipAngle, ...
    0, 0, 0, 0, camber, camber, camber, camber, ...
    0, repmat(speed, 4, 1), tire.surfaceMuReference, true, 'steady');

verifyEqual(testCase, tire.FL.Fx, tire.FR.Fx, 'AbsTol', 1e-10);
verifyEqual(testCase, tire.FL.Fy, -tire.FR.Fy, 'AbsTol', 1e-10);
verifyEqual(testCase, tire.FL.Mx, -tire.FR.Mx, 'AbsTol', 1e-10);
verifyEqual(testCase, tire.FL.My, tire.FR.My, 'AbsTol', 1e-10);
verifyEqual(testCase, tire.FL.Mz, -tire.FR.Mz, 'AbsTol', 1e-10);
verifyEqual(testCase, tire.FL.Fy + tire.FR.Fy + tire.RL.Fy + tire.RR.Fy, ...
    0, 'AbsTol', 1e-10);
end

function testRelaxedSlipStoresPhysicalAngleBeforeStiffnessScaling(testCase)
tire = createPacejkaTire();
tire.relaxationLength = 0.30;
tire.lateralStiffnessScale = 2;
dt = 0.01;
speed = 10;
targetAlpha = 0.05;
expectedAlpha = targetAlpha * (1 - exp(-speed * dt / tire.relaxationLength));

tire.updateCorner(tire.FL, 1000, targetAlpha, 0, 0, ...
    tire.surfaceMuReference, dt, speed, true, 'advance');
verifyEqual(testCase, tire.FL.slipAngle, expectedAlpha, 'AbsTol', 1e-12);

tire.FL.slipAngle = 0;
tire.FR.slipAngle = 0;
tire.RL.slipAngle = 0;
tire.RR.slipAngle = 0;
tire.updateAllCorners(1000, 1000, 1000, 1000, ...
    targetAlpha, targetAlpha, targetAlpha, targetAlpha, ...
    0, 0, 0, 0, 0, 0, 0, 0, dt, repmat(speed, 4, 1), ...
    tire.surfaceMuReference, true, 'advance');
verifyEqual(testCase, tire.FL.slipAngle, expectedAlpha, 'AbsTol', 1e-12);
end

function testLongitudinalRelaxationLengthIsIndependent(testCase)
tire = createPacejkaTire();
tire.relaxationLength = 0.30;
tire.longitudinalRelaxationLength = 0.05;
dt = 0.01;
speed = 10;
targetAlpha = 0.05;
targetKappa = 0.08;
expectedAlpha = targetAlpha * ...
    (1 - exp(-speed * dt / tire.relaxationLength));
expectedKappa = targetKappa * ...
    (1 - exp(-speed * dt / tire.longitudinalRelaxationLength));

tire.updateAllCorners(1000, 1000, 1000, 1000, ...
    targetAlpha, targetAlpha, targetAlpha, targetAlpha, ...
    targetKappa, targetKappa, targetKappa, targetKappa, ...
    0, 0, 0, 0, dt, repmat(speed, 4, 1), ...
    tire.surfaceMuReference, false, 'advance');

verifyEqual(testCase, tire.FL.slipAngle, expectedAlpha, 'AbsTol', 1e-12);
verifyEqual(testCase, tire.FL.slipRatio, expectedKappa, 'AbsTol', 1e-12);
verifyGreaterThan(testCase, tire.FL.slipRatio / targetKappa, ...
    tire.FL.slipAngle / targetAlpha);
end

function testNormalLoadRelaxationLagsForceEvaluationLoad(testCase)
tire = createPacejkaTire();
tire.relaxationLength = 0;
tire.normalLoadRelaxationLength = 0.25;
dt = 0.01;
speed = 10;
lowLoad = 800;
highLoad = 1600;
% First evaluation seeds the lagged load at the incoming load.
tire.updateAllCorners(lowLoad, lowLoad, lowLoad, lowLoad, ...
    0, 0, 0, 0, 0.05, 0.05, 0.05, 0.05, 0, 0, 0, 0, ...
    dt, repmat(speed, 4, 1), tire.surfaceMuReference, false, 'advance');
verifyEqual(testCase, tire.FL.relaxedNormalLoad, lowLoad, 'AbsTol', 1e-12);

% A load step is tracked with the exact exponential patch lag.
expectedLoad = highLoad - (highLoad - lowLoad) * ...
    exp(-speed * dt / tire.normalLoadRelaxationLength);
tire.updateAllCorners(highLoad, highLoad, highLoad, highLoad, ...
    0, 0, 0, 0, 0.05, 0.05, 0.05, 0.05, 0, 0, 0, 0, ...
    dt, repmat(speed, 4, 1), tire.surfaceMuReference, false, 'advance');
verifyEqual(testCase, tire.FL.relaxedNormalLoad, expectedLoad, ...
    'AbsTol', 1e-12);
verifyEqual(testCase, tire.FL.normalForce, highLoad, 'AbsTol', 1e-12);

% The force matches a steady evaluation at the lagged load, not the
% instantaneous one: a load change alone must not change force instantly.
steadyAtExpected = lts.components.Tire.PacejkaTire( ...
    tire.tireConstants.tirFilePath);
steadyAtExpected.relaxationLength = 0;
steadyAtExpected.normalLoadRelaxationLength = 0;
steadyAtExpected.updateAllCorners(expectedLoad, expectedLoad, ...
    expectedLoad, expectedLoad, 0, 0, 0, 0, ...
    0.05, 0.05, 0.05, 0.05, 0, 0, 0, 0, ...
    0, repmat(speed, 4, 1), tire.surfaceMuReference, false, 'steady');
verifyEqual(testCase, tire.FL.Fx, steadyAtExpected.FL.Fx, 'RelTol', 1e-9);

% Contact loss clears the lagged load without a decay tail.
tire.updateAllCorners(0, 0, 0, 0, ...
    0, 0, 0, 0, 0.05, 0.05, 0.05, 0.05, 0, 0, 0, 0, ...
    dt, repmat(speed, 4, 1), tire.surfaceMuReference, false, 'advance');
verifyEqual(testCase, tire.FL.relaxedNormalLoad, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, tire.FL.Fx, 0, 'AbsTol', 1e-12);
end

function testNormalLoadRelaxationPreviewDoesNotCommitLaggedLoad(testCase)
tire = createPacejkaTire();
tire.relaxationLength = 0;
tire.normalLoadRelaxationLength = 0.25;
dt = 0.01;
speed = 10;
tire.updateAllCorners(800, 800, 800, 800, ...
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, ...
    dt, repmat(speed, 4, 1), tire.surfaceMuReference, false, 'advance');

% Preview evaluates at the advanced load but leaves the committed state
% untouched so the next physics step advances the lag exactly once.
tire.updateAllCorners(1600, 1600, 1600, 1600, ...
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, ...
    dt, repmat(speed, 4, 1), tire.surfaceMuReference, false, 'preview');
verifyEqual(testCase, tire.FL.relaxedNormalLoad, 800, 'AbsTol', 1e-12);

tire.updateAllCorners(1600, 1600, 1600, 1600, ...
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, ...
    dt, repmat(speed, 4, 1), tire.surfaceMuReference, false, 'advance');
expectedLoad = 1600 - (1600 - 800) * ...
    exp(-speed * dt / tire.normalLoadRelaxationLength);
verifyEqual(testCase, tire.FL.relaxedNormalLoad, expectedLoad, ...
    'AbsTol', 1e-12);
end

function testNormalLoadRelaxationDisabledByDefaultKeepsLegacyForce(testCase)
tire = createPacejkaTire();
tire.relaxationLength = 0;
verifyEqual(testCase, tire.normalLoadRelaxationLength, 0);

steadyAtLoad = lts.components.Tire.PacejkaTire( ...
    tire.tireConstants.tirFilePath);
steadyAtLoad.relaxationLength = 0;
steadyAtLoad.normalLoadRelaxationLength = 0;
args = {1000, 1000, 1000, 1000, 0.03, 0.03, 0.03, 0.03, ...
    0.04, 0.04, 0.04, 0.04, 0, 0, 0, 0};
tire.updateAllCorners(args{:}, 0.001, repmat(15, 4, 1), ...
    tire.surfaceMuReference, false, 'advance');
steadyAtLoad.updateAllCorners(args{:}, 0.001, repmat(15, 4, 1), ...
    tire.surfaceMuReference, false, 'steady');
verifyEqual(testCase, tire.FL.Fx, steadyAtLoad.FL.Fx, 'AbsTol', 1e-12);
verifyEqual(testCase, tire.FL.Fy, steadyAtLoad.FL.Fy, 'AbsTol', 1e-12);
end

function testPerCornerLateralStiffnessScaleChangesForceNotSlipState(testCase)
tire = createPacejkaTire();
tire.relaxationLength = 0;
tire.lateralStiffnessScaleByCorner = [0.65 0.65 1 1];
normalLoad = 1000;
slipAngle = 0.04;
speed = 15;

tire.updateAllCorners(normalLoad, normalLoad, normalLoad, normalLoad, ...
    slipAngle, slipAngle, slipAngle, slipAngle, ...
    0, 0, 0, 0, 0, 0, 0, 0, ...
    0.001, repmat(speed, 4, 1), tire.surfaceMuReference, false, 'advance');

verifyEqual(testCase, ...
    [tire.FL.slipAngle, tire.FR.slipAngle, ...
     tire.RL.slipAngle, tire.RR.slipAngle], ...
    repmat(slipAngle, 1, 4), 'AbsTol', 1e-12);
verifyLessThan(testCase, abs(tire.FL.Fy), abs(tire.RL.Fy));
verifyLessThan(testCase, abs(tire.FR.Fy), abs(tire.RR.Fy));
end

function testPassiveWheelCanRollNegativeWithReverseLocalRoadSpeed(testCase)
tire = createPacejkaTire();
corner = tire.FL;
corner.normalForce = 1000;
corner.angularVelocity = 0;
longSpeed = -7;

tire.updateCorner(corner, 1000, 0, 1, 0, 1.2, 0, longSpeed, true, 'steady');
tire.updateWheelDynamics(corner, 0, 0, 0.001, 0.5, longSpeed);

verifyLessThan(testCase, corner.angularVelocity, 0);
end

function testReverseRoadSpeedBrakeTorqueFollowsTorqueBalance(testCase)
tire = createPacejkaTire();
corner = tire.FL;
corner.normalForce = 1000;
corner.angularVelocity = 0;
longSpeed = -7;
dt = 0.001;
I = 0.5;
brakeTorque = 2000;

tire.updateCorner(corner, 1000, 0, 1, 0, 1.2, 0, longSpeed, true, 'steady');
fxBefore = corner.Fx;
R = corner.wheelRadius;
expectedOmega = (brakeTorque - fxBefore * R ...
    + tire.rollingResistanceCoeff * corner.normalForce * R) / I * dt;
tire.updateWheelDynamics(corner, 0, brakeTorque, dt, I, longSpeed);

verifyEqual(testCase, corner.angularVelocity, expectedOmega, 'AbsTol', 1e-12);
verifyGreaterThan(testCase, corner.angularVelocity, 0);
end

function testR25ScaledTireRepresents43075GeometryAndPhysicsScales(testCase)
constants = lts.components.Tire.TireConstants( ...
    'Hoosier 43100 18.0x6.0-10 R20_7 - Scaled.tir');
p = constants.params;
cfg = lts.vehicles.R25();

verifyEqual(testCase, p.UNLOADED_RADIUS, 16.2 * 0.0254 / 2, 'AbsTol', 1e-8);
verifyEqual(testCase, p.WIDTH, 7.3 * 0.0254, 'AbsTol', 1e-8);
verifyEqual(testCase, p.RIM_WIDTH, 8.0 * 0.0254, 'AbsTol', 1e-8);
verifyEqual(testCase, p.MASS1, 3.49 * 8 / 9, 'AbsTol', 1e-5);
verifyEqual(testCase, p.LMUX, 1, 'AbsTol', 1e-12);
verifyEqual(testCase, p.LMUY, 1, 'AbsTol', 1e-12);
verifyEqual(testCase, p.LKX, 0.67, 'AbsTol', 1e-12);
verifyEqual(testCase, p.LKY, 1.05, 'AbsTol', 1e-12);
verifyEqual(testCase, p.LTR, 0.95, 'AbsTol', 1e-12);
assemblyMass = 13 * 0.45359237;
rimRadius = 10 * 0.0254 / 2;
expectedInertia = 0.5 * p.MASS1 * ...
    (p.UNLOADED_RADIUS^2 + rimRadius^2) + ...
    (assemblyMass - p.MASS1) * rimRadius^2;
verifyEqual(testCase, cfg.tire.wheelInertia, expectedInertia, 'AbsTol', 1e-12);
verifyEqual(testCase, cfg.tire.relaxationLength, 0.255, 'AbsTol', 1e-12);
% NaN shares the lateral length: the former 0.05 m stability de-tune is
% replaced by the contact-patch load response below.
verifyTrue(testCase, isscalar(cfg.tire.longitudinalRelaxationLength) && ...
    isnan(cfg.tire.longitudinalRelaxationLength));
verifyEqual(testCase, cfg.tire.normalLoadRelaxationLength, ...
    cfg.tire.relaxationLength, 'AbsTol', 1e-12);
end

function tire = createPacejkaTire()
tire = lts.components.Tire.PacejkaTire('43105_18x7.5_10_R25B_7.tir');
end
