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
    wheelSpeed = corner.angularVelocity * corner.wheelRadius;
    denom = max(abs(wheelSpeed), abs(longSpeed));
    kappa = (wheelSpeed - longSpeed) / max(denom, 1.0);
    kappa = max(-1, min(1, kappa));
    tire.updateCorner(corner, Fz, 0, kappa, 0, 1.2, 0, longSpeed, true, 'steady');
    tire.updateWheelDynamics(corner, 0, 0, dt, 0.5, longSpeed);
end

finalWheelSpeed = corner.angularVelocity * corner.wheelRadius;
finalKappa = (finalWheelSpeed - longSpeed) / max(abs(finalWheelSpeed), abs(longSpeed), 1.0);
verifyLessThan(testCase, abs(finalKappa), 0.05);
verifyLessThan(testCase, abs(finalWheelSpeed - longSpeed), 1.0);
end

function testLargeBrakeTorqueCanIntegrateWheelSpeedThroughZero(testCase)
tire = createPacejkaTire();
corner = tire.FL;
Fz = 1000;
longSpeed = 20;
largeBrakeTorque = 2000;

corner.angularVelocity = 0;
% Prime tire force at full brake slip (wheel at standstill, road moving).
kappa = -1;
tire.updateCorner(corner, Fz, 0, kappa, 0, 1.2, 0, longSpeed, true, 'steady');
% One brake step from standstill — the combined brake + slip force
% drives omega negative.
tire.updateWheelDynamics(corner, 0, largeBrakeTorque, 0.001, 0.5, longSpeed);

verifyLessThan(testCase, corner.angularVelocity, 0);
end

function testDrivenWheelProducesPositiveSlipAndForce(testCase)
tire = createPacejkaTire();
corner = tire.RL;
Fz = 1000;
longSpeed = 12;

corner.angularVelocity = longSpeed / corner.wheelRadius;
% Prime tire force at zero slip (free-rolling).
tire.updateCorner(corner, Fz, 0, 0, 0, 1.2, 0, longSpeed, true, 'steady');
% Apply drive torque — wheel speeds up, producing positive slip.
tire.updateWheelDynamics(corner, 250, 0, 0.001, 0.5, longSpeed);

wheelSpeed = corner.angularVelocity * corner.wheelRadius;
kappa = (wheelSpeed - longSpeed) / max(abs(wheelSpeed), abs(longSpeed), 1.0);
verifyGreaterThan(testCase, kappa, 0);

% Re-evaluate force at the new slip.
tire.updateCorner(corner, Fz, 0, kappa, 0, 1.2, 0, longSpeed, true, 'steady');
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

function testPacejkaSurfaceMuScalesForcesFromDryReference(testCase)
tire = createPacejkaTire();
tire.relaxationLength = 0;
corner = tire.FL;

tire.updateCorner(corner, 1000, 0.04, 0.06, 0, 0, 20);
baselineFx = corner.Fx;
baselineFy = corner.Fy;
baselineMz = corner.Mz;
baselinePeakMu = corner.peakMu;

tire.updateCorner(corner, 1000, 0.04, 0.06, 0, 1.2, 0, 20);
dryFx = corner.Fx;
dryFy = corner.Fy;
dryMz = corner.Mz;
dryPeakMu = corner.peakMu;

verifyEqual(testCase, dryFx, baselineFx, 'RelTol', 1e-12);
verifyEqual(testCase, dryFy, baselineFy, 'RelTol', 1e-12);
verifyEqual(testCase, dryMz, baselineMz, 'RelTol', 1e-12);
verifyEqual(testCase, dryPeakMu, baselinePeakMu, 'RelTol', 1e-12);

tire.updateCorner(corner, 1000, 0.04, 0.06, 0, 0.6, 0, 20);
verifyEqual(testCase, corner.Fx, 0.5 * dryFx, 'RelTol', 1e-12);
verifyEqual(testCase, corner.Fy, 0.5 * dryFy, 'RelTol', 1e-12);
verifyEqual(testCase, corner.Mz, 0.5 * dryMz, 'RelTol', 1e-12);
verifyEqual(testCase, corner.peakMu, 0.5 * dryPeakMu, 'RelTol', 1e-12);
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

function testRelaxationCommitNotScaledByLateralStiffnessScale(testCase)
% The committed lagged slip state must stay in physical radians.
% lateralStiffnessScale only affects the MFeval evaluation slip, not the
% stored relaxation state. This guards the invariant documented in the
% PacejkaTire property comments.
tire = createPacejkaTire();
tire.relaxationLength = 0.30;
tire.lateralStiffnessScale = 1.5;
corner = tire.FL;
corner.slipAngle = 0;
corner.slipRatio = 0;

ssAlpha = 0.10;
dt = 0.001;
longSpeed = 20;
decay = exp(-longSpeed * dt / tire.relaxationLength);
expectedSlipAngle = ssAlpha * (1 - decay);

tire.updateCorner(corner, 1000, ssAlpha, 0, 0, 1.2, dt, longSpeed, true, 'advance');

% Committed state must be the unscaled relaxed value.
verifyEqual(testCase, corner.slipAngle, expectedSlipAngle, 'AbsTol', 1e-12);
verifyGreaterThan(testCase, expectedSlipAngle * tire.lateralStiffnessScale, ...
    expectedSlipAngle);
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

function tire = createPacejkaTire()
tire = lts.components.Tire.PacejkaTire('43105_18x7.5_10_R25B_7.tir');
end
