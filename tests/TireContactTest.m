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

function testLockedWheelStaysLockedWhenBrakeTorqueExceedsRoadTorque(testCase)
tire = createPacejkaTire();
corner = tire.FL;
Fz = 1000;
longSpeed = 20;
largeBrakeTorque = 2000;

corner.angularVelocity = 0;
tire.solveWheelContact(corner, Fz, 0, 0, 1.2, longSpeed, 0, largeBrakeTorque, 0.001);

verifyEqual(testCase, corner.angularVelocity, 0, 'AbsTol', 1e-12);
verifyLessThanOrEqual(testCase, corner.slipRatio, -0.95);
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

function testZeroSpeedWheelBrakingUsesRoadLongitudinalSpeed(testCase)
tire = createPacejkaTire();
corner = tire.FL;
corner.normalForce = 1000;
corner.angularVelocity = 0;
corner.Fx = -1000;

tire.updateWheelDynamics(corner, 0, 500, 0.001, 0.5, 20);

verifyEqual(testCase, corner.angularVelocity, 0, 'AbsTol', 1e-12);
end

function tire = createPacejkaTire()
tire = lts.components.Tire.PacejkaTire('43105_18x7.5_10_R25B_7.tir');
end
