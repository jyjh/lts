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

tire.updateCorner(corner, 1000, 0.05, 0, 0, 1.2, 0.2);
keysAfterLowSpeed = tire.peakMuCache.keys;
tire.updateCorner(corner, 1000, 0.05, 0, 0, 1.2, 20);
keysAfterHighSpeed = tire.peakMuCache.keys;

verifyTrue(testCase, any(contains(keysAfterLowSpeed, '_1.0')));
verifyGreaterThan(testCase, numel(keysAfterHighSpeed), numel(keysAfterLowSpeed));
end

function tire = createPacejkaTire()
tire = components.Tire.PacejkaTire('43105_18x7.5_10_R25B_7.tir');
end
