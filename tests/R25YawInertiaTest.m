function tests = R25YawInertiaTest
% R25YAWINERTIATEST Locks the geometry-derived yaw inertia for R25: the
% former value was the unmeasured 130 kg*m^2 baseline placeholder, which
% sits outside the 83-120 kg*m^2 geometry band from the 2026-08-30 audit.
tests = functiontests(localfunctions);
end

function testR25YawInertiaMatchesGeometryDerivation(testCase)
cfg = lts.vehicles.R25();
cornerMassKg = 13 * 0.45359237 + 9.3;   % wheel assembly + unsprung corner
frontArmM = cfg.wheelbase * (1 - cfg.staticFrontWeight);
rearArmM = cfg.wheelbase * cfg.staticFrontWeight;
halfTrackM = cfg.trackWidth / 2;
sprungMassKg = cfg.totalMass - 4 * cornerMassKg;
expected = 2 * cornerMassKg * (frontArmM^2 + halfTrackM^2) ...
    + 2 * cornerMassKg * (rearArmM^2 + halfTrackM^2) ...
    + sprungMassKg * 0.50^2;
verifyEqual(testCase, cfg.yawInertia, expected, 'RelTol', 1e-12);
% Inside the audit's geometry band (the old placeholder 130 sat outside).
verifyGreaterThanOrEqual(testCase, cfg.yawInertia, 83);
verifyLessThanOrEqual(testCase, cfg.yawInertia, 120);
end
