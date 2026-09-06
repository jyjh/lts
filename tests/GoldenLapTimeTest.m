function tests = GoldenLapTimeTest
tests = functiontests(localfunctions);
end

function testR25Straight10LapTimeMatchesGoldenBaseline(testCase)
    assumeTrue(testCase, tireDataAvailable(), 'TTC tire data not present: see src/+lts/+components/+Tire/README.md');
% Locks the whole physics chain (tire, suspension, chassis, aero,
% driveline, driver) to a committed lap-time baseline. If this fails with
% a small delta, a physics change shifted behavior — regenerate the
% baseline deliberately and reference the responsible commit in a
% commit message. Regenerate by running this scenario and printing
% lapTime with 12 digits:
%   matlab -batch "addpath('src'); fprintf('%.12f\n', goldenStraight10Lap())"
track = lts.components.TestTrack('straight10');
vehicle = lts.vehicle.VehicleManager.fromConfig( ...
    lts.vehicles.R25(), track, 0.001, 'Verbose', false);
driver = lts.driver.DriverModel(vehicle);
simulator = lts.simulation.Simulator(vehicle, driver, 0.001);
simulator.telemetryMode = "lean";
simulator.verbose = false;
initialState = lts.simulation.VehicleState('s', 0, 'speed', 0.1);
[~, lapTime] = simulator.simulate(initialState, track);

% Baseline generated at commit 91dee2b (Phase 4.3, 2026-08-17), after the
% audit-driven physics work: tire/suspension cleanup, attitude predictor,
% anti-geometry plumbing (0 for R25), hub-height unsprung transfer, and
% the 0.10 m longitudinal relaxation length.
%
% Regenerated to 1.880 (2026-09-05, branch fidelity/brake-and-slip-accuracy)
% for the physics-accuracy rework: grip-limited brake capacity (the
% fictional brakeForceCoefficient = 0.7 cap removed), axis-aware peak
% friction (longitudinal peak for brake/traction capability), widened
% slip-angle evaluation range, and the reverse slip-angle sign fix. The
% launch is slightly more conservative because the planner's drive
% traction now references the longitudinal peak (~1.41) instead of the
% lateral peak (~1.60).
baselineLapTime = 1.880;
verifyEqual(testCase, lapTime, baselineLapTime, 'AbsTol', 0.005, ...
    'R25 straight10 lap time drifted from the golden baseline');
end
