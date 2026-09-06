function tests = AeroWiringTest
% AEROWIRINGTEST Pins the cfg.aero component-split wiring end to end:
% fromConfig dispatches through lts.components.Aero.buildFromConfig, the
% split reproduces the whole-car measured datum at the nominal attitude,
% and the added pitch/height response moves the aero map in the physical
% directions.
tests = functiontests(localfunctions);
end

function [vmSplit, vmWhole] = buildVehicles(testCase)
    assumeTrue(testCase, tireDataAvailable(), 'TTC tire data not present: see src/+lts/+components/+Tire/README.md');
track = lts.components.TestTrack('straight10');
cfgSplit = lts.vehicles.R25();
cfgWhole = lts.vehicles.R25();
cfgWhole.aero = rmfield(cfgWhole.aero, 'components');
vmSplit = lts.vehicle.VehicleManager.fromConfig( ...
    cfgSplit, track, 0.001, 'Verbose', false);
vmWhole = lts.vehicle.VehicleManager.fromConfig( ...
    cfgWhole, track, 0.001, 'Verbose', false);
end

function state = aeroState(vehicleManager, speed, pitchAngle, rideHeight)
state = lts.simulation.VehicleState('speed', speed, 'vx', speed);
state.vehicleManager = vehicleManager;
state.pitchAngle = pitchAngle;
state.rideHeight = rideHeight;
end

function testR25SplitsIntoFourAeroDevices(testCase)
[vmSplit, ~] = buildVehicles(testCase);
verifyTrue(testCase, isa(vmSplit.aero, 'lts.components.Aero.AeroManager'));
verifyEqual(testCase, numel(vmSplit.aero.components), 4);
end

function testSplitReproducesWholeCarDatumAtNominalAttitude(testCase)
[vmSplit, vmWhole] = buildVehicles(testCase);
fSplit = vmSplit.aero.computeForces(aeroState(vmSplit, 25, 0, 0));
fWhole = vmWhole.aero.computeForces(aeroState(vmWhole, 25, 0, 0));
verifyEqual(testCase, fSplit.Fz_front, fWhole.Fz_front, 'RelTol', 1e-9);
verifyEqual(testCase, fSplit.Fz_rear, fWhole.Fz_rear, 'RelTol', 1e-9);
verifyEqual(testCase, fSplit.F_drag, fWhole.F_drag, 'RelTol', 1e-9);
end

function testNoseUpPitchReducesDownforceAndFrontShare(testCase)
[vmSplit, ~] = buildVehicles(testCase);
level = vmSplit.aero.computeForces(aeroState(vmSplit, 25, 0, 0));
noseUp = vmSplit.aero.computeForces(aeroState(vmSplit, 25, 0.02, 0));
% Braking pitch unloads the front wing and floor; the rear wing only
% partially offsets, so total downforce and the front aero share fall.
verifyLessThan(testCase, noseUp.Fz_front + noseUp.Fz_rear, ...
    level.Fz_front + level.Fz_rear);
frontShareLevel = level.Fz_front / (level.Fz_front + level.Fz_rear);
frontShareNoseUp = noseUp.Fz_front / (noseUp.Fz_front + noseUp.Fz_rear);
verifyLessThan(testCase, frontShareNoseUp, frontShareLevel);
% The response is a gentle trim on the measured map, not a cliff: at
% 0.02 rad the car keeps > 90% of its nominal downforce.
verifyGreaterThan(testCase, (noseUp.Fz_front + noseUp.Fz_rear) ...
    / (level.Fz_front + level.Fz_rear), 0.90);
end

function testRideHeightRiseReducesDownforce(testCase)
[vmSplit, ~] = buildVehicles(testCase);
nominal = vmSplit.aero.computeForces(aeroState(vmSplit, 25, 0, 0));
risen = vmSplit.aero.computeForces(aeroState(vmSplit, 25, 0, 0.01));
verifyLessThan(testCase, risen.Fz_front + risen.Fz_rear, ...
    nominal.Fz_front + nominal.Fz_rear);
end

function testFromConfigRejectsSplitThatDriftsFromTheMeasuredDatum(testCase)
    assumeTrue(testCase, tireDataAvailable(), 'TTC tire data not present: see src/+lts/+components/+Tire/README.md');
track = lts.components.TestTrack('straight10');
cfg = lts.vehicles.R25();
cfg.aero.components.body.ClA = cfg.aero.components.body.ClA + 0.01;
verifyError(testCase, ...
    @() lts.vehicle.VehicleManager.fromConfig( ...
    cfg, track, 0.001, 'Verbose', false), ...
    'lts_aero_buildFromConfig:ComponentMismatch');
end
