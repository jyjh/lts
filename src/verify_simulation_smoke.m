function verify_simulation_smoke()
% VERIFY_SIMULATION_SMOKE Run headless end-to-end lap checks.
%
% This is intentionally broader than the focused physics invariants: it
% exercises the same vehicle setup used by run_simulation.m across every
% built-in test track, with SimpleTire always and PacejkaTire when the
% external MFeval dependency is available.

trackTypes = {'straight', 'oval', 'skidpad', 'autocross', 'busstop', '90turn'};
tireKinds = {'simple'};

try
    components.Tire.PacejkaTire('43105_18x7.5_10_R25B_7.tir');
    tireKinds{end + 1} = 'pacejka';
catch ME
    fprintf('Skipping Pacejka smoke checks: %s\n', ME.message);
end

for tireIdx = 1:numel(tireKinds)
    tireKind = tireKinds{tireIdx};
    for trackIdx = 1:numel(trackTypes)
        trackType = trackTypes{trackIdx};
        fprintf('Smoke: tire=%s track=%s\n', tireKind, trackType);

        [vehicle, simulator, track] = buildSmokeVehicle(tireKind, trackType);
        initialState = VehicleState('s', 0, 'speed', 0.1);
        [stateLog, lapTime] = simulator.simulate(initialState, track);

        assertSmokeRun(stateLog, lapTime, vehicle, track, tireKind, trackType);
    end
end

fprintf('Simulation smoke checks passed.\n');
end

function [vehicle, simulator, track] = buildSmokeVehicle(tireKind, trackType)
frontWing = components.Aero.FrontWing( ...
    0.9, 0.08, 0.45, 0.35, -5.0, 0.03);
rearWing = components.Aero.RearWing( ...
    -0.85, 0.45, 0.55, 0.8, 3.0, 0.005);
floor = components.Aero.UnderbodyFloor( ...
    0.0, 0.035, 0.4, 0.10, -8.0, 0.015, 0.6);

aero = components.Aero.AeroManager();
aero = aero.addComponent(frontWing);
aero = aero.addComponent(rearWing);
aero = aero.addComponent(floor);

powertrain = components.Powertrain.EMRAX228Powertrain();

switch lower(tireKind)
    case 'simple'
        tire = components.Tire.SimpleTire();
    case 'pacejka'
        tire = components.Tire.PacejkaTire('43105_18x7.5_10_R25B_7.tir');
    otherwise
        error('verify_simulation_smoke:UnknownTire', ...
            'Unknown tire kind "%s".', tireKind);
end

track = components.TestTrack(trackType);
vehicle = VehicleManager(aero, [], powertrain, tire, track);
vehicle.chassis = components.Chassis.SimpleChassis(vehicle);
vehicle.suspension = components.Suspension.SuspensionManager( ...
    vehicle, ...
    0.55, ...
    45000, 3000, 4500, ...
    42000, 2800, 4200, ...
    0.95, ...
    0.025, ...
    200000, ...
    200000, ...
    25);
vehicle.suspension.warmup(vehicle.totalMass, 0.001);

driver = DriverModel(vehicle);
simulator = Simulator(vehicle, driver, 0.001);
end

function assertSmokeRun(stateLog, lapTime, vehicle, track, tireKind, trackType)
label = sprintf('%s/%s', tireKind, trackType);
trackLen = track.getTotalLength();

assert(~isempty(stateLog.time), '%s produced an empty state log.', label);
assert(isfinite(lapTime) && lapTime > 0, ...
    '%s produced invalid lap time %.12g.', label, lapTime);
assert(stateLog.s(end) >= trackLen, ...
    '%s stopped at %.3f m before track length %.3f m.', ...
    label, stateLog.s(end), trackLen);
assert(all(stateLog.onTrack), '%s left the track.', label);
assert(all(diff(stateLog.s) >= -1e-9), ...
    '%s regressed in arc-length progress.', label);

assertAllFinite(stateLog, label);

assert(max(abs(stateLog.pitchAngle)) < deg2rad(5), ...
    '%s pitch exceeded 5 deg: %.3f deg.', ...
    label, rad2deg(max(abs(stateLog.pitchAngle))));
assert(max(abs(stateLog.rollAngle)) < deg2rad(10), ...
    '%s roll exceeded 10 deg: %.3f deg.', ...
    label, rad2deg(max(abs(stateLog.rollAngle))));
assert(max(abs(stateLog.lateralError)) <= vehicle.trackHalfWidth + 1e-6, ...
    '%s lateral error exceeded track half-width.', label);
assert(max(stateLog.speed) <= vehicle.maxSpeed + 1e-6, ...
    '%s exceeded vehicle max speed.', label);

normalLoads = [stateLog.Fz_FL, stateLog.Fz_FR, stateLog.Fz_RL, stateLog.Fz_RR];
assert(all(normalLoads(:) >= -1e-9), ...
    '%s produced negative tire normal load.', label);

tireUsage = [stateLog.tireUsage_FL, stateLog.tireUsage_FR, ...
    stateLog.tireUsage_RL, stateLog.tireUsage_RR];
assert(all(tireUsage(:) <= 1 + 1e-8), ...
    '%s tire usage exceeded 1.0.', label);

assert(all(stateLog.dt > 0), '%s logged nonpositive dt.', label);
assert(max(stateLog.dt) <= 0.010 + 1e-12, ...
    '%s adaptive dt exceeded the configured max.', label);
subNominalDt = stateLog.dt < 0.001 - 1e-12;
if any(subNominalDt)
    assert(all(stateLog.controlS(subNominalDt) >= trackLen - 1.0), ...
        '%s adaptive dt dropped below the configured min away from the finish.', label);
end
end

function assertAllFinite(stateLog, label)
fields = fieldnames(stateLog);
for i = 1:numel(fields)
    value = stateLog.(fields{i});
    if isnumeric(value)
        assert(all(isfinite(value(:))), ...
            '%s logged non-finite values in %s.', label, fields{i});
    end
end
end
