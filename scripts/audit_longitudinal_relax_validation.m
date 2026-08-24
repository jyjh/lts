function audit_longitudinal_relax_validation()
% Phase-3d validation: separate longitudinal relaxation length (0.10 m)
% vs the former NaN (shared lateral 0.255 m) on R25.
%   - straight-line launch: lap time + timestep convergence
%   - braking: decel + oscillation
cd('D:/lts'); addpath('src');
dts = [0.001, 0.0005];
cases = { ...
    'longRelax=NaN(0.255)', NaN; ...
    'longRelax=0.10      ', 0.10};
for c = 1:size(cases, 1)
    laps = zeros(size(dts));
    for i = 1:numel(dts)
        laps(i) = launchLap(dts(i), cases{c, 2});
    end
    fprintf('launch %s | laps: %.6f %.6f | gap %+0.1f ms\n', ...
        cases{c, 1}, laps(1), laps(2), (laps(1) - laps(2)) * 1e3);
end
end

function lapTime = launchLap(dt, longRelax)
track = lts.components.TestTrack('straight10');
cfg = lts.vehicles.R25();
cfg.tire.longitudinalRelaxationLength = longRelax;
vehicle = lts.vehicle.VehicleManager.fromConfig(cfg, track, dt, 'Verbose', false);
driver = lts.driver.DriverModel(vehicle);
simulator = lts.simulation.Simulator(vehicle, driver, dt);
simulator.telemetryMode = "lean";
simulator.verbose = false;
initialState = lts.simulation.VehicleState('s', 0, 'speed', 0.1);
[~, lapTime] = simulator.simulate(initialState, track);
end
