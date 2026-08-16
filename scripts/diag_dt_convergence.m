function diag_dt_convergence(sigmaFz)
% DIAG_DT_CONVERGENCE Reproduce the straight10 launch dt-sweep used by
%   SimulatorPhysicsRegressionTest and print the raw values.
if nargin < 1
    sigmaFz = 0.15748;
end
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));

dts = [0.001, 0.00075, 0.0005];
lapTimes = zeros(size(dts));
finalSpeeds = zeros(size(dts));
finalX = zeros(size(dts));
for idx = 1:numel(dts)
    dt = dts(idx);
    track = lts.components.TestTrack('straight10');
    config = lts.vehicles.R25();
    config.tire.normalLoadRelaxationLength = sigmaFz;
    vehicle = lts.vehicle.VehicleManager.fromConfig(config, track, dt, 'Verbose', false);
    driver = lts.driver.DriverModel(vehicle);
    simulator = lts.simulation.Simulator(vehicle, driver, dt);
    simulator.telemetryMode = "lean";
    simulator.verbose = false;
    initialState = lts.simulation.VehicleState('s', 0, 'speed', 0.1);
    [stateLog, lapTime] = simulator.simulate(initialState, track);
    lapTimes(idx) = lapTime;
    finalSpeeds(idx) = stateLog.speed(end);
    finalX(idx) = stateLog.x(end);
end
fprintf('\nsigmaFz=%g\n', sigmaFz);
for idx = 1:numel(dts)
    fprintf('dt=%.5f : lapTime %.5f finalSpeed %.6f finalX %.4f | relSpeedErr vs fine %.2e\n', ...
        dts(idx), lapTimes(idx), finalSpeeds(idx), finalX(idx), ...
        abs(finalSpeeds(idx) - finalSpeeds(end)) / finalSpeeds(end));
end
end
