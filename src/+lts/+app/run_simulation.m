function [stateLog, lapTime] = run_simulation()
% RUN_SIMULATION FSAE Transient Lap Time Simulation
% Entry point function that configures and runs the simulation
%
% Architecture:
%   - The CAR is defined by a lts.vehicle.VehicleConfig from lts.vehicles
%     (e.g. lts.vehicles.baseline). Swap cars by changing that one line.
%   - lts.vehicle.VehicleManager.fromConfig turns a config into a wired vehicle.
%   - The TRACK, driver tuning, and timestep are scenario settings and
%     stay in this script.
%   - lts.driver.DriverModel decides throttle/brake inputs based on track lookahead.
%   - lts.simulation.Simulator runs the physics loop: state + inputs -> next state.

clc; close all;
stateLog = [];
lapTime = NaN;

%% ====================================================================
%  SELECT TRACK TYPE
%  Options: 'straight10', 'straight', 'straight75', 'oval', 'skidpad', 'autocross', 'busstop', 'slalom', '90turn', '2026enduro'
%  ====================================================================
trackType = 'skidpad';

%% ====================================================================
%  SELECT VEHICLE CONFIGURATION
%  Add new cars in src/+lts/+vehicles/<name>.m, then reference them here.
%  ====================================================================
config = lts.vehicles.R25();

%% ====================================================================
%  DISPLAY OPTIONS
%  Set to true to show all graphs in a single window
%  ====================================================================
singleWindow = false;

% Set to false to skip all graphs after the simulation completes.
showPlots = false;

% Export MoTeC CSV and .ld files after the simulation completes.
exportMoTeC = true;

% Simulation timestep [s]
dt = 0.001;

fprintf('=== FSAE Transient Lap Time Simulation ===\n\n');

%% ====================================================================
%  BUILD TRACK
%  ====================================================================
if lower(trackType) == "2026enduro"
    % Direction is the FSAE endurance travel direction. The exporter also
    % bakes this into points_m ordering, but passing it here makes the
    % intent explicit and forces a flip + warning if the .mat on disk was
    % re-exported in the opposite direction (or is a stale copy).
    track = lts.components.WaypointTrack.loadMat( ...
        'tracks/endurance_track_grid_25ft_from_matlab_smoothed.mat');
    track.Width = 5.0;
    fprintf('Track: 2026 Endurance (''%s'', %.1f m, %d points, direction: %s)\n', ...
        trackType, track.getTotalLength(), size(track.getTrackPoints(), 1), ...
        track.getDirection());
else
    track = lts.components.TestTrack(trackType);
    fprintf('Track: TestTrack (''%s'', %.1f m, %d points)\n', ...
        trackType, track.getTotalLength(), size(track.getTrackPoints(), 1));
end
fprintf('\n');

%% ====================================================================
%  BUILD VEHICLE FROM CONFIG
%  ====================================================================
vehicle = lts.vehicle.VehicleManager.fromConfig(config, track, dt);

%% ====================================================================
%  BUILD DRIVER MODEL AND SIMULATOR, THEN RUN
%  ====================================================================
driver    = lts.driver.DriverModel(vehicle);
simulator = lts.simulation.Simulator(vehicle, driver, dt);

initialState = lts.simulation.VehicleState('s', 0, 'speed', 0.1);
[stateLog, lapTime] = simulator.simulate(initialState, track);

hasRecordedTelemetry = isstruct(stateLog) && isfield(stateLog, 'time') && ...
    ~isempty(stateLog.time);
if ~hasRecordedTelemetry
    warning('run_simulation:NoRecordedTelemetry', ...
        ['No recorded telemetry was produced for track "%s". ' ...
        'The car likely stopped or left the track before the timed lap window. ' ...
        'Skipping export, plots, and summary.'], trackType);
    return;
end

if exportMoTeC
    repoRoot = lts.util.repoRoot(mfilename('fullpath'));
    exportDir = fullfile(repoRoot, 'exports');
    exportBase = fullfile(exportDir, sprintf('motec_%s_%s_%s', ...
        trackType, config.name, datestr(now, 'yyyymmdd_HHMMSS')));
    lts.telemetry.TelemetryExporter.exportToMoTeCLog( ...
        stateLog, [exportBase '.csv'], ...
        'OutputFile', [exportBase '.ld'], ...
        'Frequency', 1 / dt, ...
        'VehicleWeight', round(vehicle.totalMass), ...
        'VehicleId', config.name, ...
        'VenueName', trackType, ...
        'EventName', 'FSAE LTS Simulation', ...
        'VehicleType', 'FSAE');
end

%% ====================================================================
%  PLOT RESULTS
%  ====================================================================
if showPlots
    lts.telemetry.GraphPlotter.plotAll(stateLog, lapTime, track, vehicle, vehicle.aero, singleWindow);
end

% --- Summary ---
speedKmh = stateLog.speedKmh;
axG = stateLog.ax / 9.81;
ayG = stateLog.ay / 9.81;
pitchDeg = stateLog.pitchAngle * (180/pi);
rollDeg = stateLog.rollAngle * (180/pi);
twistDeg = stateLog.twistAngle * (180/pi);

fprintf('\n=== Vehicle Summary ===\n');
fprintf('Mass:       %.0f kg\n', vehicle.totalMass);
fprintf('Peak Speed: %.1f km/h\n', max(speedKmh));
fprintf('Lap Time:   %.3f s\n', lapTime);
fprintf('Avg Speed:  %.1f km/h\n', mean(speedKmh));
fprintf('Peak ax:    %.2f g\n', max(axG));
fprintf('Peak ay:    %.2f g\n', max(abs(ayG)));
fprintf('Peak Downforce: %.0f N (%.1f kg)\n', ...
    max(stateLog.F_downforce), max(stateLog.F_downforce)/9.81);
fprintf('Peak Drag:      %.0f N\n', max(stateLog.F_drag));
if isfield(stateLog, 'motorRPM')
    fprintf('Peak Motor RPM: %.0f rpm\n', max(stateLog.motorRPM));
end
if isfield(stateLog, 'rpmLimitActive')
    fprintf('RPM Limiter Hits: %d\n', nnz(stateLog.rpmLimitActive));
end
fprintf('Peak Pitch:     %.3f deg\n', max(abs(pitchDeg)));
fprintf('Peak Roll:      %.3f deg\n', max(abs(rollDeg)));
fprintf('Peak Twist:     %.3f deg\n', max(abs(twistDeg)));
end
