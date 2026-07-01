%% run_simulation.m - FSAE Transient Lap Time Simulation
% Entry point script that configures and runs the simulation
%
% Architecture:
%   - The CAR is defined by a VehicleConfig from the +vehicles package
%     (e.g. vehicles.baseline). Swap cars by changing that one line.
%   - VehicleManager.fromConfig turns a config into a wired vehicle.
%   - The TRACK, driver tuning, and timestep are scenario settings and
%     stay in this script.
%   - DriverModel decides throttle/brake inputs based on track lookahead.
%   - Simulator runs the physics loop: state + inputs -> next state.

clear; clc; close all;

%% ====================================================================
%  SELECT TRACK TYPE
%  Options: 'straight10', 'straight', 'oval', 'skidpad', 'autocross', 'busstop', 'slalom', '90turn', '2026enduro'
%  ====================================================================
trackType = '2026enduro';

%% ====================================================================
%  SELECT VEHICLE CONFIGURATION
%  Add new cars in src/+vehicles/<name>.m, then reference them here.
%  ====================================================================
config = vehicles.baseline();

%% ====================================================================
%  DISPLAY OPTIONS
%  Set to true to show all graphs in a single window
%  ====================================================================
singleWindow = false;

% Export MoTeC CSV and .ld files after the simulation completes.
exportMoTeC = true;

% Simulation timestep [s]
dt = 0.001;

fprintf('=== FSAE Transient Lap Time Simulation ===\n\n');

%% ====================================================================
%  BUILD TRACK
%  ====================================================================
if lower(trackType) == "2026enduro"
    track = components.WaypointTrack.loadMat('tracks/2026_endurance_track_pixel_units.mat');
else
    track = components.TestTrack(trackType);
end
fprintf('Track: TestTrack (''%s'', %.1f m, %d points)\n', ...
    trackType, track.getTotalLength(), size(track.getTrackPoints(), 1));
fprintf('\n');

%% ====================================================================
%  BUILD VEHICLE FROM CONFIG
%  ====================================================================
vehicle = VehicleManager.fromConfig(config, track, dt);

%% ====================================================================
%  BUILD DRIVER MODEL AND SIMULATOR, THEN RUN
%  ====================================================================
driver    = DriverModel(vehicle);
simulator = Simulator(vehicle, driver, dt);

initialState = VehicleState('s', 0, 'speed', 0.1);
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
    scriptDir = fileparts(mfilename('fullpath'));
    exportDir = fullfile(scriptDir, '..', 'exports');
    exportBase = fullfile(exportDir, sprintf('motec_%s_%s', ...
        trackType, datestr(now, 'yyyymmdd_HHMMSS')));
    TelemetryExporter.exportToMoTeCLog( ...
        stateLog, [exportBase '.csv'], ...
        'OutputFile', [exportBase '.ld'], ...
        'Frequency', 1 / dt, ...
        'VehicleWeight', round(vehicle.totalMass), ...
        'VenueName', trackType, ...
        'EventName', 'FSAE LTS Simulation', ...
        'VehicleType', 'FSAE');
end

%% ====================================================================
%  COMPUTE PER-COMPONENT AERO AT FINAL STATE (for reporting)
%  ====================================================================
% perComp = aero.computePerComponent(vehicle.state);  % vehicle.state has vehicleManager set
% fprintf('\n=== Aero Component Breakdown (at final speed) ===\n');
% for i = 1:numel(perComp)
%     fprintf('  %-16s | DF=%7.1f N | Drag=%6.1f N | x=%.2f m\n', ...
%         perComp(i).name, perComp(i).downforce, perComp(i).drag, perComp(i).xPosition);
% end

%% ====================================================================
%  PLOT RESULTS
%  ====================================================================
GraphPlotter.plotAll(stateLog, lapTime, track, vehicle, vehicle.aero, singleWindow);

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
