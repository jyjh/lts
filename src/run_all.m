%% run_all.m - Batch runner: every car x every track, with MoTeC export
%
% Convenience script for regression testing: instead of editing run_simulation.m
% and re-running by hand for each circuit, this loops a car list across a track
% list, runs the full simulation pipeline per combo, and exports MoTeC logs.
%
% The pipeline is inlined here (build track -> VehicleManager -> DriverModel ->
% Simulator -> simulate -> TelemetryExporter) for the same reason benchmark.m
% inlines it: run_simulation.m is a script that clears the workspace on entry,
% so it can't be called in a loop with parameters. Each combo rebuilds the
% vehicle/driver/simulator fresh so no learned driver state leaks between runs.
%
% Usage (from MATLAB):
%   addpath('src'); run_all
%
% Edit the car/track cell arrays below to narrow the run. Output MoTeC files
% are written to <repo>/exports/motec_<track>_<config.name>_<timestamp>.{csv,ld},
% exactly as in run_simulation.m.
%
% Note: vehicles.baseline does not set cfg.name, so its files read
% '..._VehicleConfig_...' (the VehicleConfig default). That is inherent to the
% config, not this script.

clear; clc; close all;

%% ====================================================================
%  EDIT THESE LISTS TO NARROW THE RUN
%  Cars   -> function names in src/+vehicles/<name>.m
%  Tracks -> '2026enduro' or any TestTrack type
%            ('straight10','straight','straight75','oval','skidpad',
%             'autocross','busstop','slalom','90turn')
%  ====================================================================
cars   = {'R25', 'R26_base', 'baseline'};
tracks = {'2026enduro', 'autocross', 'straight75', 'skidpad'};

dt           = 0.001;   % simulation timestep [s]
exportMoTeC  = true;    % export MoTeC CSV + .ld per combo (the point of this script)

%% ====================================================================
%  SETUP PATHS
%  ====================================================================
scriptDir = fileparts(mfilename('fullpath'));
repoRoot  = fileparts(scriptDir);
exportDir = fullfile(repoRoot, 'exports');
tracksDir = fullfile(repoRoot, 'tracks');
if ~exist(exportDir, 'dir'); mkdir(exportDir); end

nCombos = numel(cars) * numel(tracks);
fprintf('=== FSAE LTS batch run: %d car(s) x %d track(s) = %d combo(s) ===\n', ...
    numel(cars), numel(tracks), nCombos);
fprintf('    exportMoTeC = %d, dt = %.0f ms\n\n', exportMoTeC, dt * 1000);

%% ====================================================================
%  RUN EVERY CAR x TRACK COMBO
%  ====================================================================
results = cell(nCombos, 1);   % each row: {car, track, lapTime, peakSpeed, status}
idx = 0;
tBatchStart = tic;

for ic = 1:numel(cars)
    for it = 1:numel(tracks)
        idx = idx + 1;
        carName   = cars{ic};
        trackType = tracks{it};
        fprintf('--- [%d/%d] %s @ %s ---\n', idx, nCombos, carName, trackType);

        [lapTime, peakSpeed, status] = runOne( ...
            carName, trackType, dt, exportMoTeC, tracksDir, exportDir);

        if isnan(lapTime)
            fprintf('    -> %s (lap -, peak - km/h)\n\n', status);
        else
            fprintf('    -> %s (lap %.3f s, peak %.1f km/h)\n\n', status, lapTime, peakSpeed);
        end
        results{idx} = {carName, trackType, lapTime, peakSpeed, status};
    end
end

%% ====================================================================
%  SUMMARY
%  ====================================================================
printSummary(results, toc(tBatchStart), exportDir);


%% ===================== LOCAL FUNCTIONS =====================

function [lapTime, peakSpeed, status] = runOne(carName, trackType, dt, ...
        exportMoTeC, tracksDir, exportDir)
    % Run one car @ track combo through the full pipeline and (optionally)
    % export MoTeC. Never throws: failures are caught and reported as status
    % so a single bad combo does not abort the whole batch.

    lapTime   = NaN;
    peakSpeed = NaN;
    status    = 'ok';

    try
        config = feval(['vehicles.' carName]);
        track  = buildTrack(trackType, tracksDir);

        vehicle   = VehicleManager.fromConfig(config, track, dt);
        driver    = DriverModel(vehicle);
        simulator = Simulator(vehicle, driver, dt);
        initState = VehicleState('s', 0, 'speed', 0.1);
        [stateLog, lapTime] = simulator.simulate(initState, track);

        % Same telemetry-present guard as run_simulation.m: skip export if the
        % car stopped or left the track before the timed lap window.
        hasTelemetry = isstruct(stateLog) && isfield(stateLog, 'time') && ...
            ~isempty(stateLog.time);
        if ~hasTelemetry
            warning('run_all:NoTelemetry', ...
                ['No telemetry for %s @ %s (car stopped or left the track ' ...
                'before the timed window). Skipping export.'], carName, trackType);
            status = 'no telemetry';
            return;
        end

        peakSpeed = max(stateLog.speedKmh);

        if exportMoTeC
            exportBase = fullfile(exportDir, sprintf('motec_%s_%s_%s', ...
                trackType, config.name, datestr(now, 'yyyymmdd_HHMMSS')));
            TelemetryExporter.exportToMoTeCLog( ...
                stateLog, [exportBase '.csv'], ...
                'OutputFile', [exportBase '.ld'], ...
                'Frequency', 1 / dt, ...
                'VehicleWeight', round(vehicle.totalMass), ...
                'VehicleId', config.name, ...
                'VenueName', trackType, ...
                'EventName', 'FSAE LTS Simulation', ...
                'VehicleType', 'FSAE');
            status = 'exported';
        else
            status = 'ok (no export)';
        end
    catch err
        status = sprintf('error: %s', err.message);
        fprintf('    !! caught: %s\n', status);
    end
end


function track = buildTrack(trackType, tracksDir)
    % Build a track by type, mirroring run_simulation.m's two branches:
    %   '2026enduro' -> load the surveyed .mat via WaypointTrack.loadMat
    %   anything else -> procedural components.TestTrack(trackType)
    if strcmpi(trackType, '2026enduro')
        track = components.WaypointTrack.loadMat( ...
            fullfile(tracksDir, 'endurance_track_grid_25ft_from_matlab_smoothed.mat'));
        track.Width = 5.0;
    else
        track = components.TestTrack(trackType);
    end
end


function printSummary(results, elapsedSec, exportDir)
    fprintf('=== BATCH SUMMARY (%.1f s wall) ===\n', elapsedSec);
    fprintf('%-12s %-13s %9s %12s   %s\n', ...
        'Car', 'Track', 'Lap [s]', 'Peak [km/h]', 'Status');
    fprintf('%s\n', repmat('-', 1, 72));
    for k = 1:numel(results)
        r = results{k};
        [carName, trackType, lapTime, peakSpeed, status] = deal(r{:});
        lapStr  = ternaryStr(isnan(lapTime),   '-', sprintf('%.3f', lapTime));
        peakStr = ternaryStr(isnan(peakSpeed), '-', sprintf('%.1f', peakSpeed));
        fprintf('%-12s %-13s %9s %12s   %s\n', ...
            carName, trackType, lapStr, peakStr, status);
    end
    fprintf('\nMoTeC exports (if any): %s\n', exportDir);
    fprintf('\n');
end


function out = ternaryStr(cond, a, b)
    if cond
        out = a;
    else
        out = b;
    end
end
