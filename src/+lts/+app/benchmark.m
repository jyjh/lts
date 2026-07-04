%% benchmark.m - headless timing harness for the FSAE LTS
% Runs a track end-to-end with the baseline car, no plotting/export, and
% prints wall time, steps/s, and lap time. Used to establish a perf baseline
% and to regression-check lap time after each change phase.
%
% Usage (from MATLAB):
%   addpath('src'); benchmark            % default track = 'skidpad'
%   addpath('src'); benchmark('skidpad')
%   addpath('src'); benchmark('autocross', 2)   % optional repeat count
%   addpath('src'); benchmark('skidpad', 2, "lean")
%
% Set profileOn = true below to also capture a MATLAB profile.

function benchmark(trackType, repeats, telemetryMode)
    arguments
        trackType string = 'skidpad'
        repeats (1,1) double = 1
        telemetryMode string = "full"
    end

    profileOn = false;
    dt = 0.001;

    config = lts.vehicles.baseline();
    track  = lts.components.TestTrack(trackType);

    lapTimes = zeros(repeats, 1);
    wallTimes = zeros(repeats, 1);
    stepCounts = zeros(repeats, 1);

    if profileOn
        profile clear
        profile on
    end

    for r = 1:repeats
        % Rebuild vehicle/driver/simulator fresh each repeat so no learned
        % state leaks between runs (the lts.driver.DriverModel otherwise accumulates
        % lap history across simulate() calls, drifting the lap time). Only
        % the simulate() call is timed; setup is outside the timer.
        vehicle   = lts.vehicle.VehicleManager.fromConfig(config, track, dt);
        driver    = lts.driver.DriverModel(vehicle);
        simulator = lts.simulation.Simulator(vehicle, driver, dt);
        simulator.telemetryMode = telemetryMode;
        initState = lts.simulation.VehicleState('s', 0, 'speed', 0.1);
        t0 = tic;
        [stateLog, lapTime] = simulator.simulate(initState, track);
        wallTimes(r) = toc(t0);
        lapTimes(r) = lapTime;
        stepCounts(r) = numel(stateLog.time);
    end

    if profileOn
        profile off
        profsave(profile('info'), fullfile(pwd, 'profile_results'));
    end

    % Report best/median across repeats (first run includes JIT warmup).
    [bestWall, bestIdx] = min(wallTimes);
    simSeconds = stepCounts(bestIdx) * dt;
    fprintf('\n=== BENCHMARK (%s, %s telemetry, dt=%.0fms, %d run%s) ===\n', ...
        trackType, telemetryMode, dt * 1000, repeats, ternary(repeats > 1, 's', ''));
    fprintf('Lap time:      %.3f s\n', lapTimes(bestIdx));
    fprintf('Sim steps:     %d  (%.2f s simulated)\n', stepCounts(bestIdx), simSeconds);
    fprintf('Wall time:     %.3f s (best of %d)\n', bestWall, repeats);
    if bestWall > 0
        fprintf('Steps/sec:     %.0f\n', stepCounts(bestIdx) / bestWall);
        fprintf('Realtime mult: %.1fx slower than realtime\n', bestWall / simSeconds);
    end
    if repeats > 1
        fprintf('Wall times:    %s\n', mat2str(wallTimes, 4));
    end
end

function s = ternary(cond, a, b)
    if cond, s = a; else, s = b; end
end
