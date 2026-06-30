%% profile_sim.m - one-off profiler dump for the FSAE LTS hot path
% Runs a skidpad lap under the MATLAB profiler and prints the top hotspots.
% Usage:  addpath('src'); profile_sim
function profile_sim(trackType)
    arguments
        trackType string = 'skidpad'
    end
    dt = 0.001;
    config  = vehicles.baseline();
    track   = components.TestTrack(trackType);
    vehicle = VehicleManager.fromConfig(config, track, dt);
    driver  = DriverModel(vehicle);
    simulator = Simulator(vehicle, driver, dt);
    initialState = VehicleState('s', 0, 'speed', 0.1);

    profile clear
    profile on
    [stateLog, lapTime] = simulator.simulate(initialState, track);
    profile off

    info = profile('info');
    fprintf('\n=== PROFILE (%s) ===\nLap: %.3f s, %d steps\n\n', ...
        trackType, lapTime, numel(stateLog.time));

    % Build a function-name -> total-time table and show the top 25 by
    % inclusive time, plus self time.
    nF = numel(info.FunctionTable);
    names = cell(nF, 1);
    total = zeros(nF, 1);
    selft = zeros(nF, 1);
    ncalls = zeros(nF, 1);
    for i = 1:nF
        names{i} = info.FunctionTable(i).FunctionName;
        total(i) = info.FunctionTable(i).TotalTime;
        selft(i) = info.FunctionTable(i).TotalTime - ...
            sum([info.FunctionTable(i).Children.TotalTime]);
        ncalls(i) = info.FunctionTable(i).NumCalls;
    end
    [~, idx] = sort(total, 'descend');

    fprintf('%-45s %10s %10s %10s %10s\n', 'Function', 'Total(s)', 'Self(s)', 'Calls', 'us/call');
    fprintf('%s\n', repmat('-', 1, 90));
    for k = 1:min(25, nF)
        i = idx(k);
        if total(i) <= 0, continue; end
        upc = total(i) / max(ncalls(i), 1) * 1e6;
        fprintf('%-45s %10.3f %10.3f %10d %10.1f\n', ...
            shortName(names{i}), total(i), selft(i), ncalls(i), upc);
    end
end

function s = shortName(name)
    % Trim package/path noise for readability.
    [~, s] = fileparts(name);
    if isempty(s), s = name; end
end
