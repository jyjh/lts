function results = benchmark_simulation(outputFile, referenceFile, repetitions)
% Benchmark unchanged-timestep physics and full telemetry, optionally checking
% every recorded channel against a previous run of this same benchmark.
% Run from the repository root, e.g.:
%   addpath('src'); addpath('scripts');
%   benchmark_simulation('exports/before.mat', '', 3);
%   benchmark_simulation('exports/after.mat', 'exports/before.mat', 3);
if nargin < 2, referenceFile = ''; end
if nargin < 3, repetitions = 3; end
validateattributes(repetitions, {'numeric'}, {'scalar','integer','positive'});
results = struct();
names = {'launch', 'transient'};
for c = 1:numel(names)
    name = names{c};
    runCase(name, true);
    elapsed = zeros(1, repetitions);
    for k = 1:repetitions
        [elapsed(k), trace] = runCase(name, false);
        if k > 1
            assert(isequaln(trace, previousTrace), ...
                'benchmark_simulation:NonDeterministic', 'Repeated %s outputs changed.', name);
        end
        previousTrace = trace;
    end
    results.(name) = struct('seconds', elapsed, 'trace', trace);
    fprintf('%s: median %.6f s (%d repetitions)\n', name, median(elapsed), repetitions);
end
if ~isempty(referenceFile)
    reference = load(referenceFile, 'results');
    for c = 1:numel(names)
        name = names{c};
        assert(isequaln(results.(name).trace, reference.results.(name).trace), ...
            'benchmark_simulation:PhysicsChanged', '%s telemetry differs from reference.', name);
        fprintf('%s: all channels identical; speedup %.3fx\n', name, ...
            median(reference.results.(name).seconds) / median(results.(name).seconds));
    end
end
if nargin >= 1 && ~isempty(outputFile)
    folder = fileparts(outputFile);
    if ~isempty(folder) && ~isfolder(folder), mkdir(folder); end
    save(outputFile, 'results');
end
end

function [elapsed, trace] = runCase(name, warmup)
dt = 0.001;
track = lts.components.TestTrack('straight10');
vm = lts.vehicle.VehicleManager.fromConfig(lts.vehicles.R25(), track, dt, 'Verbose', false);
if strcmp(name, 'launch')
    sim = lts.simulation.Simulator(vm, lts.driver.DriverModel(vm), dt);
    sim.verbose = false;
    if warmup, sim.stopTime = 0.02; end
    state = lts.simulation.VehicleState('speed', 0.1);
    timer = tic;
    [trace.log, trace.lapTime] = sim.simulate(state, track);
    elapsed = toc(timer);
else
    sim = lts.simulation.Simulator(vm, [], dt);
    sim.resetForSimulation();
    state = lts.simulation.VehicleState('speed', 15, 'x', 0, 'y', 0, 'yaw', 0);
    state.vehicleManager = vm;
    lts.simulation.DrivelineSupport.initializeWheelSpeeds(vm, state.speed);
    ref = struct('heading',0,'x',0,'y',0,'s',0,'mu',1,'curvature',0,'referenceMode','free');
    n = 1000;
    if warmup, n = 20; end
    builder = lts.telemetry.StateLogBuilder(vm, 'full');
    builder.beginRun(n, [], true);
    timer = tic;
    for k = 1:n
        input = struct('throttle', double(k <= 400) * 0.6, ...
            'brake', double(k > 400 && k <= 600) * 0.3, ...
            'steer', double(k > 600) * 0.03);
        [next, forces] = sim.step(state, input, ref);
        builder.logStep(k, state, next, input, forces);
        state = next;
    end
    trace.log = builder.finish();
    elapsed = toc(timer);
end
end
