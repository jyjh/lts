function results = benchmark_anti_roll_bar(outputFile, referenceFile, repetitions)
% Compare ARB-active performance while checking every telemetry field exactly.
% Requires local TTC tire data. Run from the integration root, e.g.:
%   addpath('src'); addpath('scripts');
%   benchmark_anti_roll_bar('exports/arb-before.mat', '', 3);
%   benchmark_anti_roll_bar('exports/arb-after.mat', 'exports/arb-before.mat', 5);
if nargin < 2, referenceFile = ''; end
if nargin < 3, repetitions = 3; end
validateattributes(repetitions, {'numeric'}, {'scalar','integer','positive'});
results = struct();
for item = {'installed', 'stiff'}
    name = item{1};
    runCase(name, 20);
    elapsed = zeros(1, repetitions);
    for idx = 1:repetitions
        [elapsed(idx), trace] = runCase(name, 200);
        if idx > 1
            assert(isequaln(trace, previous), ...
                'benchmark_anti_roll_bar:NonDeterministic', '%s outputs changed.', name);
        end
        previous = trace;
    end
    results.(name) = struct('seconds', elapsed, 'trace', trace);
    fprintf('%s: median %.6f s\n', name, median(elapsed));
end
if ~isempty(referenceFile)
    reference = load(referenceFile, 'results');
    for item = {'installed', 'stiff'}
        name = item{1};
        assert(isequaln(results.(name).trace, reference.results.(name).trace), ...
            'benchmark_anti_roll_bar:PhysicsChanged', '%s telemetry differs.', name);
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

function [elapsed, trace] = runCase(name, steps)
dt = .001;
cfg = lts.vehicles.baseline();
if strcmp(name, 'stiff')
    % Unit geometry makes this exactly 2 MN/m at both axles and forces
    % subcycling below 1 ms, exercising repeated capability lookups.
    for axle = {'frontArb', 'rearArb'}
        cfg.suspension.(axle{1}) = struct('stiffness', 2e6, ...
            'motionRatio', 1, 'leverArm', 1, 'enabled', true);
    end
end
track = lts.components.TestTrack('straight10');
vm = lts.vehicle.VehicleManager.fromConfig(cfg, track, dt, 'Verbose', false);
sim = lts.simulation.Simulator(vm, [], dt);
sim.resetForSimulation();
state = lts.simulation.VehicleState('speed', 12, 'vx', 12, ...
    'vy', 0, 'yaw', 0, 'x', 0, 'y', 0);
state.vehicleManager = vm;
lts.simulation.DrivelineSupport.initializeWheelSpeeds(vm, state.speed);
ref = sim.freeReferenceForState(0, 0, 0, 0);
builder = lts.telemetry.StateLogBuilder(vm, 'full');
builder.beginRun(steps, [], true);
timer = tic;
for idx = 1:steps
    input = struct('throttle', .1 * double(idx <= 60), ...
        'brake', .1 * double(idx > 60 && idx <= 100), ...
        'steer', .03 * double(idx > 100));
    [next, forces] = sim.step(state, input, ref);
    builder.logStep(idx, state, next, input, forces);
    state = next;
    ref = sim.cachedNextRef;
end
trace = builder.finish();
elapsed = toc(timer);
end
