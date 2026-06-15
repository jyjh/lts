function verify_plotting_smoke()
% VERIFY_PLOTTING_SMOKE Exercise GraphPlotter in headless MATLAB.
%
% Run from the src/ directory with:
%   verify_plotting_smoke

oldVisible = get(0, 'DefaultFigureVisible');
cleanup = onCleanup(@() set(0, 'DefaultFigureVisible', oldVisible));
set(0, 'DefaultFigureVisible', 'off');
close all;

[vehicle, simulator, track, aero] = buildPlotVehicle();
initialState = VehicleState('s', 0, 'speed', 0.1);
[stateLog, lapTime] = simulator.simulate(initialState, track);

GraphPlotter.plotAll(stateLog, lapTime, track, vehicle, aero, false);
assert(numel(findall(0, 'Type', 'figure')) == 6, ...
    'Separate-window plotting did not create the expected six figures.');

close all;
GraphPlotter.plotAll(stateLog, lapTime, track, vehicle, aero, true);
figures = findall(0, 'Type', 'figure');
assert(numel(figures) == 1, ...
    'Single-window plotting should create exactly one figure.');
assert(numel(findall(figures(1), 'Type', 'axes')) >= 24, ...
    'Single-window plotting did not populate the dashboard grid.');

close all;
assertDuplicateTrackOverviewPlot();

close all;
assertZeroDistanceOverviewPlot();

close all;
fprintf('Plotting smoke checks passed.\n');
end

function [vehicle, simulator, track, aero] = buildPlotVehicle()
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

track = components.TestTrack('90turn');
tire = components.Tire.SimpleTire();
powertrain = components.Powertrain.EMRAX228Powertrain();
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

function assertDuplicateTrackOverviewPlot()
track = components.TestTrack('straight');
track.trackPoints = [0, 0; 5, 0; 5, 0; 10, 0];
stateLog = struct( ...
    'speedKmh', [0; 10; 20], ...
    'time', [0; 1; 2], ...
    's', [0; 5; 10], ...
    'ax', [0; 0; 0], ...
    'ay', [0; 0; 0]);

GraphPlotter.plotGeneralOverview(stateLog, 2, track);
assert(numel(findall(0, 'Type', 'figure')) == 1, ...
    'Duplicate-waypoint overview plot did not create one figure.');
end

function assertZeroDistanceOverviewPlot()
track = components.TestTrack('straight');
stateLog = struct( ...
    'speedKmh', [0; 0], ...
    'time', [0; 1], ...
    's', [0; 0], ...
    'ax', [0; 0], ...
    'ay', [0; 0]);

GraphPlotter.plotGeneralOverview(stateLog, 0, track);
assert(numel(findall(0, 'Type', 'figure')) == 1, ...
    'Zero-distance overview plot did not create one figure.');
end
