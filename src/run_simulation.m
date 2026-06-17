%% run_simulation.m - FSAE Transient Lap Time Simulation
% Entry point script that configures and runs the simulation
%
% Architecture:
%   - Components (aero, suspension, powertrain, tire) are swappable objects
%   - AeroManager aggregates multiple positioned AeroComponents (FW, RW, Floor)
%   - VehicleManager holds component references and vehicle parameters
%   - DriverModel decides throttle/brake inputs based on track lookahead
%   - Simulator runs the physics loop: state + inputs → next state
%   - Track provides geometry and surface properties

clear; clc; close all;

%% ====================================================================
%  SELECT TRACK TYPE
%  Options: 'straight10', 'straight', 'oval', 'skidpad', 'autocross', 'busstop', '90turn'
%  ====================================================================
trackType = 'straight10';

%% ====================================================================
%  DISPLAY OPTIONS
%  Set to true to show all graphs in a single window
%  ====================================================================
singleWindow = true;

% Export MoTeC CSV and .ld files after the simulation completes.
exportMoTeC = true;

fprintf('=== FSAE Transient Lap Time Simulation ===\n\n');

%% ====================================================================
%  CREATE AERODYNAMIC COMPONENTS
%  Each aero element is positioned independently and responds to
%  vehicle pitch and ride height from VehicleState
%  ====================================================================

% Front Wing: ahead of front axle, very pitch/height sensitive
frontWing = components.Aero.FrontWing( ...
    0.9, ...                   % xPosition: 0.9m forward of CG (ahead of front axle)
    0.08, ...                  % zPosition: 8cm above reference plane
    0.45, ...                   % ClA: Downforce coefficient * area
    0.35, ...                  % CdA: Drag coefficient * area
    -5.0, ...                  % pitchSensitivityClA: Loses DF when nose pitches up
    0.3 ...                    % heightSensitivity: Sensitive to ride height
);
fprintf('Aero: FrontWing  | x=%.2f m, ClA=%.2f, CdA=%.2f\n', ...
    frontWing.xPosition, frontWing.ClA, frontWing.CdA);

% Rear Wing: behind rear axle, moderate pitch sensitivity
rearWing = components.Aero.RearWing( ...
    -0.85, ...                 % xPosition: 0.85m behind CG (behind rear axle)
    0.45, ...                  % zPosition: 45cm above reference plane (high-mounted)
    0.55, ...                   % ClA: Highest DF element
    0.8, ...                  % CdA: Highest drag element
    3.0, ...                   % pitchSensitivityClA: Gains DF when nose pitches up
    0.15 ...                   % heightSensitivity: Moderately sensitive
);
fprintf('Aero: RearWing   | x=%.2f m, ClA=%.2f, CdA=%.2f\n', ...
    rearWing.xPosition, rearWing.ClA, rearWing.CdA);

% Underbody Floor / Diffuser: near CG, extremely height sensitive
floor = components.Aero.UnderbodyFloor( ...
    0.0, ...                   % xPosition: At CG
    0.035, ...                 % zPosition: 3.5cm (nominal floor height)
    0.4, ...                   % ClA: Moderate DF
    0.10, ...                  % CdA: Very low drag
    -8.0, ...                  % pitchSensitivityClA: Very pitch-sensitive (ground effect)
    0.015, ...                 % stallHeight: Stall below 1.5cm
    0.6 ...                    % heightExponent: Ground-effect sensitivity curve
);
fprintf('Aero: Floor      | x=%.2f m, ClA=%.2f, CdA=%.2f\n', ...
    floor.xPosition, floor.ClA, floor.CdA);

% AeroManager: aggregates all aero components
aero = components.Aero.AeroManager();
aero = aero.addComponent(frontWing);
aero = aero.addComponent(rearWing);
aero = aero.addComponent(floor);
fprintf('Aero: AeroManager with %d components\n', aero.numComponents());

% Print aero summary
totalClA = frontWing.ClA + rearWing.ClA + floor.ClA;
totalCdA = frontWing.CdA + rearWing.CdA + floor.CdA;
fprintf('  Total ClA=%.2f, Total CdA=%.2f\n', totalClA, totalCdA);

fprintf('\n');

%% ====================================================================
%  CREATE REMAINING COMPONENTS
%  ====================================================================

% --- Powertrain ---
powertrain = components.Powertrain.EMRAX228Powertrain();
fprintf('Powertrain: EMRAX 228 (Tq=%.0f Nm, FDR=%.1f, falloff %.0f->%.0f rpm, factor=%.2f)\n', ...
    powertrain.maxEngineTorque, powertrain.totalGearRatio, ...
    powertrain.rpmFalloffStartRPM, powertrain.rpmLimitRPM, ...
    powertrain.rpmFalloffFactor);

% --- Tires (Pacejka Magic Formula via MFeval) ---
% Requires MFeval toolbox: https://www.mathworks.com/matlabcentral/fileexchange/63618-mfeval
tire = components.Tire.PacejkaTire('43105_18x7.5_10_R25B_7.tir');

% --- Track ---
track = components.TestTrack(trackType);
fprintf('Track: TestTrack (''%s'', %.1f m, %d points)\n', ...
    trackType, track.getTotalLength(), size(track.getTrackPoints(), 1));

fprintf('\n');

%% ====================================================================
%  CREATE VEHICLE MANAGER, DRIVER MODEL, AND SIMULATOR
%  ====================================================================

% Simulation timestep [s]
dt = 0.001;

% VehicleManager is created first so SuspensionManager can reference it
vehicle = VehicleManager(aero, [], powertrain, tire, track);

% --- Suspension geometry ---
% Options: 'neutral', 'baseline', 'high-camber-gain', 'pro-ackermann'
geometryPreset = 'baseline';
geometry = components.Suspension.SuspensionGeometry.fromPreset(geometryPreset, vehicle);
fprintf('Suspension Geometry: %s (Ackermann %.0f%%)\n', ...
    geometryPreset, geometry.ackermann * 100);

% --- Suspension (needs vehicleManager for geometry) ---
suspension = components.Suspension.SuspensionManager( ...
    vehicle, ...                    % vehicleManager handle
    0.55, ...                       % frontRollStiffDist: 55% front
    45000, 3000, 4500, ...          % front: springRate, dampingCoeff, reboundCoeff
    42000, 2800, 4200, ...          % rear:  springRate, dampingCoeff, reboundCoeff
    0.95, ...                       % motionRatio
    0.025, ...                      % bumpStopLength [m]
    200000, ...                     % bumpStopRate [N/m]
    200000, ...                     % tireSpringRate [N/m]
    25, ...                         % unsprungMass per corner [kg]
    geometry ...                    % suspension/steering geometry preset
);
vehicle.suspension = suspension;
fprintf('Suspension: SuspensionManager (4-corner transient + geometry)\n');

% Warmup suspension to static equilibrium (prevents zero-state startup transient)
suspension.warmup(vehicle.totalMass, dt);

driver    = DriverModel(vehicle);
simulator = Simulator(vehicle, driver, dt);

initialState = VehicleState('s', 0, 'speed', 0.1);
[stateLog, lapTime] = simulator.simulate(initialState, track);

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
GraphPlotter.plotAll(stateLog, lapTime, track, vehicle, aero, singleWindow);

% --- Summary ---
speedKmh = stateLog.speedKmh;
axG = stateLog.ax / 9.81;
ayG = stateLog.ay / 9.81;
pitchDeg = stateLog.pitchAngle * (180/pi);

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
