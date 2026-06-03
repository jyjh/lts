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
%  Options: 'straight', 'oval', 'skidpad', 'autocross'
%  ====================================================================
trackType = 'straight';

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
    0.9, ...                   % ClA: Downforce coefficient * area
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
    1.1, ...                   % ClA: Highest DF element
    0.55, ...                  % CdA: Highest drag element
    3.0, ...                   % pitchSensitivityClA: Gains DF when nose pitches up
    0.15 ...                   % heightSensitivity: Moderately sensitive
);
fprintf('Aero: RearWing   | x=%.2f m, ClA=%.2f, CdA=%.2f\n', ...
    rearWing.xPosition, rearWing.ClA, rearWing.CdA);

% Underbody Floor / Diffuser: near CG, extremely height sensitive
floor = components.Aero.UnderbodyFloor( ...
    0.0, ...                   % xPosition: At CG
    0.035, ...                 % zPosition: 3.5cm (nominal floor height)
    0.8, ...                   % ClA: Moderate DF
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

% --- Suspension ---
suspension = components.Suspension.SimpleSuspension( ...
    1.2, ...                    % trackWidth [m]
    1.55, ...                   % wheelbase [m]
    0.28, ...                   % cgHeight [m]
    0.55, ...                   % rollStiffDist: 55% front roll stiffness
    0.48 ...                    % staticFrontWeight: 48% front static weight
);
fprintf('Suspension: SimpleSuspension (WB=%.2f m, CG=%.2f m)\n', ...
    suspension.wheelbase, suspension.cgHeight);

% --- Powertrain ---
powertrain = components.Powertrain.SimplePowertrain( ...
    55, ...                     % maxEngineTorque [Nm]
    12.0, ...                   % totalGearRatio: gear * final drive
    0.2286, ...                 % wheelRadius [m]
    0.90 ...                    % drivetrainEfficiency: 90% efficient
);
fprintf('Powertrain: SimplePowertrain (Tq=%.0f Nm, Ratio=%.1f)\n', ...
    powertrain.maxEngineTorque, powertrain.totalGearRatio);

% --- Tires ---
tire = components.Tire.SimpleTire( ...
    800, ...                    % corneringStiffness [N/deg]
    10000, ...                  % longitudinalStiffness [N/unit slip]
    1.8, ...                    % peakMuLat: Peak lateral friction
    -0.1 ...                    % loadSensitivityExp: Slight load sensitivity
);
fprintf('Tires: SimpleTire (Peak mu=%.1f)\n', tire.peakMuLat);

% --- Track ---
track = components.TestTrack(trackType);
fprintf('Track: TestTrack (''%s'', %.1f m, %d points)\n', ...
    trackType, track.getTotalLength(), size(track.getTrackPoints(), 1));

fprintf('\n');

%% ====================================================================
%  CREATE VEHICLE MANAGER, DRIVER MODEL, AND SIMULATOR
%  ====================================================================
vehicle = VehicleManager(aero, suspension, powertrain, tire, track, ...
    280, ...                    % totalMass: Car + driver [kg]
    0.001, ...                  % dt: Timestep [s]
    40 ...                      % maxSpeed: Speed limit [m/s] ~144 km/h
);

driver    = DriverModel(vehicle);
simulator = Simulator(vehicle, driver);

initialState = VehicleState('s', 0, 'speed', 0.1);
[stateLog, lapTime] = simulator.simulate(initialState, track);

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

% Convert to km/h for display
speedKmh = stateLog.speedKmh;
time = stateLog.time;
s = stateLog.s;

figure('Name', 'LTS Results', 'Position', [50 50 1400 900]);

% --- Speed vs Distance ---
subplot(2,4,1);
plot(s, speedKmh, 'b-', 'LineWidth', 1.5);
xlabel('Distance [m]');
ylabel('Speed [km/h]');
title('Speed vs Distance');
grid on;
xlim([0 max(s)]);

% --- Speed vs Time ---
subplot(2,4,2);
plot(time, speedKmh, 'r-', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Speed [km/h]');
title('Speed vs Time');
grid on;

% --- Longitudinal and Lateral Acceleration ---
subplot(2,4,3);
axG = stateLog.ax / 9.81;
ayG = stateLog.ay / 9.81;
plot(time, axG, 'b-', 'LineWidth', 1); hold on;
plot(time, ayG, 'r-', 'LineWidth', 1);
xlabel('Time [s]');
ylabel('Acceleration [g]');
title('Accelerations');
legend('a_x', 'a_y', 'Location', 'best');
grid on;

% --- Throttle and Brake ---
subplot(2,4,4);
plot(time, stateLog.throttle * 100, 'g-', 'LineWidth', 1); hold on;
plot(time, stateLog.brake * 100, 'r-', 'LineWidth', 1);
xlabel('Time [s]');
ylabel('Input [%]');
title('Driver Inputs');
legend('Throttle', 'Brake', 'Location', 'best');
grid on;
ylim([-5 105]);

% --- Forces ---
subplot(2,4,5);
plot(time, stateLog.F_downforce, 'b-', 'LineWidth', 1); hold on;
plot(time, stateLog.F_drag, 'r-', 'LineWidth', 1);
plot(time, stateLog.F_drive, 'g-', 'LineWidth', 1);
xlabel('Time [s]');
ylabel('Force [N]');
title('Forces');
legend('Downforce', 'Drag', 'Drive', 'Location', 'best');
grid on;

% --- Pitch Angle ---
subplot(2,4,6);
pitchDeg = stateLog.pitchAngle * (180/pi);
plot(time, pitchDeg, 'm-', 'LineWidth', 1);
xlabel('Time [s]');
ylabel('Pitch Angle [deg]');
title('Vehicle Pitch');
grid on;

% --- Track Map (colored by speed) ---
subplot(2,4,7);
trackPts = track.getTrackPoints();

% Interpolate track position from logged s
arcLen = [0; cumsum(sqrt(diff(trackPts(:,1)).^2 + diff(trackPts(:,2)).^2))];
xFit = interp1(arcLen, trackPts(:,1), stateLog.s, 'linear', 'extrap');
yFit = interp1(arcLen, trackPts(:,2), stateLog.s, 'linear', 'extrap');

scatter(xFit, yFit, 10, speedKmh, 'filled');
colorbar;
colormap('jet');
xlabel('X [m]');
ylabel('Y [m]');
title(sprintf('Track Map (Lap: %.2f s)', lapTime));
axis equal;
grid on;

% --- Aero axle loads vs speed ---
subplot(2,4,8);
sampleSpeeds = [10, 15, 20, 25, 30, 35];
sampleFf = zeros(1, numel(sampleSpeeds));
sampleFr = zeros(1, numel(sampleSpeeds));
sampleFd = zeros(1, numel(sampleSpeeds));
for j = 1:numel(sampleSpeeds)
    tempState = VehicleState('speed', sampleSpeeds(j), 'ax', 0, 'pitchAngle', 0, 'rideHeight', 0);
    tempState.vehicleManager = vehicle;
    af = aero.computeForces(tempState);
    sampleFf(j) = af.Fz_front;
    sampleFr(j) = af.Fz_rear;
    sampleFd(j) = af.F_drag;
end
bar(sampleSpeeds * 3.6, [sampleFf', sampleFr', sampleFd'], 'grouped');
xlabel('Speed [km/h]');
ylabel('Force [N]');
title('Aero Loads by Axle');
legend('Front axle', 'Rear axle', 'Drag', 'Location', 'northwest');
grid on;

% --- Summary ---
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
fprintf('Peak Pitch:     %.3f deg\n', max(abs(pitchDeg)));