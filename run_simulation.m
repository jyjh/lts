%% run_simulation.m - FSAE Transient Lap Time Simulation
% Entry point script that configures and runs the simulation
%
% Architecture:
%   - Components (aero, suspension, powertrain, tire) are swappable objects
%   - AeroManager aggregates multiple positioned AeroComponents (FW, RW, Floor)
%   - VehicleManager composes them and runs the simulation loop
%   - Track provides geometry and surface properties

clear; clc; close all;

%% ====================================================================
%  SELECT TRACK TYPE
%  Options: 'straight', 'oval', 'skidpad', 'autocross'
%  ====================================================================
trackType = 'oval';

fprintf('=== FSAE Transient Lap Time Simulation ===\n\n');

%% ====================================================================
%  CREATE AERODYNAMIC COMPONENTS
%  Each aero element is positioned independently and responds to
%  vehicle pitch and ride height from VehicleState
%  ====================================================================

% Front Wing: ahead of front axle, very pitch/height sensitive
frontWing = components.FrontWing( ...
    'name',       'Front Wing', ...
    'xPosition',  0.9, ...     % 0.9m forward of CG (ahead of front axle)
    'zPosition',  0.08, ...    % 8cm above reference plane
    'ClA',        0.9, ...     % Downforce coefficient * area
    'CdA',        0.35, ...    % Drag coefficient * area
    'pitchSensitivityClA', -5.0, ...   % Loses DF when nose pitches up
    'heightSensitivity',    0.3, ...   % Sensitive to ride height
    'referenceHeight',      0.04 ...   % Design ride height 4cm
);
fprintf('Aero: FrontWing  | x=%.2f m, ClA=%.2f, CdA=%.2f\n', ...
    frontWing.xPosition, frontWing.ClA, frontWing.CdA);

% Rear Wing: behind rear axle, moderate pitch sensitivity
rearWing = components.RearWing( ...
    'name',       'Rear Wing', ...
    'xPosition',  -0.85, ...   % 0.85m behind CG (behind rear axle)
    'zPosition',  0.45, ...    % 45cm above reference plane (high-mounted)
    'ClA',        1.1, ...     % Highest DF element
    'CdA',        0.55, ...    % Highest drag element
    'pitchSensitivityClA', 3.0, ...    % Gains DF when nose pitches up
    'heightSensitivity',   0.15, ...   % Moderately sensitive
    'referenceHeight',     0.30 ...    % Design ride height
);
fprintf('Aero: RearWing   | x=%.2f m, ClA=%.2f, CdA=%.2f\n', ...
    rearWing.xPosition, rearWing.ClA, rearWing.CdA);

% Underbody Floor / Diffuser: near CG, extremely height sensitive
floor = components.UnderbodyFloor( ...
    'name',       'Underbody Floor', ...
    'xPosition',  0.0, ...     % At CG
    'zPosition',  0.035, ...   % 3.5cm (nominal floor height)
    'ClA',        0.8, ...     % Moderate DF
    'CdA',        0.10, ...    % Very low drag
    'pitchSensitivityClA', -8.0, ...   % Very pitch-sensitive (ground effect)
    'referenceHeight',      0.035, ... % Design ride height
    'stallHeight',          0.015 ...  % Stall below 1.5cm
);
fprintf('Aero: Floor      | x=%.2f m, ClA=%.2f, CdA=%.2f\n', ...
    floor.xPosition, floor.ClA, floor.CdA);

% AeroManager: aggregates all aero components
aero = components.AeroManager(1.55, 0.002);  % wheelbase, pitchStiffness [rad/g]
aero = aero.addComponent(frontWing);
aero = aero.addComponent(rearWing);
aero = aero.addComponent(floor);
fprintf('Aero: AeroManager with %d components (pitch stiffness=%.3f rad/g)\n', ...
    aero.numComponents(), aero.pitchStiffness);

% Print aero summary
totalClA = frontWing.ClA + rearWing.ClA + floor.ClA;
totalCdA = frontWing.CdA + rearWing.CdA + floor.CdA;
fprintf('  Total ClA=%.2f, Total CdA=%.2f\n', totalClA, totalCdA);

fprintf('\n');

%% ====================================================================
%  CREATE REMAINING COMPONENTS
%  ====================================================================

% --- Suspension ---
suspension = components.SimpleSuspension( ...
    'trackWidth', 1.2, ...       % [m]
    'wheelbase', 1.55, ...       % [m]
    'cgHeight', 0.28, ...        % [m]
    'rollStiffDist', 0.55, ...   % 55% front roll stiffness
    'staticFrontWeight', 0.48    % 48% front static weight
);
fprintf('Suspension: SimpleSuspension (WB=%.2f m, CG=%.2f m)\n', ...
    suspension.wheelbase, suspension.cgHeight);

% --- Powertrain ---
powertrain = components.SimplePowertrain( ...
    'maxEngineTorque', 55, ...     % [Nm]
    'totalGearRatio', 12.0, ...    % gear * final drive
    'wheelRadius', 0.2286, ...     % [m]
    'drivetrainEfficiency', 0.90   % 90% efficient
);
fprintf('Powertrain: SimplePowertrain (Tq=%.0f Nm, Ratio=%.1f)\n', ...
    powertrain.maxEngineTorque, powertrain.totalGearRatio);

% --- Tires ---
tire = components.SimpleTire( ...
    'corneringStiffness', 800, ...    % [N/deg]
    'longitudinalStiffness', 10000, ...% [N/unit slip]
    'peakMuLat', 1.8, ...             % Peak lateral friction
    'loadSensitivityExp', -0.1         % Slight load sensitivity
);
fprintf('Tires: SimpleTire (Peak mu=%.1f)\n', tire.peakMuLat);

% --- Track ---
track = components.TestTrack(trackType);
fprintf('Track: TestTrack (''%s'', %.1f m, %d points)\n', ...
    trackType, track.getTotalLength(), size(track.getTrackPoints(), 1));

fprintf('\n');

%% ====================================================================
%  CREATE VEHICLE MANAGER AND RUN SIMULATION
%  ====================================================================
vehicle = VehicleManager(aero, suspension, powertrain, tire, track, ...
    'totalMass', 280, ...     % Car + driver [kg]
    'dt', 0.001, ...          % Timestep [s]
    'maxSpeed', 40 ...        % Speed limit [m/s] ~144 km/h
);

[stateLog, lapTime] = vehicle.simulate();

%% ====================================================================
%  COMPUTE PER-COMPONENT AERO AT FINAL STATE (for reporting)
%  ====================================================================
perComp = aero.computePerComponent(vehicle.state);
fprintf('\n=== Aero Component Breakdown (at final speed) ===\n');
for i = 1:numel(perComp)
    fprintf('  %-16s | DF=%7.1f N | Drag=%6.1f N | x=%.2f m\n', ...
        perComp(i).name, perComp(i).downforce, perComp(i).drag, perComp(i).xPosition);
end

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

% --- Per-component aero breakdown bar chart ---
subplot(2,4,8);
% Sample aero at several speeds for breakdown
sampleSpeeds = [10, 15, 20, 25, 30, 35];
sampleDF = zeros(3, numel(sampleSpeeds));  % 3 components
for j = 1:numel(sampleSpeeds)
    tempState = VehicleState('speed', sampleSpeeds(j), 'ax', 0, 'pitchAngle', 0, 'rideHeight', 0);
    pc = aero.computePerComponent(tempState);
    for k = 1:min(3, numel(pc))
        sampleDF(k, j) = pc(k).downforce;
    end
end
bar(sampleSpeeds * 3.6, sampleDF', 'stacked');
xlabel('Speed [km/h]');
ylabel('Downforce [N]');
title('Aero Breakdown by Component');
legend({perComp(1).name, perComp(2).name, perComp(3).name}, 'Location', 'northwest');
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