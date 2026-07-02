%% powertrain_power_curve.m
% Motor power, torque, and tractive-force curves for the configured powertrain.
% Samples the EMRAX228Powertrain public API (getMaxTorque / computeMaxDriveForce)
% over a dense RPM grid so the plot matches what the simulator actually sees,
% including the constant-power field-weakening rolloff and the rev-limit cut.

clear; clc;

configName = 'baseline';   % Matches src/run_simulation.m by default
makePlots = true;
savePlots = true;

scriptDir = fileparts(mfilename('fullpath'));
repoRoot = fileparts(scriptDir);
addpath(fullfile(repoRoot, 'src'));

cfg = feval(['vehicles.' configName]);

powertrain = components.Powertrain.EMRAX228Powertrain( ...
    cfg.powertrain.matFile, ...
    cfg.powertrain.efficiency, ...
    fieldOr(cfg.powertrain, 'motorRotorInertia', 0.07));

% Single dense RPM grid spanning the full operating range up to the rev limit.
rpmGrid = linspace(0, powertrain.rpmLimitRPM, 500)';
T_motor = arrayfun(@(r) powertrain.getMaxTorque(r), rpmGrid);          % [Nm]
P_kW = T_motor .* rpmGrid * 2 * pi / 60 / 1000;                        % P = T * omega [kW]
speedMs = rpmGrid / powertrain.totalGearRatio * ...
    2 * pi * powertrain.wheelRadius / 60;                              % [m/s]
F_drive = arrayfun(@(v) powertrain.computeMaxDriveForce(v), speedMs);  % [N]
wheelTorque = F_drive * powertrain.wheelRadius;                        % [Nm]

[peakTorque, peakTorqueIdx] = max(T_motor);
[peakPowerKW, peakPowerIdx] = max(P_kW);
[peakForce, peakForceIdx] = max(F_drive);
vMapEnd = powertrain.rpmFalloffStartRPM / powertrain.totalGearRatio * ...
    2 * pi * powertrain.wheelRadius / 60;
vRevLimit = powertrain.rpmLimitRPM / powertrain.totalGearRatio * ...
    2 * pi * powertrain.wheelRadius / 60;

fprintf('\n=== Powertrain Power & Torque Curves ===\n');
fprintf('Config:            vehicles.%s\n', configName);
fprintf('Motor map:         %s\n', powertrain.matFilePath);
fprintf('Efficiency:        %.3f\n', powertrain.drivetrainEfficiency);
fprintf('Final drive ratio: %.2f\n', powertrain.totalGearRatio);
fprintf('Wheel radius:      %.3f m\n', powertrain.wheelRadius);
fprintf('\n');
fprintf('Peak motor torque: %.1f Nm at %d rpm\n', ...
    peakTorque, round(rpmGrid(peakTorqueIdx)));
fprintf('Peak motor power:  %.2f kW (%.1f hp) at %d rpm\n', ...
    peakPowerKW, peakPowerKW / 0.7457, round(rpmGrid(peakPowerIdx)));
fprintf('Map end:           %d rpm (%.1f km/h)\n', ...
    round(powertrain.rpmFalloffStartRPM), vMapEnd * 3.6);
fprintf('Rev limit:         %d rpm (%.1f km/h)\n', ...
    round(powertrain.rpmLimitRPM), vRevLimit * 3.6);
fprintf('Peak tractive force: %.0f N at %.1f km/h\n', ...
    peakForce, speedMs(peakForceIdx) * 3.6);
fprintf('Max mapped vehicle speed: %.1f km/h\n', powertrain.maxVehicleSpeed * 3.6);

if makePlots
    plotPowertrainCurves(rpmGrid, T_motor, P_kW, speedMs, F_drive, wheelTorque, ...
        powertrain, vMapEnd, vRevLimit, scriptDir, savePlots);
end

function plotPowertrainCurves(rpmGrid, T_motor, P_kW, speedMs, F_drive, ...
        wheelTorque, powertrain, vMapEnd, vRevLimit, scriptDir, savePlots)
    speedKmh = speedMs * 3.6;

    fig = figure('Name', 'Powertrain power & torque curves', 'Color', 'w');
    fig.Position = [100 100 1200 950];
    set(fig, ...
        'DefaultTextColor', 'k', ...
        'DefaultAxesColor', 'w', ...
        'DefaultAxesXColor', 'k', ...
        'DefaultAxesYColor', 'k', ...
        'DefaultLegendColor', 'w', ...
        'DefaultLegendTextColor', 'k');
    tiledlayout(fig, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    % Tile 1: Motor torque vs RPM
    nexttile;
    hTorque = plot(rpmGrid, T_motor, 'LineWidth', 1.6);
    hold on;
    hMapEnd = xline(powertrain.rpmFalloffStartRPM, ':', 'Map end', 'LineWidth', 1.0);
    hRevLimit = xline(powertrain.rpmLimitRPM, '--', 'Rev limit', 'LineWidth', 1.0);
    hold off;
    styleAxes(gca);
    grid on;
    xlabel('Motor speed [rpm]');
    ylabel('Motor torque [Nm]');
    title('Motor torque vs speed');
    lgd = legend([hTorque hMapEnd hRevLimit], ...
        {'Torque', 'Map end', 'Rev limit'}, 'Location', 'best');
    styleLegend(lgd);

    % Tile 2: Motor power vs RPM
    nexttile;
    hPower = plot(rpmGrid, P_kW, 'LineWidth', 1.6);
    hold on;
    hMapEndP = xline(powertrain.rpmFalloffStartRPM, ':', 'Map end', 'LineWidth', 1.0);
    hRevLimitP = xline(powertrain.rpmLimitRPM, '--', 'Rev limit', 'LineWidth', 1.0);
    hold off;
    styleAxes(gca);
    grid on;
    xlabel('Motor speed [rpm]');
    ylabel('Motor power [kW]');
    title('Motor power vs speed');
    lgd = legend([hPower hMapEndP hRevLimitP], ...
        {'Power', 'Map end', 'Rev limit'}, 'Location', 'best');
    styleLegend(lgd);

    % Tile 3: Tractive force vs vehicle speed
    nexttile;
    hForce = plot(speedKmh, F_drive / 1000, 'LineWidth', 1.6);
    hold on;
    hMapEndF = xline(vMapEnd * 3.6, ':', 'Map end', 'LineWidth', 1.0);
    hRevLimitF = xline(vRevLimit * 3.6, '--', 'Rev limit', 'LineWidth', 1.0);
    hold off;
    styleAxes(gca);
    grid on;
    xlabel('Vehicle speed [km/h]');
    ylabel('Tractive force [kN]');
    title('Tractive force vs vehicle speed');
    lgd = legend([hForce hMapEndF hRevLimitF], ...
        {'Tractive force', 'Map end', 'Rev limit'}, 'Location', 'best');
    styleLegend(lgd);

    % Tile 4: Wheel torque vs vehicle speed
    nexttile;
    hWheelT = plot(speedKmh, wheelTorque, 'LineWidth', 1.6);
    hold on;
    hMapEndW = xline(vMapEnd * 3.6, ':', 'Map end', 'LineWidth', 1.0);
    hRevLimitW = xline(vRevLimit * 3.6, '--', 'Rev limit', 'LineWidth', 1.0);
    hold off;
    styleAxes(gca);
    grid on;
    xlabel('Vehicle speed [km/h]');
    ylabel('Wheel torque [Nm]');
    title('Wheel torque vs vehicle speed');
    lgd = legend([hWheelT hMapEndW hRevLimitW], ...
        {'Wheel torque', 'Map end', 'Rev limit'}, 'Location', 'best');
    styleLegend(lgd);

    if savePlots
        exportDir = fullfile(fileparts(scriptDir), 'exports');
        if ~exist(exportDir, 'dir')
            mkdir(exportDir);
        end
        outFile = fullfile(exportDir, 'powertrain_power_curve.png');
        saveas(fig, outFile);
        fprintf('Saved powertrain plot: %s\n', outFile);
    end
end

function styleAxes(ax)
    ax.Color = 'w';
    ax.XColor = 'k';
    ax.YColor = 'k';
    ax.Title.Color = 'k';
    ax.XLabel.Color = 'k';
    ax.YLabel.Color = 'k';
    ax.GridColor = [0.65 0.65 0.65];
    ax.MinorGridColor = [0.8 0.8 0.8];
    ax.LineWidth = 0.8;
end

function styleLegend(lgd)
    lgd.Color = 'w';
    lgd.TextColor = 'k';
    lgd.EdgeColor = [0.35 0.35 0.35];
end

function value = fieldOr(s, name, fallback)
    if isfield(s, name)
        value = s.(name);
    else
        value = fallback;
    end
end
