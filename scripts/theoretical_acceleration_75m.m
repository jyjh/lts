%% theoretical_acceleration_75m.m
% Best-case 75 m acceleration estimate for the current vehicle/tire config.
% Assumes straight-line RWD launch, zero steering, nominal aero attitude, and
% full-throttle force capped by rear-axle tire grip from the active .tir file.

clear; clc;

configName = 'baseline';     % Matches src/run_simulation.m by default
distanceM = 75.0;            % FSAE acceleration distance [m]
dsM = 0.05;                  % Integration distance step [m]
surfaceMu = [];              % [] = raw tire-file reference surface
usePowertrainLimit = true;   % false = ideal tire-only acceleration envelope
includeRotatingInertia = false;
makePlots = true;
savePlots = true;

scriptDir = fileparts(mfilename('fullpath'));
repoRoot = fileparts(scriptDir);
addpath(fullfile(repoRoot, 'src'));

cfg = feval(['vehicles.' configName]);
tire = components.Tire.PacejkaTire(cfg.tire.tirFile);
tire.wheelInertia = cfg.tire.wheelInertia;
if isfield(cfg.tire, 'relaxationLength')
    tire.relaxationLength = cfg.tire.relaxationLength;
end
if isfield(cfg.tire, 'surfaceMuReference')
    tire.surfaceMuReference = cfg.tire.surfaceMuReference;
end
if isempty(surfaceMu)
    surfaceMu = tire.surfaceMuReference;
end

powertrain = components.Powertrain.EMRAX228Powertrain( ...
    cfg.powertrain.matFile, ...
    cfg.powertrain.efficiency, ...
    fieldOr(cfg.powertrain, 'motorRotorInertia', 0.07));

massForAccel = effectiveMass(cfg, powertrain, includeRotatingInertia);

[launchAx, launch] = maxAccelAtSpeed( ...
    0, cfg, tire, powertrain, massForAccel, surfaceMu, usePowertrainLimit);
[launchTireOnlyAx, ~] = maxAccelAtSpeed( ...
    0, cfg, tire, powertrain, massForAccel, surfaceMu, false);

run = integrateAccelerationTest( ...
    distanceM, dsM, cfg, tire, powertrain, massForAccel, surfaceMu, ...
    usePowertrainLimit);

[peakAx, peakIdx] = max(run.ax);
avgAx = run.finalSpeed^2 / (2 * distanceM);
firstPowerIdx = find(run.Fmotor < run.FtractionRear - 1e-6, 1, 'first');

fprintf('\n=== Theoretical 75 m Acceleration Estimate ===\n');
fprintf('Config:              vehicles.%s\n', configName);
fprintf('Tire file:           %s\n', cfg.tire.tirFile);
fprintf('Surface mu input:    %.3f\n', surfaceMu);
fprintf('Powertrain limit:    %s\n', onOff(usePowertrainLimit));
fprintf('Rotating inertia:    %s, effective mass %.1f kg (vehicle %.1f kg)\n', ...
    onOff(includeRotatingInertia), massForAccel, cfg.totalMass);
fprintf('\n');
fprintf('Rear static tire peak mu_x: %.3f at kappa %.3f\n', ...
    launch.staticRearMu, launch.staticRearKappa);
fprintf('Tire-only launch cap:       %.2f m/s^2 (%.2f g)\n', ...
    launchTireOnlyAx, launchTireOnlyAx / VehicleManager.g);
fprintf('Launch acceleration used:   %.2f m/s^2 (%.2f g)\n', ...
    launchAx, launchAx / VehicleManager.g);
fprintf('Peak acceleration over 75m: %.2f m/s^2 (%.2f g) at %.1f m, %.1f km/h\n', ...
    peakAx, peakAx / VehicleManager.g, run.s(peakIdx), run.v(peakIdx) * 3.6);
if isempty(firstPowerIdx)
    fprintf('Limiter transition:          never power-limited over %.1f m\n', distanceM);
else
    fprintf('Limiter transition:          power-limited from %.1f m, %.1f km/h\n', ...
        run.s(firstPowerIdx), run.v(firstPowerIdx) * 3.6);
end
fprintf('\n');
fprintf('Theoretical minimum 75 m time: %.3f s\n', run.time);
fprintf('Trap speed:                   %.1f km/h\n', run.finalSpeed * 3.6);
fprintf('Equivalent average accel:      %.2f m/s^2 (%.2f g)\n', ...
    avgAx, avgAx / VehicleManager.g);

if makePlots
    plotAccelerationDiagnostics(run, cfg, powertrain, distanceM, scriptDir, savePlots);
end

function result = integrateAccelerationTest(distanceM, dsM, cfg, tire, ...
        powertrain, massForAccel, surfaceMu, usePowertrainLimit)
    nMax = ceil(distanceM / dsM) + 2;
    sLog = zeros(nMax, 1);
    vLog = zeros(nMax, 1);
    axLog = zeros(nMax, 1);
    rearNormalLog = zeros(nMax, 1);
    rearMuLog = zeros(nMax, 1);
    rearKappaLog = zeros(nMax, 1);
    FtractionRearLog = zeros(nMax, 1);
    FmotorLog = zeros(nMax, 1);
    FdriveLog = zeros(nMax, 1);
    FdragLog = zeros(nMax, 1);
    FrollLog = zeros(nMax, 1);
    motorRpmLog = zeros(nMax, 1);

    s = 0;
    v = 0;
    t = 0;
    i = 0;
    while s < distanceM - 1e-9
        ds = min(dsM, distanceM - s);
        [ax, detail] = maxAccelAtSpeed( ...
            v, cfg, tire, powertrain, massForAccel, surfaceMu, ...
            usePowertrainLimit);
        if v <= 1e-9 && ax <= 1e-9
            error('Acceleration estimate cannot launch from %.2f m.', s);
        end

        vNextSq = v^2 + 2 * ax * ds;
        if vNextSq <= 0
            error('Acceleration estimate stalled at %.2f m and %.2f km/h.', ...
                s, v * 3.6);
        end

        i = i + 1;
        sLog(i) = s;
        vLog(i) = v;
        axLog(i) = ax;
        rearNormalLog(i) = detail.rearNormal;
        rearMuLog(i) = detail.rearMu;
        rearKappaLog(i) = detail.rearKappa;
        FtractionRearLog(i) = detail.FtractionRear;
        FmotorLog(i) = detail.Fmotor;
        FdriveLog(i) = detail.Fdrive;
        FdragLog(i) = detail.Fdrag;
        FrollLog(i) = detail.Froll;
        motorRpmLog(i) = detail.motorRPM;

        vNext = sqrt(vNextSq);
        if abs(ax) < 1e-9
            t = t + ds / max(v, eps);
        else
            t = t + (vNext - v) / ax;
        end
        s = s + ds;
        v = vNext;
    end

    result.s = sLog(1:i);
    result.v = vLog(1:i);
    result.ax = axLog(1:i);
    result.rearNormal = rearNormalLog(1:i);
    result.rearMu = rearMuLog(1:i);
    result.rearKappa = rearKappaLog(1:i);
    result.FtractionRear = FtractionRearLog(1:i);
    result.Fmotor = FmotorLog(1:i);
    result.Fdrive = FdriveLog(1:i);
    result.Fdrag = FdragLog(1:i);
    result.Froll = FrollLog(1:i);
    result.motorRPM = motorRpmLog(1:i);
    result.powerLimited = result.Fmotor < result.FtractionRear - 1e-6;
    result.gripLimited = result.FtractionRear <= result.Fmotor + 1e-6;
    result.time = t;
    result.finalSpeed = v;
end

function [ax, detail] = maxAccelAtSpeed(v, cfg, tire, powertrain, ...
        massForAccel, surfaceMu, usePowertrainLimit)
    g = VehicleManager.g;
    W = cfg.totalMass * g;
    crr = fieldOr(cfg.tire, 'rollingResistanceCoeff', 0);
    ax = 0;

    for iter = 1:12 %#ok<NASGU>
        [Fdrag, FzAeroFront, FzAeroRear] = aeroForces(cfg, v);
        totalNormal = W + FzAeroFront + FzAeroRear;
        Froll = crr * totalNormal;

        rearNormal = W * (1 - cfg.staticFrontWeight) + FzAeroRear + ...
            cfg.totalMass * ax * cfg.cgHeight / cfg.wheelbase;
        rearNormal = max(rearNormal, 0);
        rearPerTire = rearNormal / 2;

        [muRear, kappaRear] = peakLongitudinalMu(tire, rearPerTire, v, surfaceMu);
        FtractionRear = muRear * rearNormal;

        if usePowertrainLimit
            Fmotor = max(0, powertrain.computeMaxDriveForce(v));
        else
            Fmotor = Inf;
        end

        Fdrive = min(Fmotor, FtractionRear);
        axNew = (Fdrive - Fdrag - Froll) / massForAccel;
        if abs(axNew - ax) < 1e-5
            ax = axNew;
            break;
        end
        ax = 0.6 * ax + 0.4 * axNew;
    end

    staticRearPerTire = W * (1 - cfg.staticFrontWeight) / 2;
    [staticRearMu, staticRearKappa] = peakLongitudinalMu( ...
        tire, staticRearPerTire, v, surfaceMu);

    detail = struct( ...
        'rearNormal', rearNormal, ...
        'rearMu', muRear, ...
        'rearKappa', kappaRear, ...
        'staticRearMu', staticRearMu, ...
        'staticRearKappa', staticRearKappa, ...
        'FtractionRear', FtractionRear, ...
        'Fmotor', Fmotor, ...
        'Fdrive', Fdrive, ...
        'Fdrag', Fdrag, ...
        'Froll', Froll, ...
        'motorRPM', motorRpmAtSpeed(v, powertrain));
end

function plotAccelerationDiagnostics(run, cfg, powertrain, distanceM, scriptDir, savePlots)
    s = run.s(:);
    speedKmh = run.v(:) * 3.6;
    resistance = run.Fdrag(:) + run.Froll(:);
    forceMargin = (run.Fmotor(:) - run.FtractionRear(:)) / 1000;
    firstPowerIdx = find(run.powerLimited, 1, 'first');

    fig = figure('Name', '75 m acceleration diagnostics', 'Color', 'w');
    fig.Position = [100 100 1200 950];
    set(fig, ...
        'DefaultTextColor', 'k', ...
        'DefaultAxesColor', 'w', ...
        'DefaultAxesXColor', 'k', ...
        'DefaultAxesYColor', 'k', ...
        'DefaultLegendColor', 'w', ...
        'DefaultLegendTextColor', 'k');
    tiledlayout(fig, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    hTire = plot(s, run.FtractionRear / 1000, 'LineWidth', 1.6);
    hold on;
    hMotor = plot(s, run.Fmotor / 1000, 'LineWidth', 1.6);
    hDrive = plot(s, run.Fdrive / 1000, 'k--', 'LineWidth', 1.2);
    hResistance = plot(s, resistance / 1000, ':', 'LineWidth', 1.2);
    addTransitionLine(firstPowerIdx, run);
    hold off;
    styleAxes(gca);
    grid on;
    ylabel('Force [kN]');
    title('Rear tire grip cap vs motor force');
    lgd = legend([hTire hMotor hDrive hResistance], ...
        {'Rear tire cap', 'Motor available', 'Used drive force', ...
        'Drag + rolling resistance'}, 'Location', 'best');
    styleLegend(lgd);
    xlim([0 distanceM]);

    nexttile;
    yyaxis left;
    hRearLoad = plot(s, run.rearNormal / 1000, 'LineWidth', 1.5);
    ylabel('Rear normal load [kN]');
    yyaxis right;
    hRearMu = plot(s, run.rearMu, 'LineWidth', 1.5);
    hold on;
    hRearKappa = plot(s, run.rearKappa, '--', 'LineWidth', 1.2);
    hold off;
    ylabel('Peak \mu_x / kappa');
    styleAxes(gca);
    grid on;
    title('Rear tire operating envelope');
    lgd = legend([hRearLoad hRearMu hRearKappa], ...
        {'Rear normal load', 'Peak \mu_x', 'Peak kappa'}, 'Location', 'best');
    styleLegend(lgd);
    xlim([0 distanceM]);

    nexttile;
    yyaxis left;
    hRpm = plot(s, run.motorRPM, 'LineWidth', 1.5);
    hold on;
    hMap = yline(powertrain.rpmFalloffStartRPM, ':', 'Map end', 'LineWidth', 1.0);
    hLimit = yline(powertrain.rpmLimitRPM, '--', 'RPM limit', 'LineWidth', 1.0);
    hold off;
    ylabel('Motor speed [rpm]');
    yyaxis right;
    hSpeed = plot(s, speedKmh, '--', 'LineWidth', 1.3);
    ylabel('Speed [km/h]');
    styleAxes(gca);
    grid on;
    title('Motor speed and vehicle speed');
    lgd = legend([hRpm hMap hLimit hSpeed], ...
        {'Motor RPM', 'Map end', 'RPM limit', 'Speed'}, 'Location', 'best');
    styleLegend(lgd);
    xlim([0 distanceM]);

    nexttile;
    yyaxis left;
    hAccel = plot(s, run.ax / VehicleManager.g, 'LineWidth', 1.5);
    ylabel('Longitudinal accel [g]');
    yyaxis right;
    hMargin = plot(s, forceMargin, 'LineWidth', 1.4);
    hold on;
    hZero = yline(0, 'k--', 'LineWidth', 1.0);
    hold off;
    ylabel('Motor minus tire cap [kN]');
    xlabel('Distance [m]');
    styleAxes(gca);
    grid on;
    title('Acceleration and limiting margin');
    lgd = legend([hAccel hMargin hZero], ...
        {'Acceleration', '+ grip-limited / - power-limited', 'Transition'}, ...
        'Location', 'best');
    styleLegend(lgd);
    xlim([0 distanceM]);

    if savePlots
        exportDir = fullfile(fileparts(scriptDir), 'exports');
        if ~exist(exportDir, 'dir')
            mkdir(exportDir);
        end
        outFile = fullfile(exportDir, 'theoretical_acceleration_75m_diagnostics.png');
        saveas(fig, outFile);
        fprintf('Saved diagnostic plot: %s\n', outFile);
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

function addTransitionLine(firstPowerIdx, run)
    if isempty(firstPowerIdx)
        return;
    end
    x = run.s(firstPowerIdx);
    yl = ylim;
    plot([x x], yl, 'Color', [0.45 0.45 0.45], ...
        'LineStyle', '-.', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    ylim(yl);
end

function [muPeak, kappaPeak] = peakLongitudinalMu(tire, Fz, speed, surfaceMu)
    if Fz <= 0
        muPeak = 0;
        kappaPeak = 0;
        return;
    end

    persistent cache
    if isempty(cache)
        cache = containers.Map('KeyType', 'char', 'ValueType', 'any');
    end

    Vx = mfevalSpeed(tire, speed);
    FzKey = round(Fz / 10) * 10;
    VxKey = round(Vx * 10) / 10;
    muKey = round(surfaceMu * 1000) / 1000;
    key = sprintf('%.0f_%.1f_%.3f', FzKey, VxKey, muKey);
    if isKey(cache, key)
        value = cache(key);
        muPeak = value(1);
        kappaPeak = value(2);
        return;
    end

    kappa = linspace(0, 1.0, 161)';
    n = numel(kappa);
    inputsMF = [ ...
        repmat(Fz, n, 1), ...
        kappa, ...
        zeros(n, 1), ...
        zeros(n, 1), ...
        zeros(n, 1), ...
        repmat(Vx, n, 1), ...
        repmat(tire.tireConstants.nomPressure, n, 1)];
    outputs = mfeval(tire.tireConstants.params, inputsMF, 111);

    surfaceScale = max(surfaceMu, 0) / max(tire.surfaceMuReference, eps);
    Fx = outputs(:, 1) * surfaceScale;
    [FxPeak, idx] = max(Fx);
    muPeak = max(0, FxPeak / Fz);
    kappaPeak = kappa(idx);
    cache(key) = [muPeak, kappaPeak];
end

function [Fdrag, FzFront, FzRear] = aeroForces(cfg, speed)
    q = 0.5 * cfg.airDensity * speed^2;
    p = cfg.aero;
    rearArm = cfg.wheelbase * cfg.staticFrontWeight;

    downforce = q * p.ClA;
    Fdrag = q * p.CdA;
    frontFrac = (rearArm + p.xPosition) / cfg.wheelbase;
    frontFrac = min(1, max(0, frontFrac));
    FzFront = downforce * frontFrac;
    FzRear = downforce * (1 - frontFrac);
end

function Vx = mfevalSpeed(tire, speed)
    lowSpeedLimit = 0.1;
    if isfield(tire.tireConstants.params, 'VXLOW')
        lowSpeedLimit = max(lowSpeedLimit, tire.tireConstants.params.VXLOW);
    end
    lowSpeedLimit = lowSpeedLimit + max(1e-3, 1e-6 * lowSpeedLimit);
    Vx = max(abs(speed), lowSpeedLimit);
end

function rpm = motorRpmAtSpeed(speed, powertrain)
    rpm = max(speed, 0) ./ (2 * pi * max(powertrain.wheelRadius, eps)) * ...
        60 * powertrain.totalGearRatio;
end

function mEff = effectiveMass(cfg, powertrain, includeRotatingInertia)
    mEff = cfg.totalMass;
    if ~includeRotatingInertia
        return;
    end

    R = max(cfg.tire.wheelRadius, eps);
    baseI = cfg.tire.wheelInertia;
    rearI = baseI;
    if ismethod(powertrain, 'getReflectedRotorInertia')
        rearI = baseI + 0.5 * powertrain.getReflectedRotorInertia();
    end
    mEff = mEff + (2 * baseI + 2 * rearI) / R^2;
end

function value = fieldOr(s, name, fallback)
    if isfield(s, name)
        value = s.(name);
    else
        value = fallback;
    end
end

function text = onOff(tf)
    if tf
        text = 'on';
    else
        text = 'off';
    end
end
