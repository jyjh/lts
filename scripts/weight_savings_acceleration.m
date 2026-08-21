function result = weight_savings_acceleration(tirFile, varargin)
%WEIGHT_SAVINGS_ACCELERATION Find the point of diminishing returns for weight
%savings in terms of grip-limited FSAE 75 m acceleration time.
%
%   result = weight_savings_acceleration()
%   result = weight_savings_acceleration('43105_18x7.5_10_R25B_7.tir')
%   result = weight_savings_acceleration(tirFile, 'MassRangeKg', [180 340], ...
%       'CgHeight', 0.256, 'StaticFrontWeight', 0.5095, 'Wheelbase', 1.528)
%
%   Sweeps vehicle mass and, at each mass, solves for the grip-limited launch
%   acceleration of a straight-line RWD car using a bicycle longitudinal
%   load-transfer model fed by the Pacejka tire's peak longitudinal-force
%   envelope. Aero drag and downforce are neglected (a clean traction study);
%   the powertrain cap is optional (default off = pure grip-limited). The 75 m
%   time follows in closed form, T = sqrt(2*d/ax), because ax is speed-
%   independent once aero is off. The diminishing-returns mass is found with
%   the Kneedle elbow on the time-vs-mass curve and reported with the marginal
%   gain [ms per kg saved].
%
%   Defaults target the theoretical tire and R25 measured geometry so the
%   default call returns a meaningful answer; every parameter is overridable.
%   Shared sweep/plot/option machinery lives in the +wsc package next to this
%   script and is reused by weight_savings_skidpad and tire_sensitivity.

defaultTirFile = '43105_18x7.5_10_R25B_7_theoretical.tir';
optionNames = {'MassRangeKg', 'MassStepKg', 'EnvelopeSlipRatios', ...
    'EnvelopeLoadsN', 'Wheelbase', 'CgHeight', ...
    'StaticFrontWeight', 'DistanceM', 'ReferenceMassKg', ...
    'CgDropPerKgSavedM', 'CoupledCg', 'UsePowertrainLimit', ...
    'PowertrainMatFile', 'PowertrainEfficiency', 'MotorRotorInertia', ...
    'OutputFile', 'SavePlot', 'ShowFigure', 'CloseFigure'};
[tirFile, varargin] = wsc.resolveTireArg(tirFile, defaultTirFile, ...
    optionNames, varargin{:});
wsc.addScriptPaths();

defaultOutput = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
    'exports', 'weight_savings_acceleration.png');

parser = inputParser;
parser.FunctionName = 'weight_savings_acceleration';
parser.addParameter('MassRangeKg', [180 280], ...
    @(x) wsc.validatePositiveVector(x, 'MassRangeKg'));
parser.addParameter('MassStepKg', 0.5, @wsc.validatePositiveScalar);
parser.addParameter('EnvelopeSlipRatios', linspace(0, 1.0, 161), ...
    @(x) wsc.validateNonnegativeVector(x, 'EnvelopeSlipRatios'));
parser.addParameter('EnvelopeLoadsN', 100:10:2500, ...
    @(x) wsc.validateNonnegativeVector(x, 'EnvelopeLoadsN'));
parser.addParameter('Wheelbase', 1.528, @wsc.validatePositiveScalar);
parser.addParameter('CgHeight', 0.256, @wsc.validateNonnegativeScalar);
parser.addParameter('StaticFrontWeight', 0.5095, @wsc.validateUnitScalar);
parser.addParameter('DistanceM', 75.0, @wsc.validatePositiveScalar);
parser.addParameter('ReferenceMassKg', 264, @wsc.validateNonnegativeScalar);
parser.addParameter('CgDropPerKgSavedM', 0.001, @wsc.validateNonnegativeScalar);
parser.addParameter('CoupledCg', false, @wsc.validateScalarLogical);
parser.addParameter('UsePowertrainLimit', false, @wsc.validateScalarLogical);
parser.addParameter('PowertrainMatFile', '', @(x) ischar(x) || isstring(x));
parser.addParameter('PowertrainEfficiency', 0.90, ...
    @wsc.validateUnitScalarOrEmpty);
parser.addParameter('MotorRotorInertia', 0.07, @wsc.validateNonnegativeScalar);
wsc.addFigureOptions(parser, defaultOutput);
parser.parse(varargin{:});
opts = parser.Results;
loadTransfer = struct( ...
    'wheelbase', opts.Wheelbase, ...
    'cgHeight', opts.CgHeight, ...
    'staticFrontWeight', opts.StaticFrontWeight);

tirFile = char(tirFile);
massRange = opts.MassRangeKg(:).';
slipRatios = opts.EnvelopeSlipRatios(:).';
envelopeLoadsN = unique(opts.EnvelopeLoadsN(:), 'sorted').';
if numel(massRange) ~= 2 || massRange(2) <= massRange(1)
    error('weight_savings_acceleration:BadMassRange', ...
        'MassRangeKg must be a two-element vector [min max] with min < max.');
end
if isempty(slipRatios)
    error('weight_savings_acceleration:EmptySlipRatios', ...
        'EnvelopeSlipRatios must contain at least one value.');
end
if isempty(envelopeLoadsN)
    error('weight_savings_acceleration:EmptyLoads', ...
        'EnvelopeLoadsN must contain at least one value.');
end

tire = lts.components.Tire.PacejkaTire(tirFile);
surfaceMu = 1.0;
kneeFields = {'launchG', 'launchAxMps2', 'timeToDistanceS'};

powertrain = [];
if opts.UsePowertrainLimit
    powertrain = buildPowertrain(opts);
end

tireFilePath = tire.tireConstants.tirFilePath;
envelope = wsc.longitudinalEnvelope(tire, envelopeLoadsN, slipRatios);
massKg = wsc.massGrid(massRange, opts.MassStepKg);
sweep = sweepMass(massKg, envelope, loadTransfer, opts.DistanceM, ...
    powertrain);
kneePoint = wsc.massSweepKnee(sweep, 'timeToDistanceS', kneeFields);

% Optional coupled-CG sweep: every kg of mass saved also lowers the CG by
% CgDropPerKgSavedM (default 1 mm/kg). Anchored at the reference mass so the
% two curves coincide there and diverge as the car gets lighter. NOTE the
% sign is opposite to the skidpad case: for RWD grip-limited launch, lowering
% the CG REDUCES rear-axle load transfer, so the driven axle is less planted
% and traction FALLS. The coupled curve therefore lies ABOVE the fixed-CG
% curve in time (worse) at low mass -- the honest trade-off between a low-CG
% car and a traction-friendly high-CG launch. Skip entirely when CoupledCg
% is off (no sweep, no report block, no figure).
sweepCoupled = [];
kneePointCoupled = wsc.emptyKnee(kneeFields);
coupledCgHeightM = [];
if opts.CoupledCg
    coupledTransfer = loadTransfer;
    coupledTransfer.cgHeight = wsc.coupledCgHeight(massKg, ...
        loadTransfer.cgHeight, opts.ReferenceMassKg, opts.CgDropPerKgSavedM);
    coupledCgHeightM = coupledTransfer.cgHeight;
    sweepCoupled = sweepMass(massKg, envelope, coupledTransfer, ...
        opts.DistanceM, powertrain);
    kneePointCoupled = wsc.massSweepKnee(sweepCoupled, 'timeToDistanceS', ...
        kneeFields);
end

printReport(tireFilePath, surfaceMu, loadTransfer, massKg, sweep, ...
    kneePoint, opts.ReferenceMassKg, opts.DistanceM, ...
    opts.CgDropPerKgSavedM, sweepCoupled, kneePointCoupled, powertrain);

visibleState = wsc.onOff(opts.ShowFigure, 'on', 'off');
outputFile = char(opts.OutputFile);
fig = [];
figCoupled = [];
if opts.SavePlot
    fig = plotCurve(sweep, kneePoint, opts.ReferenceMassKg, opts.DistanceM, ...
        tireFilePath, surfaceMu, loadTransfer, visibleState, powertrain);
    wsc.writeFigureToFile(fig, outputFile);
    if opts.CoupledCg
        coupledFile = wsc.coupledOutputFile(outputFile);
        figCoupled = plotCoupledCgCurve(sweep, kneePoint, sweepCoupled, ...
            kneePointCoupled, opts.ReferenceMassKg, opts.CgDropPerKgSavedM, ...
            opts.DistanceM, tireFilePath, surfaceMu, loadTransfer, ...
            visibleState, powertrain);
        wsc.writeFigureToFile(figCoupled, coupledFile);
    end
end

result = struct( ...
    'tirFile', tireFilePath, ...
    'surfaceMu', surfaceMu, ...
    'loadTransfer', loadTransfer, ...
    'distanceM', opts.DistanceM, ...
    'massKg', massKg, ...
    'launchAxMps2', sweep.launchAxMps2, ...
    'launchG', sweep.launchG, ...
    'timeToDistanceS', sweep.timeToDistanceS, ...
    'benefitGPerKgSaved', sweep.benefitGPerKgSaved, ...
    'benefitSPerKgSaved', sweep.benefitSPerKgSaved, ...
    'limitedBy', sweep.limitedBy, ...
    'rearNormalN', sweep.rearNormalN, ...
    'rearPeakMu', sweep.rearPeakMu, ...
    'envelope', envelope, ...
    'kneePoint', kneePoint, ...
    'outputFile', outputFile, ...
    'figure', fig, ...
    'cgDropPerKgSavedM', opts.CgDropPerKgSavedM, ...
    'coupledCgHeightM', coupledCgHeightM, ...
    'sweepCoupled', sweepCoupled, ...
    'kneePointCoupled', kneePointCoupled, ...
    'figureCoupled', figCoupled);

if opts.CloseFigure && ~isempty(fig)
    close(fig);
    result.figure = [];
end
if opts.CloseFigure && ~isempty(figCoupled)
    close(figCoupled);
    result.figureCoupled = [];
end
end

function powertrain = buildPowertrain(opts)
matFile = char(opts.PowertrainMatFile);
if isempty(matFile)
    cfg = lts.vehicles.R25();
    if isfield(cfg, 'powertrain') && isfield(cfg.powertrain, 'matFile')
        matFile = cfg.powertrain.matFile;
    else
        error('weight_savings_acceleration:NoPowertrainMat', ...
            ['UsePowertrainLimit is on but no PowertrainMatFile was given ' ...
             'and lts.vehicles.R25 has no powertrain.matFile.']);
    end
    efficiency = cfg.powertrain.efficiency;
    rotorInertia = lts.util.fieldOr(cfg.powertrain, 'motorRotorInertia', ...
        opts.MotorRotorInertia);
else
    efficiency = opts.PowertrainEfficiency;
    if isempty(efficiency)
        efficiency = 0.90;
    end
    rotorInertia = opts.MotorRotorInertia;
end
powertrain = lts.components.Powertrain.EMRAX228Powertrain( ...
    matFile, efficiency, rotorInertia);
end

function sweep = sweepMass(massKg, envelope, loadTransfer, distanceM, ...
        powertrain)
% With aero neglected and a speed-independent grip limit, launch ax is
% constant over the whole distance, so the 75 m time is the closed form
% T = sqrt(2*d/ax) (no integration needed). This is the longitudinal twin of
% the skidpad script's T = 5.9527 / sqrt(a_y): the 1/sqrt(ax) compression is
% what creates diminishing returns in time even though raw g has accelerating
% returns to lightness.
nMass = numel(massKg);
launchAxMps2 = zeros(nMass, 1);
launchG = zeros(nMass, 1);
timeToDistanceS = zeros(nMass, 1);
rearNormalN = zeros(nMass, 1);
rearPeakMu = zeros(nMass, 1);
limitedBy = strings(nMass, 1);
cgHeightM = loadTransfer.cgHeight;
if isscalar(cgHeightM)
    cgHeightM = cgHeightM(1) * ones(nMass, 1);
else
    cgHeightM = cgHeightM(:);
end

for i = 1:nMass
    sampleTransfer = loadTransfer;
    sampleTransfer.cgHeight = cgHeightM(i);
    [launchAxMps2(i), detail] = solveLaunchGrip(massKg(i), envelope, ...
        sampleTransfer, powertrain);
    launchG(i) = launchAxMps2(i) / 9.80665;
    timeToDistanceS(i) = sqrt(2 * distanceM / max(launchAxMps2(i), eps));
    rearNormalN(i) = detail.rearNormalN;
    rearPeakMu(i) = detail.rearPeakMu;
    limitedBy(i) = string(detail.limitedBy);
end

if nMass >= 2
    % Marginal benefit of removing 1 kg, reported as a positive quantity.
    %   - launch g: heavier -> less g, so dG/dm < 0; gain per kg = -dG/dm.
    %   - time:     heavier -> more time, so dT/dm > 0; time saved = +dT/dm.
    % Time is where diminishing returns live: the 1/sqrt(ax) compression in T
    % means each kg saved shaves less time off as the car gets lighter.
    dTdm = gradient(timeToDistanceS, massKg);
    benefitSPerKgSaved = wsc.smoothMarginal(dTdm, nMass);
    dGdKg = gradient(launchG, massKg);
    benefitGPerKgSaved = -wsc.smoothMarginal(dGdKg, nMass);
else
    benefitSPerKgSaved = NaN(nMass, 1);
    benefitGPerKgSaved = NaN(nMass, 1);
end

sweep = struct( ...
    'massKg', massKg, ...
    'launchAxMps2', launchAxMps2, ...
    'launchG', launchG, ...
    'timeToDistanceS', timeToDistanceS, ...
    'benefitGPerKgSaved', benefitGPerKgSaved, ...
    'benefitSPerKgSaved', benefitSPerKgSaved, ...
    'limitedBy', limitedBy, ...
    'rearNormalN', rearNormalN, ...
    'rearPeakMu', rearPeakMu, ...
    'cgHeightM', cgHeightM);
end

function [axMps2, detail] = solveLaunchGrip(massKg, envelope, loadTransfer, ...
        powertrain)
% Fixed-point solve for grip-limited launch acceleration (aero neglected).
% Rear load depends on ax (load transfer) which depends on rear load (grip),
% so iterate to convergence -- same scheme as theoretical_acceleration_75m.m
% but with aero and rolling resistance stripped and the powertrain cap
% optional. Returns ax [m/s^2] and the operating-point detail.
ax = 0;
rearNormalN = 0;
rearPeakMu = 0;
FtractionRear = 0;
Fmotor = Inf;
Fdrive = 0;

for iter = 1:12
    rearNormalN = bicycleRearLoad(massKg, ax, loadTransfer);
    rearPerTire = rearNormalN / 2;
    rearPeakMu = peakMuForLoad(rearPerTire, envelope);
    FtractionRear = rearPeakMu * rearNormalN;

    if ~isempty(powertrain)
        Fmotor = max(0, powertrain.computeMaxDriveForce(0));
    else
        Fmotor = Inf;
    end

    Fdrive = min(Fmotor, FtractionRear);
    axNew = Fdrive / massKg;
    if abs(axNew - ax) < 1e-5
        ax = axNew;
        break;
    end
    ax = 0.6 * ax + 0.4 * axNew;
end

axMps2 = ax;
if isfinite(Fmotor) && Fmotor < FtractionRear - 1e-6
    limitedBy = 'power';
else
    limitedBy = 'grip';
end

detail = struct( ...
    'rearNormalN', rearNormalN, ...
    'rearPeakMu', rearPeakMu, ...
    'FtractionRear', FtractionRear, ...
    'Fmotor', Fmotor, ...
    'Fdrive', Fdrive, ...
    'limitedBy', limitedBy);
end

function rearNormal = bicycleRearLoad(massKg, axMps2, loadTransfer)
% Rear-axle normal load under longitudinal load transfer (bicycle model,
% RWD). Positive ax (acceleration) transfers load onto the rear (driven)
% axle. Floored at 0 so an unphysical operating point cannot produce negative
% capacity through extrapolation.
g = 9.80665;
W = massKg * g;
rearStatic = W * (1 - loadTransfer.staticFrontWeight);
totalTransfer = massKg * axMps2 * loadTransfer.cgHeight / ...
    loadTransfer.wheelbase;
rearNormal = max(rearStatic + totalTransfer, 0);
end

function mu = peakMuForLoad(loadPerTire, envelope)
% Peak longitudinal mu at a given per-tire normal load, interpolated from the
% envelope. Per-tire because the envelope peakForceN is per tire; the caller
% multiplies back by total axle load.
if loadPerTire <= 0
    mu = 0;
    return;
end
peakForce = max(interp1(envelope.loadsN, envelope.peakForceN, loadPerTire, ...
    'linear', 'extrap'), 0);
mu = peakForce / max(loadPerTire, eps);
end

function printReport(tireFilePath, surfaceMu, loadTransfer, massKg, sweep, ...
        knee, referenceMassKg, distanceM, cgDropPerKgSavedM, sweepCoupled, ...
        kneeCoupled, powertrain)
[~, tireName, tireExt] = fileparts(tireFilePath);
fprintf('\n=== Weight Savings Acceleration (grip-limited, RWD, aero neglected) ===\n');
fprintf('Tire file:           %s%s\n', tireName, tireExt);
fprintf('Surface mu:          %.3f\n', surfaceMu);
fprintf('Geometry:            wheelbase %.3f m, CG %.3f m, static front %.1f%%\n', ...
    loadTransfer.wheelbase, loadTransfer.cgHeight, ...
    100 * loadTransfer.staticFrontWeight);
fprintf('Event:               FSAE acceleration, %.1f m, T = sqrt(2*d/ax)\n', ...
    distanceM);
fprintf('Limiter:             %s\n', limitModeLabel(powertrain));
fprintf('Mass sweep:          %.1f to %.1f kg (%d points)\n', ...
    min(massKg), max(massKg), numel(massKg));

[refIdx, refLimit] = wsc.indexAtMass(referenceMassKg, massKg, sweep);
if isfinite(refIdx)
    fprintf('Reference mass:      %.1f kg -> %.3f g, %.3f s, limited by %s\n', ...
        referenceMassKg, sweep.launchG(refIdx), ...
        sweep.timeToDistanceS(refIdx), refLimit);
end

% The headline answer is in TIME, where diminishing returns are real: the
% 1/sqrt(ax) compression means each kg saved shaves less time off as the car
% gets lighter. Raw launch g has ACCELERATING returns (tire mu rises as load
% falls), so the g curve bends the wrong way for diminishing returns.
fprintf('>> Diminishing-return mass (time knee): %.1f kg', knee.massKg);
if isfinite(knee.timeToDistanceS)
    fprintf(' (%.3f s, %.3f g', knee.timeToDistanceS, knee.launchG);
end
if isfinite(knee.benefitSPerKgSaved)
    fprintf(', %.4f s/kg saved', knee.benefitSPerKgSaved);
end
fprintf(', %s)\n', knee.method);

gCurv = wsc.curveLinearity(massKg, sweep.launchG);
tCurv = wsc.curveLinearity(massKg, sweep.timeToDistanceS);
fprintf('   g curve:    quad coeff %+.3e  (>=0 = accelerating returns, <0 = diminishing)\n', gCurv.quadCoeff);
fprintf('   Time:       quad coeff %+.3e  (>=0 = diminishing returns as car gets lighter)\n', tCurv.quadCoeff);
if gCurv.quadCoeff >= 0 && tCurv.quadCoeff >= 0
    fprintf('   Raw g gives accelerating returns to lightness; time shows diminishing returns\n');
    fprintf('   (each kg saved shaves less time off as the car gets lighter) but the elbow is gentle.\n');
elseif tCurv.quadCoeff < 0
    fprintf('   Time curve is concave; knee marks the diminishing-returns elbow.\n');
end

endpts = [1, numel(massKg)];
for k = 1:numel(endpts)
    i = endpts(k);
    fprintf('   At %.0f kg: %.3f g, %.3f s', massKg(i), ...
        sweep.launchG(i), sweep.timeToDistanceS(i));
    if isfinite(sweep.benefitSPerKgSaved(i))
        fprintf(', %.4f s/kg', sweep.benefitSPerKgSaved(i));
    end
    fprintf(' (%s)\n', sweep.limitedBy(i));
end
fprintf('Aero downforce and drag are neglected; only tire load sensitivity couples mass to grip.\n');

wsc.printCoupledCgReport(massKg, knee, sweepCoupled, kneeCoupled, ...
    referenceMassKg, cgDropPerKgSavedM, 'timeToDistanceS', 'launchG', ...
    'time', '%.3f s');
end

function label = limitModeLabel(powertrain)
if isempty(powertrain)
    label = 'grip-limited (powertrain cap off)';
else
    label = 'grip- or power-limited (powertrain cap on)';
end
end

function fig = plotCurve(sweep, knee, referenceMassKg, distanceM, ...
        tireFilePath, surfaceMu, loadTransfer, visibleState, powertrain)
colors = wsc.plotColors();
fig = figure('Name', 'Weight savings diminishing returns (acceleration)', ...
    'Color', 'w', 'Visible', visibleState);
fig.Position = [120 60 1120 900];
wsc.styleFigure(fig);

layout = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'loose');
mass = sweep.massKg;
launchG = sweep.launchG;
eventTime = sweep.timeToDistanceS;
benefitMsPerKg = 1000 * sweep.benefitSPerKgSaved;

% --- Tile 1: launch g vs mass (accelerating returns to lightness) ---
axAccel = nexttile(layout);
plot(axAccel, mass, launchG, 'Color', colors.blue, 'LineWidth', 1.8, ...
    'DisplayName', 'Launch a_x');
hold(axAccel, 'on');
wsc.referenceLine(axAccel, referenceMassKg, colors.ref);
wsc.kneeMarker(axAccel, knee.massKg, knee.launchG, colors.knee, '');
hold(axAccel, 'off');
wsc.styleAxis(axAccel);
grid(axAccel, 'on');
ylabel(axAccel, 'Launch a_x [g]');
title(axAccel, 'Grip-limited launch g vs mass (convex: lighter pays off progressively more per kg)');
legend(axAccel, 'Location', 'best');

% --- Tile 2: 75 m time vs mass (where diminishing returns live) ---
axTime = nexttile(layout);
plot(axTime, mass, eventTime, 'Color', colors.blue, 'LineWidth', 1.8, ...
    'DisplayName', sprintf('%.0f m time', distanceM));
hold(axTime, 'on');
wsc.referenceLine(axTime, referenceMassKg, colors.ref);
wsc.kneeMarker(axTime, knee.massKg, knee.timeToDistanceS, colors.knee, ...
    'Diminishing-returns knee');
hold(axTime, 'off');
wsc.styleAxis(axTime);
grid(axTime, 'on');
ylabel(axTime, sprintf('Time to %.0f m [s]', distanceM));
title(axTime, 'Acceleration time vs mass (concave: each kg saved shaves less time off as car gets lighter)');
legend(axTime, 'Location', 'best');

% --- Tile 3: marginal time benefit per kg saved (the key panel) ---
axBenefit = nexttile(layout);
plot(axBenefit, mass, benefitMsPerKg, 'Color', colors.green, ...
    'LineWidth', 1.6, 'DisplayName', 'Marginal time gain');
hold(axBenefit, 'on');
wsc.zeroLine(axBenefit, colors.ref);
wsc.referenceLine(axBenefit, referenceMassKg, colors.ref);
wsc.kneeMarker(axBenefit, knee.massKg, ...
    1000 * knee.benefitSPerKgSaved, colors.knee, ...
    'Diminishing-returns knee');
hold(axBenefit, 'off');
wsc.styleAxis(axBenefit);
grid(axBenefit, 'on');
xlabel(axBenefit, 'Vehicle mass [kg]');
ylabel(axBenefit, 'Gain [ms / kg saved]');
title(axBenefit, 'Marginal time gain per kg removed (falls as car gets lighter)');
legend(axBenefit, 'Location', 'best');

linkaxes([axAccel, axTime, axBenefit], 'x');
[~, tireName, tireExt] = fileparts(tireFilePath);
plotTitle = sgtitle(sprintf( ...
    ['Weight savings vs FSAE acceleration | %s%s | surface mu %.3f | %s | ' ...
     'wheelbase %.2f m, CG %.2f m, front %.0f%%'], ...
    tireName, tireExt, surfaceMu, limitModeLabel(powertrain), ...
    loadTransfer.wheelbase, loadTransfer.cgHeight, ...
    100 * loadTransfer.staticFrontWeight), 'Interpreter', 'none');
set(plotTitle, 'Color', 'k', 'FontWeight', 'bold');
end

function fig = plotCoupledCgCurve(sweep, knee, sweepCoupled, kneeCoupled, ...
        referenceMassKg, cgDropPerKgSavedM, distanceM, tireFilePath, ...
        surfaceMu, loadTransfer, visibleState, powertrain)
% Comparison figure: fixed CG vs coupled CG (CG drops 1 mm per kg saved,
% anchored at the reference mass). Same three panels as plotCurve, with both
% sweeps overlaid. The coupled curve sits ABOVE the fixed-CG curve in time
% (worse) at low mass because lowering the CG reduces the rearward load
% transfer that plants the driven axle -- the opposite sign to skidpad.
colors = wsc.plotColors();
fig = figure('Name', 'Weight savings: fixed CG vs coupled CG (acceleration)', ...
    'Color', 'w', 'Visible', visibleState);
fig.Position = [120 60 1120 900];
wsc.styleFigure(fig);

layout = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'loose');
mass = sweep.massKg;
launchG = sweep.launchG;
launchGCoupled = sweepCoupled.launchG;
eventTime = sweep.timeToDistanceS;
eventTimeCoupled = sweepCoupled.timeToDistanceS;
benefitMsPerKg = 1000 * sweep.benefitSPerKgSaved;
benefitMsPerKgCoupled = 1000 * sweepCoupled.benefitSPerKgSaved;

% --- Tile 1: launch g vs mass ---
axAccel = nexttile(layout);
plot(axAccel, mass, launchG, 'Color', colors.blue, 'LineWidth', 1.8, ...
    'DisplayName', 'Launch a_x (fixed CG)');
hold(axAccel, 'on');
plot(axAccel, mass, launchGCoupled, 'Color', colors.orange, ...
    'LineWidth', 1.8, 'DisplayName', 'Launch a_x (CG -1 mm/kg)');
wsc.referenceLine(axAccel, referenceMassKg, colors.ref);
wsc.kneePoint(axAccel, knee.massKg, knee.launchG, colors.knee, ...
    'Fixed-CG knee');
wsc.kneePoint(axAccel, kneeCoupled.massKg, kneeCoupled.launchG, ...
    colors.kneeCoupled, 'Coupled-CG knee', 's');
hold(axAccel, 'off');
wsc.styleAxis(axAccel);
grid(axAccel, 'on');
ylabel(axAccel, 'Launch a_x [g]');
title(axAccel, 'Launch g vs mass (lower CG reduces rear load transfer, hurts traction)');
legend(axAccel, 'Location', 'best');

% --- Tile 2: time vs mass ---
axTime = nexttile(layout);
plot(axTime, mass, eventTime, 'Color', colors.blue, 'LineWidth', 1.8, ...
    'DisplayName', 'Time (fixed CG)');
hold(axTime, 'on');
plot(axTime, mass, eventTimeCoupled, 'Color', colors.orange, ...
    'LineWidth', 1.8, 'DisplayName', 'Time (CG -1 mm/kg)');
wsc.referenceLine(axTime, referenceMassKg, colors.ref);
wsc.kneePoint(axTime, knee.massKg, knee.timeToDistanceS, colors.knee, ...
    'Fixed-CG knee');
wsc.kneePoint(axTime, kneeCoupled.massKg, kneeCoupled.timeToDistanceS, ...
    colors.kneeCoupled, 'Coupled-CG knee', 's');
hold(axTime, 'off');
wsc.styleAxis(axTime);
grid(axTime, 'on');
ylabel(axTime, sprintf('Time to %.0f m [s]', distanceM));
title(axTime, 'Time vs mass (coupled CG is slower at low mass: less rear planting)');
legend(axTime, 'Location', 'best');

% --- Tile 3: marginal time benefit per kg saved ---
axBenefit = nexttile(layout);
plot(axBenefit, mass, benefitMsPerKg, 'Color', colors.green, ...
    'LineWidth', 1.6, 'DisplayName', 'Marginal gain (fixed CG)');
hold(axBenefit, 'on');
plot(axBenefit, mass, benefitMsPerKgCoupled, 'Color', colors.teal, ...
    'LineWidth', 1.6, 'DisplayName', 'Marginal gain (CG -1 mm/kg)');
wsc.zeroLine(axBenefit, colors.ref);
wsc.referenceLine(axBenefit, referenceMassKg, colors.ref);
wsc.kneePoint(axBenefit, knee.massKg, 1000 * knee.benefitSPerKgSaved, ...
    colors.knee, 'Fixed-CG knee');
wsc.kneePoint(axBenefit, kneeCoupled.massKg, ...
    1000 * kneeCoupled.benefitSPerKgSaved, colors.kneeCoupled, ...
    'Coupled-CG knee', 's');
hold(axBenefit, 'off');
wsc.styleAxis(axBenefit);
grid(axBenefit, 'on');
xlabel(axBenefit, 'Vehicle mass [kg]');
ylabel(axBenefit, 'Gain [ms / kg saved]');
title(axBenefit, 'Marginal time gain per kg removed (coupled CG stays below fixed CG at low mass)');
legend(axBenefit, 'Location', 'best');

linkaxes([axAccel, axTime, axBenefit], 'x');
[~, tireName, tireExt] = fileparts(tireFilePath);
plotTitle = sgtitle(sprintf( ...
    ['Weight savings vs acceleration: fixed CG vs coupled CG (-%.1f mm/kg, ' ...
     'anchored at %.0f kg) | %s%s | surface mu %.3f | %s | wheelbase %.2f ' ...
     'm, base CG %.2f m, front %.0f%%'], ...
    1000 * cgDropPerKgSavedM, referenceMassKg, tireName, tireExt, surfaceMu, ...
    limitModeLabel(powertrain), loadTransfer.wheelbase, ...
    loadTransfer.cgHeight, 100 * loadTransfer.staticFrontWeight), ...
    'Interpreter', 'none');
set(plotTitle, 'Color', 'k', 'FontWeight', 'bold');
end
