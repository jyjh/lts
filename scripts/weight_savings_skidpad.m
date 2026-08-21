function result = weight_savings_skidpad(tirFile, varargin)
%WEIGHT_SAVINGS_SKIDPAD Find the point of diminishing returns for weight
%savings in terms of max sustained skidpad lateral G.
%
%   result = weight_savings_skidpad()
%   result = weight_savings_skidpad('43105_18x7.5_10_R25B_7.tir')
%   result = weight_savings_skidpad(tirFile, 'MassRangeKg', [180 340], ...
%       'CgHeight', 0.256, 'StaticFrontWeight', 0.5095)
%
%   Sweeps vehicle mass and, at each mass, solves for the maximum steady-
%   state skidpad lateral acceleration using a simplified bicycle load-
%   transfer model fed by the Pacejka tire's peak lateral-force envelope.
%   Aero downforce is neglected (a clean grip-via-load-sensitivity study).
%   The diminishing-returns mass is found with the Kneedle elbow (max
%   perpendicular distance from the chord of the G-vs-mass curve) and
%   reported together with the marginal gain [milli-g per kg saved].
%
%   Defaults target the theoretical tire and R25 measured geometry so the
%   default call returns a meaningful answer; every parameter is overridable.
%   Shared sweep/plot/option machinery lives in the +wsc package next to this
%   script and is reused by weight_savings_acceleration and tire_sensitivity.

defaultTirFile = '43105_18x7.5_10_R25B_7_theoretical.tir';
optionNames = {'MassRangeKg', 'MassStepKg', 'EnvelopeSlipAnglesDeg', ...
    'EnvelopeLoadsN', 'TrackWidth', 'CgHeight', ...
    'StaticFrontWeight', 'LateralLoadTransferDistribution', ...
    'ReferenceMassKg', 'CgDropPerKgSavedM', 'CoupledCg', 'OutputFile', ...
    'SavePlot', 'ShowFigure', 'CloseFigure'};
[tirFile, varargin] = wsc.resolveTireArg(tirFile, defaultTirFile, ...
    optionNames, varargin{:});
wsc.addScriptPaths();

defaultOutput = fullfile(fileparts(fileparts(mfilename('fullpath'))), ...
    'exports', 'weight_savings_skidpad.png');

parser = inputParser;
parser.FunctionName = 'weight_savings_skidpad';
parser.addParameter('MassRangeKg', [180 280], ...
    @(x) wsc.validatePositiveVector(x, 'MassRangeKg'));
parser.addParameter('MassStepKg', 0.5, @wsc.validatePositiveScalar);
parser.addParameter('EnvelopeSlipAnglesDeg', 0:0.1:14, ...
    @(x) wsc.validateFiniteVector(x, 'EnvelopeSlipAnglesDeg'));
parser.addParameter('EnvelopeLoadsN', 100:10:2500, ...
    @(x) wsc.validateNonnegativeVector(x, 'EnvelopeLoadsN'));
parser.addParameter('TrackWidth', 1.21, @wsc.validatePositiveScalar);
parser.addParameter('CgHeight', 0.256, @wsc.validateNonnegativeScalar);
parser.addParameter('StaticFrontWeight', 0.5095, @wsc.validateUnitScalar);
parser.addParameter('LateralLoadTransferDistribution', [], ...
    @wsc.validateUnitScalarOrEmpty);
parser.addParameter('ReferenceMassKg', 264, @wsc.validateNonnegativeScalar);
parser.addParameter('CgDropPerKgSavedM', 0.001, @wsc.validateNonnegativeScalar);
parser.addParameter('CoupledCg', true, @wsc.validateScalarLogical);
wsc.addFigureOptions(parser, defaultOutput);
parser.parse(varargin{:});
opts = parser.Results;
loadTransfer = wsc.skidpadLoadTransfer(opts);

tirFile = char(tirFile);
massRange = opts.MassRangeKg(:).';
slipAnglesDeg = opts.EnvelopeSlipAnglesDeg(:).';
envelopeLoadsN = unique(opts.EnvelopeLoadsN(:), 'sorted').';
if numel(massRange) ~= 2 || massRange(2) <= massRange(1)
    error('weight_savings_skidpad:BadMassRange', ...
        'MassRangeKg must be a two-element vector [min max] with min < max.');
end
if isempty(slipAnglesDeg)
    error('weight_savings_skidpad:EmptySlipAngles', ...
        'EnvelopeSlipAnglesDeg must contain at least one value.');
end
if isempty(envelopeLoadsN)
    error('weight_savings_skidpad:EmptyLoads', ...
        'EnvelopeLoadsN must contain at least one value.');
end

tire = lts.components.Tire.PacejkaTire(tirFile);
surfaceMu = 1.0;
kneeFields = {'accelG=sustainedG', 'accelMps2=sustainedMps2', ...
    'lapTimeS=lapTimeS'};

tireFilePath = tire.tireConstants.tirFilePath;
envelope = wsc.lateralEnvelope(tire, envelopeLoadsN, slipAnglesDeg);
massKg = wsc.massGrid(massRange, opts.MassStepKg);
sweep = sweepMass(massKg, envelope, loadTransfer);
kneePoint = wsc.massSweepKnee(sweep, 'lapTimeS', kneeFields);

% Optional coupled-CG sweep: every kg of mass saved also lowers the CG by
% CgDropPerKgSavedM (default 1 mm/kg). Anchored at the reference mass so the
% two curves coincide there and diverge as the car gets lighter. Lower CG ->
% less load transfer -> more capacity at low mass, so lightness is rewarded
% more steeply and the diminishing-returns knee moves. Skip entirely when
% CoupledCg is off (no sweep, no report block, no figure).
sweepCoupled = [];
kneePointCoupled = wsc.emptyKnee(kneeFields);
coupledCgHeightM = [];
if opts.CoupledCg
    coupledTransfer = loadTransfer;
    coupledTransfer.cgHeight = wsc.coupledCgHeight(massKg, ...
        loadTransfer.cgHeight, opts.ReferenceMassKg, opts.CgDropPerKgSavedM);
    coupledCgHeightM = coupledTransfer.cgHeight;
    sweepCoupled = sweepMass(massKg, envelope, coupledTransfer);
    kneePointCoupled = wsc.massSweepKnee(sweepCoupled, 'lapTimeS', ...
        kneeFields);
end

printReport(tireFilePath, surfaceMu, loadTransfer, massKg, sweep, ...
    kneePoint, opts.ReferenceMassKg, opts.CgDropPerKgSavedM, ...
    sweepCoupled, kneePointCoupled);

visibleState = wsc.onOff(opts.ShowFigure, 'on', 'off');
outputFile = char(opts.OutputFile);
fig = [];
figCoupled = [];
if opts.SavePlot
    fig = plotCurve(sweep, kneePoint, opts.ReferenceMassKg, tireFilePath, ...
        surfaceMu, loadTransfer, visibleState);
    wsc.writeFigureToFile(fig, outputFile);
    if opts.CoupledCg
        coupledFile = wsc.coupledOutputFile(outputFile);
        figCoupled = plotCoupledCgCurve(sweep, kneePoint, sweepCoupled, ...
            kneePointCoupled, opts.ReferenceMassKg, opts.CgDropPerKgSavedM, ...
            tireFilePath, surfaceMu, loadTransfer, visibleState);
        wsc.writeFigureToFile(figCoupled, coupledFile);
    end
end

result = struct( ...
    'tirFile', tireFilePath, ...
    'surfaceMu', surfaceMu, ...
    'loadTransfer', loadTransfer, ...
    'massKg', massKg, ...
    'sustainedG', sweep.sustainedG, ...
    'sustainedMps2', sweep.sustainedMps2, ...
    'lapTimeS', sweep.lapTimeS, ...
    'benefitGPerKgSaved', sweep.benefitGPerKgSaved, ...
    'benefitSPerKgSaved', sweep.benefitSPerKgSaved, ...
    'limitedBy', sweep.limitedBy, ...
    'cornerLoadsN', sweep.cornerLoadsN, ...
    'frontCapacityN', sweep.frontCapacityN, ...
    'rearCapacityN', sweep.rearCapacityN, ...
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

function sweep = sweepMass(massKg, envelope, loadTransfer)
% FSAE skidpad figure-8 constant: lap time T [s] relates to sustained lateral
% accel a_y [g] as a_y = (5.9527 / T)^2, so T = 5.9527 / sqrt(a_y) for the
% timed figure-8 run. This 1/sqrt compression is what creates diminishing
% returns in lap time even though raw G has accelerating returns to lightness.
FSAE_SKIDPAD_CONST = 5.9527;

nMass = numel(massKg);
sustainedG = zeros(nMass, 1);
sustainedMps2 = zeros(nMass, 1);
lapTimeS = zeros(nMass, 1);
cornerLoadsN = zeros(nMass, 4);
frontCapacityN = zeros(nMass, 1);
rearCapacityN = zeros(nMass, 1);
limitedBy = strings(nMass, 1);
% Resolve the per-sample CG height actually used in the load-transfer model.
% For a fixed-CG sweep loadTransfer.cgHeight is a scalar (broadcast); for a
% coupled-CG sweep it is a per-sample vector.
cgHeightM = loadTransfer.cgHeight;
if isscalar(cgHeightM)
    cgHeightM = cgHeightM(1) * ones(nMass, 1);
else
    cgHeightM = cgHeightM(:);
end

for i = 1:nMass
    sampleTransfer = loadTransfer;
    sampleTransfer.cgHeight = cgHeightM(i);
    [sustainedG(i), detail] = wsc.solveSkidpadCapacity( ...
        massKg(i), envelope.loadsN, envelope.peakForceN, ...
        envelope.slipAtPeakDeg, sampleTransfer);
    sustainedMps2(i) = sustainedG(i) * 9.80665;
    lapTimeS(i) = FSAE_SKIDPAD_CONST / sqrt(max(sustainedG(i), eps));
    cornerLoadsN(i, :) = detail.cornerLoadsN(:).';
    frontCapacityN(i) = detail.frontCapacityN;
    rearCapacityN(i) = detail.rearCapacityN;
    limitedBy(i) = string(detail.limitedBy);
end

if nMass >= 2
    % Marginal benefit of removing 1 kg, reported as a positive quantity.
    % Sign depends on whether the metric is "higher-is-better" (G) or
    % "lower-is-better" (lap time):
    %   - G:        heavier -> less G, so dG/dm < 0; gain per kg = -dG/dm.
    %   - lap time: heavier -> more time, so dT/dm > 0; time saved = +dT/dm.
    % Lap time is where diminishing returns actually live: the 1/sqrt(G)
    % compression in T means each kg saved shaves less time off as the car
    % gets lighter. Raw G itself has ACCELERATING returns to lightness (tire
    % mu rises as load falls), so its curvature points the wrong way for
    % diminishing returns; lap time is the honest metric.
    dTdm = gradient(lapTimeS, massKg);
    benefitSPerKgSaved = wsc.smoothMarginal(dTdm, nMass);
    dGdKg = gradient(sustainedG, massKg);
    benefitGPerKgSaved = -wsc.smoothMarginal(dGdKg, nMass);
else
    benefitSPerKgSaved = NaN(nMass, 1);
    benefitGPerKgSaved = NaN(nMass, 1);
end

sweep = struct( ...
    'massKg', massKg, ...
    'sustainedG', sustainedG, ...
    'sustainedMps2', sustainedMps2, ...
    'lapTimeS', lapTimeS, ...
    'benefitGPerKgSaved', benefitGPerKgSaved, ...
    'benefitSPerKgSaved', benefitSPerKgSaved, ...
    'limitedBy', limitedBy, ...
    'cornerLoadsN', cornerLoadsN, ...
    'frontCapacityN', frontCapacityN, ...
    'rearCapacityN', rearCapacityN, ...
    'cgHeightM', cgHeightM);
end

function printReport(tireFilePath, surfaceMu, loadTransfer, massKg, sweep, ...
        knee, referenceMassKg, cgDropPerKgSavedM, sweepCoupled, kneeCoupled)
[~, tireName, tireExt] = fileparts(tireFilePath);
fprintf('\n=== Weight Savings Skidpad (bicycle model, aero neglected) ===\n');
fprintf('Tire file:           %s%s\n', tireName, tireExt);
fprintf('Surface mu:          %.3f\n', surfaceMu);
fprintf('Geometry:            track %.3f m, CG %.3f m, static front %.1f%%, front LLTD %.1f%%\n', ...
    loadTransfer.trackWidth, loadTransfer.cgHeight, ...
    100 * loadTransfer.staticFrontWeight, ...
    100 * loadTransfer.frontLoadTransferDistribution);
fprintf('Skidpad metric:      FSAE figure-8, T = 5.9527 / sqrt(a_y)\n');
fprintf('Mass sweep:          %.1f to %.1f kg (%d points)\n', ...
    min(massKg), max(massKg), numel(massKg));

[refIdx, refLimit] = wsc.indexAtMass(referenceMassKg, massKg, sweep);
if isfinite(refIdx)
    fprintf('Reference mass:      %.1f kg -> %.3f g, %.3f s lap, limited by %s\n', ...
        referenceMassKg, sweep.sustainedG(refIdx), ...
        sweep.lapTimeS(refIdx), refLimit);
end

% The headline answer is in LAP TIME, where diminishing returns are real:
% the 1/sqrt(G) compression means each kg saved shaves less time off as the
% car gets lighter. Raw G itself has ACCELERATING returns (tire mu rises as
% load falls), so the G curve bends the wrong way for diminishing returns.
fprintf('>> Diminishing-return mass (lap-time knee): %.1f kg', knee.massKg);
if isfinite(knee.lapTimeS)
    fprintf(' (%.3f s lap, %.3f g', knee.lapTimeS, knee.accelG);
end
if isfinite(knee.benefitSPerKgSaved)
    fprintf(', %.4f s/kg saved', knee.benefitSPerKgSaved);
end
fprintf(', %s)\n', knee.method);

gCurv = wsc.curveLinearity(massKg, sweep.sustainedG);
tCurv = wsc.curveLinearity(massKg, sweep.lapTimeS);
fprintf('   G curve:    quad coeff %+.3e  (>=0 = accelerating returns, <0 = diminishing)\n', gCurv.quadCoeff);
fprintf('   Lap-time:   quad coeff %+.3e  (>=0 = diminishing returns as car gets lighter)\n', tCurv.quadCoeff);
if gCurv.quadCoeff >= 0 && tCurv.quadCoeff >= 0
    fprintf('   Raw G gives accelerating returns to lightness; lap time shows diminishing returns\n');
    fprintf('   (each kg saved shaves less time off as the car gets lighter) but the elbow is gentle.\n');
elseif tCurv.quadCoeff < 0
    fprintf('   Lap-time curve is concave; knee marks the diminishing-returns elbow.\n');
end

endpts = [1, numel(massKg)];
for k = 1:numel(endpts)
    i = endpts(k);
    fprintf('   At %.0f kg: %.3f g, %.3f s', massKg(i), ...
        sweep.sustainedG(i), sweep.lapTimeS(i));
    if isfinite(sweep.benefitSPerKgSaved(i))
        fprintf(', %.4f s/kg', sweep.benefitSPerKgSaved(i));
    end
    fprintf(' (%s)\n', sweep.limitedBy(i));
end
fprintf('Aero downforce is neglected; only tire load sensitivity couples mass to grip.\n');

wsc.printCoupledCgReport(massKg, knee, sweepCoupled, kneeCoupled, ...
    referenceMassKg, cgDropPerKgSavedM, 'lapTimeS', 'accelG=sustainedG', ...
    'lap-time', '%.3f s lap');
end

function fig = plotCurve(sweep, knee, referenceMassKg, tireFilePath, ...
        surfaceMu, loadTransfer, visibleState)
colors = wsc.plotColors();
fig = figure('Name', 'Weight savings diminishing returns (lap time)', ...
    'Color', 'w', 'Visible', visibleState);
fig.Position = [120 60 1120 900];
wsc.styleFigure(fig);

layout = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'loose');
mass = sweep.massKg;
accel = sweep.sustainedG;
lapTime = sweep.lapTimeS;
benefitMsPerKg = 1000 * sweep.benefitSPerKgSaved;

% --- Tile 1: sustained G vs mass (accelerating returns to lightness) ---
axAccel = nexttile(layout);
plot(axAccel, mass, accel, 'Color', colors.blue, 'LineWidth', 1.8, ...
    'DisplayName', 'Sustained a_y');
hold(axAccel, 'on');
wsc.referenceLine(axAccel, referenceMassKg, colors.ref);
wsc.kneeMarker(axAccel, knee.massKg, knee.accelG, colors.knee, '');
hold(axAccel, 'off');
wsc.styleAxis(axAccel);
grid(axAccel, 'on');
ylabel(axAccel, 'Sustained a_y [g]');
title(axAccel, 'Skidpad G vs mass (convex: lighter pays off progressively more per kg)');
legend(axAccel, 'Location', 'best');

% --- Tile 2: lap time vs mass (where diminishing returns live) ---
axLap = nexttile(layout);
plot(axLap, mass, lapTime, 'Color', colors.blue, 'LineWidth', 1.8, ...
    'DisplayName', 'Lap time');
hold(axLap, 'on');
wsc.referenceLine(axLap, referenceMassKg, colors.ref);
wsc.kneeMarker(axLap, knee.massKg, knee.lapTimeS, colors.knee, ...
    'Diminishing-returns knee');
hold(axLap, 'off');
wsc.styleAxis(axLap);
grid(axLap, 'on');
ylabel(axLap, 'FSAE skidpad lap time [s]');
title(axLap, 'Lap time vs mass (concave: each kg saved shaves less time off as car gets lighter)');
legend(axLap, 'Location', 'best');

% --- Tile 3: marginal lap-time benefit per kg saved (the key panel) ---
axBenefit = nexttile(layout);
plot(axBenefit, mass, benefitMsPerKg, 'Color', colors.green, ...
    'LineWidth', 1.6, 'DisplayName', 'Marginal lap-time gain');
hold(axBenefit, 'on');
wsc.zeroLine(axBenefit, colors.ref);
wsc.referenceLine(axBenefit, referenceMassKg, colors.ref);
wsc.kneeMarker(axBenefit, knee.massKg, 1000 * knee.benefitSPerKgSaved, ...
    colors.knee, 'Diminishing-returns knee');
hold(axBenefit, 'off');
wsc.styleAxis(axBenefit);
grid(axBenefit, 'on');
xlabel(axBenefit, 'Vehicle mass [kg]');
ylabel(axBenefit, 'Gain [ms / kg saved]');
title(axBenefit, 'Marginal lap-time gain per kg removed (falls as car gets lighter)');
legend(axBenefit, 'Location', 'best');

linkaxes([axAccel, axLap, axBenefit], 'x');
[~, tireName, tireExt] = fileparts(tireFilePath);
plotTitle = sgtitle(sprintf( ...
    ['Weight savings vs FSAE skidpad lap time | %s%s | surface mu %.3f | ' ...
     'track %.2f m, CG %.2f m, front %.0f%%, LLTD %.0f%%'], ...
    tireName, tireExt, surfaceMu, loadTransfer.trackWidth, ...
    loadTransfer.cgHeight, 100 * loadTransfer.staticFrontWeight, ...
    100 * loadTransfer.frontLoadTransferDistribution), ...
    'Interpreter', 'none');
set(plotTitle, 'Color', 'k', 'FontWeight', 'bold');
end

function fig = plotCoupledCgCurve(sweep, knee, sweepCoupled, kneeCoupled, ...
        referenceMassKg, cgDropPerKgSavedM, tireFilePath, surfaceMu, ...
        loadTransfer, visibleState)
% Comparison figure: fixed CG vs coupled CG (CG drops 1 mm per kg saved,
% anchored at the reference mass). Same three panels as plotCurve, with both
% sweeps overlaid so the extra grip from reduced load transfer is visible.
colors = wsc.plotColors();
fig = figure('Name', 'Weight savings: fixed CG vs coupled CG (lap time)', ...
    'Color', 'w', 'Visible', visibleState);
fig.Position = [120 60 1120 900];
wsc.styleFigure(fig);

layout = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'loose');
mass = sweep.massKg;
accel = sweep.sustainedG;
accelCoupled = sweepCoupled.sustainedG;
lapTime = sweep.lapTimeS;
lapTimeCoupled = sweepCoupled.lapTimeS;
benefitMsPerKg = 1000 * sweep.benefitSPerKgSaved;
benefitMsPerKgCoupled = 1000 * sweepCoupled.benefitSPerKgSaved;

% --- Tile 1: sustained G vs mass ---
axAccel = nexttile(layout);
plot(axAccel, mass, accel, 'Color', colors.blue, 'LineWidth', 1.8, ...
    'DisplayName', 'Sustained a_y (fixed CG)');
hold(axAccel, 'on');
plot(axAccel, mass, accelCoupled, 'Color', colors.orange, ...
    'LineWidth', 1.8, 'DisplayName', 'Sustained a_y (CG -1 mm/kg)');
wsc.referenceLine(axAccel, referenceMassKg, colors.ref);
wsc.kneePoint(axAccel, knee.massKg, knee.accelG, colors.knee, ...
    'Fixed-CG knee');
wsc.kneePoint(axAccel, kneeCoupled.massKg, kneeCoupled.accelG, ...
    colors.kneeCoupled, 'Coupled-CG knee', 's');
hold(axAccel, 'off');
wsc.styleAxis(axAccel);
grid(axAccel, 'on');
ylabel(axAccel, 'Sustained a_y [g]');
title(axAccel, 'Skidpad G vs mass (lower CG from mass loss steepens the gain)');
legend(axAccel, 'Location', 'best');

% --- Tile 2: lap time vs mass ---
axLap = nexttile(layout);
plot(axLap, mass, lapTime, 'Color', colors.blue, 'LineWidth', 1.8, ...
    'DisplayName', 'Lap time (fixed CG)');
hold(axLap, 'on');
plot(axLap, mass, lapTimeCoupled, 'Color', colors.orange, ...
    'LineWidth', 1.8, 'DisplayName', 'Lap time (CG -1 mm/kg)');
wsc.referenceLine(axLap, referenceMassKg, colors.ref);
wsc.kneePoint(axLap, knee.massKg, knee.lapTimeS, colors.knee, ...
    'Fixed-CG knee');
wsc.kneePoint(axLap, kneeCoupled.massKg, kneeCoupled.lapTimeS, ...
    colors.kneeCoupled, 'Coupled-CG knee', 's');
hold(axLap, 'off');
wsc.styleAxis(axLap);
grid(axLap, 'on');
ylabel(axLap, 'FSAE skidpad lap time [s]');
title(axLap, 'Lap time vs mass (coupled CG buys back grip at low mass)');
legend(axLap, 'Location', 'best');

% --- Tile 3: marginal lap-time benefit per kg saved ---
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
title(axBenefit, 'Marginal lap-time gain per kg removed (coupled CG stays higher at low mass)');
legend(axBenefit, 'Location', 'best');

linkaxes([axAccel, axLap, axBenefit], 'x');
[~, tireName, tireExt] = fileparts(tireFilePath);
plotTitle = sgtitle(sprintf( ...
    ['Weight savings vs skidpad: fixed CG vs coupled CG (-%.1f mm/kg, ' ...
     'anchored at %.0f kg) | %s%s | surface mu %.3f | track %.2f m, ' ...
     'base CG %.2f m, front %.0f%%, LLTD %.0f%%'], ...
    1000 * cgDropPerKgSavedM, referenceMassKg, tireName, tireExt, ...
    surfaceMu, loadTransfer.trackWidth, loadTransfer.cgHeight, ...
    100 * loadTransfer.staticFrontWeight, ...
    100 * loadTransfer.frontLoadTransferDistribution), ...
    'Interpreter', 'none');
set(plotTitle, 'Color', 'k', 'FontWeight', 'bold');
end
