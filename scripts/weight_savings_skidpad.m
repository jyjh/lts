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

defaultTirFile = '43105_18x7.5_10_R25B_7_theoretical.tir';
optionNames = {'MassRangeKg', 'MassStepKg', 'EnvelopeSlipAnglesDeg', ...
    'EnvelopeLoadsN', 'SurfaceMu', 'TrackWidth', 'CgHeight', ...
    'StaticFrontWeight', 'LateralLoadTransferDistribution', ...
    'ReferenceMassKg', 'OutputFile', 'SavePlot', 'ShowFigure', ...
    'CloseFigure'};

if nargin < 1 || isempty(tirFile)
    tirFile = defaultTirFile;
elseif isOptionName(tirFile, optionNames)
    varargin = [{tirFile}, varargin];
    tirFile = defaultTirFile;
end

scriptDir = fileparts(mfilename('fullpath'));
repoRoot = fileparts(scriptDir);
addpath(fullfile(repoRoot, 'src'));

defaultOutput = fullfile(repoRoot, 'exports', 'weight_savings_skidpad.png');

parser = inputParser;
parser.FunctionName = 'weight_savings_skidpad';
parser.addParameter('MassRangeKg', [180 340], @validateMassRange);
parser.addParameter('MassStepKg', 0.5, @validatePositiveScalar);
parser.addParameter('EnvelopeSlipAnglesDeg', 0:0.1:14, @validateSlipAngles);
parser.addParameter('EnvelopeLoadsN', 100:10:2500, @validateNormalLoads);
parser.addParameter('SurfaceMu', [], @validateSurfaceMu);
parser.addParameter('TrackWidth', 1.21, @validatePositiveScalar);
parser.addParameter('CgHeight', 0.256, @validateNonnegativeScalar);
parser.addParameter('StaticFrontWeight', 0.5095, @validateUnitScalar);
parser.addParameter('LateralLoadTransferDistribution', [], ...
    @validateUnitScalarOrEmpty);
parser.addParameter('ReferenceMassKg', 264, @validateNonnegativeScalar);
parser.addParameter('OutputFile', defaultOutput, ...
    @(x) ischar(x) || isstring(x));
parser.addParameter('SavePlot', true, @validateScalarLogical);
parser.addParameter('ShowFigure', true, @validateScalarLogical);
parser.addParameter('CloseFigure', false, @validateScalarLogical);
parser.parse(varargin{:});
opts = parser.Results;
loadTransfer = loadTransferOptions(opts);

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
surfaceMu = opts.SurfaceMu;
if isempty(surfaceMu)
    surfaceMu = tire.surfaceMuReference;
end

tireFilePath = tire.tireConstants.tirFilePath;
envelope = buildTireEnvelope(tire, envelopeLoadsN, slipAnglesDeg, surfaceMu);
massKg = massGrid(massRange, opts.MassStepKg);
sweep = sweepMass(massKg, envelope, loadTransfer);
kneePoint = findKnee(sweep);

printReport(tireFilePath, surfaceMu, loadTransfer, massKg, sweep, ...
    kneePoint, opts.ReferenceMassKg);

visibleState = onOff(opts.ShowFigure, 'on', 'off');
outputFile = char(opts.OutputFile);
fig = [];
if opts.SavePlot
    fig = plotCurve(sweep, kneePoint, opts.ReferenceMassKg, tireFilePath, ...
        surfaceMu, loadTransfer, visibleState);
    writeFigureToFile(fig, outputFile);
end

result = struct( ...
    'tirFile', tireFilePath, ...
    'surfaceMu', surfaceMu, ...
    'loadTransfer', loadTransfer, ...
    'massKg', massKg, ...
    'sustainedG', sweep.sustainedG, ...
    'sustainedMps2', sweep.sustainedMps2, ...
    'benefitGPerKgSaved', sweep.benefitGPerKgSaved, ...
    'limitedBy', sweep.limitedBy, ...
    'cornerLoadsN', sweep.cornerLoadsN, ...
    'frontCapacityN', sweep.frontCapacityN, ...
    'rearCapacityN', sweep.rearCapacityN, ...
    'envelope', envelope, ...
    'kneePoint', kneePoint, ...
    'outputFile', outputFile, ...
    'figure', fig);

if opts.CloseFigure && ~isempty(fig)
    close(fig);
    result.figure = [];
end
end

function loadTransfer = loadTransferOptions(opts)
frontLoadTransferDistribution = opts.LateralLoadTransferDistribution;
if isempty(frontLoadTransferDistribution)
    frontLoadTransferDistribution = opts.StaticFrontWeight;
end

loadTransfer = struct( ...
    'trackWidth', opts.TrackWidth, ...
    'cgHeight', opts.CgHeight, ...
    'staticFrontWeight', opts.StaticFrontWeight, ...
    'frontLoadTransferDistribution', frontLoadTransferDistribution);
end

function envelope = buildTireEnvelope(tire, loadsN, slipAnglesDeg, surfaceMu)
% Peak lateral force vs normal load from a batched Pacejka sweep over slip
% angle. peakForce(Fz) = max_alpha |Fy|, with the slip angle that produces it.
Fy = computeLateralForceGrid(tire, loadsN, slipAnglesDeg, surfaceMu);
[peakForceN, slipIdx] = max(abs(Fy), [], 2);
peakForceN = peakForceN(:);
slipAtPeakDeg = slipAnglesDeg(slipIdx).';
envelope = struct( ...
    'loadsN', loadsN(:), ...
    'peakForceN', peakForceN, ...
    'slipAtPeakDeg', slipAtPeakDeg, ...
    'slipAnglesDeg', slipAnglesDeg, ...
    'lateralForceN', Fy);
end

function Fy = computeLateralForceGrid(tire, normalLoads, slipAnglesDeg, surfaceMu)
normalLoads = normalLoads(:);
Fy = zeros(numel(normalLoads), numel(slipAnglesDeg));
surfaceScale = tireSurfaceScale(tire, surfaceMu);
active = normalLoads > 0;
if ~any(active)
    return;
end

activeLoads = normalLoads(active);
alphas = deg2rad(slipAnglesDeg(:).');
[loadGrid, alphaGrid] = ndgrid(activeLoads, alphas);
nRows = numel(loadGrid);
FyActive = zeros(size(loadGrid));
maxRowsPerCall = 100000;

for startIdx = 1:maxRowsPerCall:nRows
    endIdx = min(startIdx + maxRowsPerCall - 1, nRows);
    idx = startIdx:endIdx;
    chunkLoads = loadGrid(idx);
    chunkAlphas = alphaGrid(idx);
    chunkLoads = chunkLoads(:);
    chunkAlphas = chunkAlphas(:);
    inputsMF = [ ...
        chunkLoads, ...
        zeros(numel(idx), 1), ...
        chunkAlphas, ...
        zeros(numel(idx), 1), ...
        zeros(numel(idx), 1), ...
        repmat(tire.tireConstants.refVelocity, numel(idx), 1), ...
        repmat(tire.tireConstants.nomPressure, numel(idx), 1)];
    % mfeval saturates and warns for very small vertical loads (its internal
    % lower limit sits near ~100 N). Those samples lie below any realistic
    % inside-tire load, so silence the expected warning here.
    warnState = warning('off', 'Solver:Limits:Exceeded');
    cleanups = onCleanup(@() warning(warnState));
    outputs = mfeval(tire.tireConstants.params, inputsMF, 111);
    clear cleanups;
    FyActive(idx) = -outputs(:, 2) * surfaceScale;
end

Fy(active, :) = FyActive;
end

function sweep = sweepMass(massKg, envelope, loadTransfer)
nMass = numel(massKg);
sustainedG = zeros(nMass, 1);
sustainedMps2 = zeros(nMass, 1);
cornerLoadsN = zeros(nMass, 4);
frontCapacityN = zeros(nMass, 1);
rearCapacityN = zeros(nMass, 1);
limitedBy = strings(nMass, 1);

for i = 1:nMass
    [sustainedG(i), detail] = solveSkidpad(massKg(i), envelope, loadTransfer);
    sustainedMps2(i) = sustainedG(i) * 9.80665;
    cornerLoadsN(i, :) = detail.cornerLoadsN(:).';
    frontCapacityN(i) = detail.frontCapacityN;
    rearCapacityN(i) = detail.rearCapacityN;
    limitedBy(i) = string(detail.limitedBy);
end

if nMass >= 2
    % sustainedG is solved by bisection to ~1e-15, but over this near-linear
    % range its real variation is in the ~4th significant digit, so the
    % numerical derivative is dominated by solver/float noise and reads as
    % spurious oscillation on the marginal-gain plot. The benefit (gain per
    % kg saved) is -dG/dKg; smooth that with a centered moving average and
    % round to suppress the residual float wiggles.
    dGdKg = gradient(sustainedG, massKg);
    benefitGPerKgSaved = -movingAverage(dGdKg, max(5, ceil(nMass / 20)));
    benefitGPerKgSaved = round(benefitGPerKgSaved, 5);
else
    benefitGPerKgSaved = NaN(nMass, 1);
end

sweep = struct( ...
    'massKg', massKg, ...
    'sustainedG', sustainedG, ...
    'sustainedMps2', sustainedMps2, ...
    'benefitGPerKgSaved', benefitGPerKgSaved, ...
    'limitedBy', limitedBy, ...
    'cornerLoadsN', cornerLoadsN, ...
    'frontCapacityN', frontCapacityN, ...
    'rearCapacityN', rearCapacityN);
end

function [ayG, detail] = solveSkidpad(massKg, envelope, loadTransfer)
% Bisection root-find on capacityG(m, ay) - ay = 0. The peak-force envelope
% is monotonically increasing in ay (more load transfer -> more outside
% capacity) while the demand ay rises linearly, so a unique crossing exists.
maxEqualLoadMu = max(envelope.peakForceN ./ max(envelope.loadsN, eps));
upper = max(0.5, 1.5 * maxEqualLoadMu);
while skidpadResidual(upper, massKg, envelope, loadTransfer) > 0 ...
        && upper < 5
    upper = upper * 1.5;
end

lower = 0;
for iter = 1:50 %#ok<NASGU>
    mid = 0.5 * (lower + upper);
    residual = skidpadResidual(mid, massKg, envelope, loadTransfer);
    if residual >= 0
        lower = mid;
    else
        upper = mid;
    end
end

ayG = lower;
detail = capacityAtAy(massKg, ayG, envelope, loadTransfer);
end

function residual = skidpadResidual(ayG, massKg, envelope, loadTransfer)
detail = capacityAtAy(massKg, ayG, envelope, loadTransfer);
residual = detail.capacityG - ayG;
end

function detail = capacityAtAy(massKg, ayG, envelope, loadTransfer)
g = 9.80665;
cornerLoads = bicycleCornerLoads(massKg, ayG, loadTransfer);
cornerCapacity = tireCapacityForLoads(cornerLoads, envelope);

frontCapacity = cornerCapacity(1) + cornerCapacity(2);
rearCapacity = cornerCapacity(3) + cornerCapacity(4);
frontFraction = loadTransfer.staticFrontWeight;
rearFraction = 1 - frontFraction;
frontLimitedTotal = frontCapacity / max(frontFraction, eps);
rearLimitedTotal = rearCapacity / max(rearFraction, eps);

if frontLimitedTotal <= rearLimitedTotal
    totalCapacity = frontLimitedTotal;
    limitedBy = 'front';
else
    totalCapacity = rearLimitedTotal;
    limitedBy = 'rear';
end

capacityG = totalCapacity / max(massKg * g, eps);

detail = struct( ...
    'capacityG', capacityG, ...
    'totalCapacityN', totalCapacity, ...
    'frontCapacityN', frontCapacity, ...
    'rearCapacityN', rearCapacity, ...
    'cornerCapacityN', cornerCapacity, ...
    'cornerLoadsN', cornerLoads, ...
    'limitedBy', limitedBy);
end

function loads = bicycleCornerLoads(massKg, ayG, loadTransfer)
% Simplified steady-state lateral load transfer (bicycle model). Corner
% order is [frontInside; frontOutside; rearInside; rearOutside]. Inside
% loads are floored at zero so inside-tire-lift does not generate negative
% capacity through extrapolation.
g = 9.80665;
W = massKg * g;
frontAxleStatic = W * loadTransfer.staticFrontWeight;
rearAxleStatic = W * (1 - loadTransfer.staticFrontWeight);
totalTransfer = W * ayG * loadTransfer.cgHeight / loadTransfer.trackWidth;
frontTransfer = totalTransfer * loadTransfer.frontLoadTransferDistribution;
rearTransfer = totalTransfer * (1 - loadTransfer.frontLoadTransferDistribution);

frontInside = max((frontAxleStatic - frontTransfer) / 2, 0);
frontOutside = frontAxleStatic - frontInside;
rearInside = max((rearAxleStatic - rearTransfer) / 2, 0);
rearOutside = rearAxleStatic - rearInside;
loads = [frontInside; frontOutside; rearInside; rearOutside];
end

function capacity = tireCapacityForLoads(loads, envelope)
capacity = zeros(size(loads));
loadsGrid = envelope.loadsN;
peakForce = envelope.peakForceN;
for i = 1:numel(loads)
    Fz = loads(i);
    if Fz <= 0
        capacity(i) = 0;
        continue;
    end
    capacity(i) = max(interp1(loadsGrid, peakForce, Fz, ...
        'linear', 'extrap'), 0);
end
end

function knee = findKnee(sweep)
% Kneedle elbow: the point of maximum perpendicular distance from the chord
% of the G-vs-mass curve (endpoints excluded). This is the point where
% returns from further weight savings start to diminish.
mass = sweep.massKg(:);
accel = sweep.sustainedG(:);

if isempty(mass)
    knee = emptyKnee();
    return;
elseif numel(mass) < 3
    knee = kneeFromIndex(sweep, 1, 'insufficient-samples');
    return;
end

x = normalizeRange(mass);
y = normalizeRange(accel);
p1 = [x(1), y(1)];
p2 = [x(end), y(end)];
lineVec = p2 - p1;
lineNorm = hypot(lineVec(1), lineVec(2));
if lineNorm <= eps
    knee = kneeFromIndex(sweep, ceil(numel(mass) / 2), 'knee');
    return;
end

n = numel(mass);
distance = zeros(n, 1);
for i = 1:n
    p = [x(i), y(i)];
    distance(i) = abs(lineVec(1) * (p1(2) - p(2)) - ...
        (p1(1) - p(1)) * lineVec(2)) / lineNorm;
end
distance([1, end]) = -Inf;
[~, idx] = max(distance);
if ~isfinite(distance(idx))
    idx = ceil(n / 2);
end
knee = kneeFromIndex(sweep, idx, 'knee');
end

function knee = kneeFromIndex(sweep, idx, method)
mass = sweep.massKg(:);
if numel(sweep.benefitGPerKgSaved) >= numel(mass)
    benefit = sweep.benefitGPerKgSaved(idx);
else
    benefit = NaN;
end
knee = struct( ...
    'method', method, ...
    'index', idx, ...
    'massKg', mass(idx), ...
    'accelG', sweep.sustainedG(idx), ...
    'accelMps2', sweep.sustainedMps2(idx), ...
    'benefitGPerKgSaved', benefit);
end

function knee = emptyKnee()
knee = struct( ...
    'method', 'empty', ...
    'index', NaN, ...
    'massKg', NaN, ...
    'accelG', NaN, ...
    'accelMps2', NaN, ...
    'benefitGPerKgSaved', NaN);
end

function massKg = massGrid(massRange, massStepKg)
massKg = (massRange(1):massStepKg:massRange(2)).';
if isempty(massKg)
    massKg = [massRange(1); massRange(2)];
elseif massKg(end) < massRange(2)
    massKg(end + 1, 1) = massRange(2);
end
massKg = unique(massKg, 'stable');
end

function values = normalizeRange(values)
values = values(:);
valueRange = max(values) - min(values);
if valueRange <= eps
    values = zeros(size(values));
else
    values = (values - min(values)) / valueRange;
end
end

function scale = tireSurfaceScale(tire, surfaceMu)
if isempty(surfaceMu) || ~isfinite(surfaceMu)
    surfaceMu = tire.surfaceMuReference;
end
scale = max(surfaceMu, 0) / max(tire.surfaceMuReference, eps);
end

function out = curveLinearity(massKg, sustainedG)
% Characterize how close the G-vs-mass curve is to a straight line. When the
% fit is nearly linear, a Kneedle knee is a weak/arbitrary pick and the
% honest takeaway is "each kg saved is worth about the same".
mass = massKg(:);
g = sustainedG(:);
out = struct('linearFitR2', NaN, 'isNearlyLinear', false);
finite = isfinite(mass) & isfinite(g);
mass = mass(finite);
g = g(finite);
if numel(mass) < 3
    return;
end
p = polyfit(mass, g, 1);
residuals = g - polyval(p, mass);
ssRes = sum(residuals.^2);
ssTot = sum((g - mean(g)).^2);
if ssTot > eps
    out.linearFitR2 = max(0, 1 - ssRes / ssTot);
else
    out.linearFitR2 = 1;
end
out.isNearlyLinear = out.linearFitR2 >= 0.995;
end

function m = meanFinite(values)
values = values(isfinite(values));
if isempty(values)
    m = NaN;
else
    m = mean(values);
end
end

function out = movingAverage(values, halfWindow)
% Centered moving average over a finite-only window. NaNs/Infs are excluded
% from each local mean but keep their position, so edges and gaps degrade
% gracefully rather than propagating NaN.
values = values(:);
n = numel(values);
out = zeros(n, 1);
for i = 1:n
    lo = max(1, i - halfWindow);
    hi = min(n, i + halfWindow);
    window = values(lo:hi);
    window = window(isfinite(window));
    if isempty(window)
        out(i) = values(i);
    else
        out(i) = mean(window);
    end
end
end

function out = smoothFinite(values)
% Round to 1e-6 g/kg (1e-3 milli-g/kg) after smoothing: fine enough to keep
% any real trend, coarse enough to flatten residual float wiggles.
out = round(values(:), 6);
end

function printReport(tireFilePath, surfaceMu, loadTransfer, massKg, sweep, ...
        knee, referenceMassKg)
g = 9.80665;
[~, tireName, tireExt] = fileparts(tireFilePath);
fprintf('\n=== Weight Savings Skidpad (bicycle model, aero neglected) ===\n');
fprintf('Tire file:           %s%s\n', tireName, tireExt);
fprintf('Surface mu:          %.3f\n', surfaceMu);
fprintf('Geometry:            track %.3f m, CG %.3f m, static front %.1f%%, front LLTD %.1f%%\n', ...
    loadTransfer.trackWidth, loadTransfer.cgHeight, ...
    100 * loadTransfer.staticFrontWeight, ...
    100 * loadTransfer.frontLoadTransferDistribution);
fprintf('Mass sweep:          %.1f to %.1f kg (%d points)\n', ...
    min(massKg), max(massKg), numel(massKg));

[refG, ~, refLimit] = sustainedAtMass(referenceMassKg, massKg, sweep);
if isfinite(refG)
    fprintf('Reference mass:      %.1f kg -> %.3f g sustained, limited by %s\n', ...
        referenceMassKg, refG, refLimit);
end

fprintf('>> Diminishing-return mass: %.1f kg (%.3f g', ...
    knee.massKg, knee.accelG);
if isfinite(knee.benefitGPerKgSaved)
    fprintf(', %.1f milli-g/kg saved', 1000 * knee.benefitGPerKgSaved);
end
fprintf(', %s)\n', knee.method);

linearity = curveLinearity(massKg, sweep.sustainedG);
if linearity.isNearlyLinear
    fprintf('   NOTE: the G-vs-mass curve is nearly linear over this range (R^2 %.3f vs a straight line).\n', ...
        linearity.linearFitR2);
    fprintf('   The knee is therefore weak; each kg saved is worth roughly the same (~%.2f milli-g/kg).\n', ...
        1000 * meanFinite(sweep.benefitGPerKgSaved));
else
    fprintf('   Curve curvature is significant (R^2 %.3f vs a straight line); the knee marks the elbow.\n', ...
        linearity.linearFitR2);
end

[gMin, idxMin, limitMin] = sustainedAtMass(min(massKg), massKg, sweep);
[gMax, idxMax, limitMax] = sustainedAtMass(max(massKg), massKg, sweep);
if isfinite(gMin) && isfinite(gMax)
    fprintf('   At %.0f kg: %.3f g', min(massKg), gMin);
    if idxMin >= 1 && idxMin <= numel(sweep.benefitGPerKgSaved) ...
            && isfinite(sweep.benefitGPerKgSaved(idxMin))
        fprintf(', %.1f milli-g/kg', 1000 * sweep.benefitGPerKgSaved(idxMin));
    end
    fprintf(' (%s)   |   At %.0f kg: %.3f g', limitMin, max(massKg), gMax);
    if idxMax >= 1 && idxMax <= numel(sweep.benefitGPerKgSaved) ...
            && isfinite(sweep.benefitGPerKgSaved(idxMax))
        fprintf(', %.1f milli-g/kg', 1000 * sweep.benefitGPerKgSaved(idxMax));
    end
    fprintf(' (%s)\n', limitMax);
end
fprintf('Aero downforce is neglected; only tire load sensitivity couples mass to grip.\n');
end

function [gVal, idx, limit] = sustainedAtMass(targetMassKg, massKg, sweep)
idx = [];
[~, idx] = min(abs(massKg(:) - targetMassKg));
if isempty(idx) || ~isfinite(idx)
    gVal = NaN;
    limit = '';
    return;
end
gVal = sweep.sustainedG(idx);
limit = char(sweep.limitedBy(idx));
end

function fig = plotCurve(sweep, knee, referenceMassKg, tireFilePath, ...
        surfaceMu, loadTransfer, visibleState)
fig = figure('Name', 'Weight savings diminishing returns', ...
    'Color', 'w', 'Visible', visibleState);
fig.Position = [120 80 1100 780];
set(fig, ...
    'DefaultTextColor', 'k', ...
    'DefaultAxesColor', 'w', ...
    'DefaultAxesXColor', 'k', ...
    'DefaultAxesYColor', 'k', ...
    'DefaultLegendColor', 'w', ...
    'DefaultLegendTextColor', 'k');

layout = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'loose');
mass = sweep.massKg;
accel = sweep.sustainedG;
benefitMilliGPerKg = 1000 * sweep.benefitGPerKgSaved;
kneeColor = [0.80 0.12 0.10];
refColor = [0.45 0.45 0.45];

axAccel = nexttile(layout);
plot(axAccel, mass, accel, 'Color', [0.07 0.29 0.67], ...
    'LineWidth', 1.8, 'DisplayName', 'Sustained a_y');
hold(axAccel, 'on');
if isfinite(referenceMassKg)
    xline(axAccel, referenceMassKg, ':', 'Color', refColor, ...
        'LineWidth', 1.1, 'HandleVisibility', 'off');
end
if isfinite(knee.massKg)
    xline(axAccel, knee.massKg, '--', 'Color', kneeColor, ...
        'LineWidth', 1.1, 'HandleVisibility', 'off');
    plot(axAccel, knee.massKg, knee.accelG, 'o', 'MarkerSize', 8, ...
        'LineWidth', 1.5, 'MarkerFaceColor', kneeColor, ...
        'MarkerEdgeColor', 'w', 'DisplayName', 'Diminishing returns');
end
hold(axAccel, 'off');
styleAxis(axAccel);
grid(axAccel, 'on');
ylabel(axAccel, 'Sustained a_y [g]');
title(axAccel, 'Max steady-state skidpad G vs vehicle mass');
legend(axAccel, 'Location', 'best');

axBenefit = nexttile(layout);
plot(axBenefit, mass, benefitMilliGPerKg, 'Color', [0.10 0.50 0.28], ...
    'LineWidth', 1.6, 'DisplayName', 'Marginal gain');
hold(axBenefit, 'on');
yline(axBenefit, 0, ':', 'Color', refColor, 'HandleVisibility', 'off');
if isfinite(referenceMassKg)
    xline(axBenefit, referenceMassKg, ':', 'Color', refColor, ...
        'LineWidth', 1.1, 'HandleVisibility', 'off');
end
if isfinite(knee.massKg)
    xline(axBenefit, knee.massKg, '--', 'Color', kneeColor, ...
        'LineWidth', 1.1, 'HandleVisibility', 'off');
    if isfinite(knee.benefitGPerKgSaved)
        plot(axBenefit, knee.massKg, 1000 * knee.benefitGPerKgSaved, ...
            'o', 'MarkerSize', 8, 'LineWidth', 1.5, ...
            'MarkerFaceColor', kneeColor, 'MarkerEdgeColor', 'w', ...
            'DisplayName', 'Diminishing returns');
    end
end
hold(axBenefit, 'off');
styleAxis(axBenefit);
grid(axBenefit, 'on');
xlabel(axBenefit, 'Vehicle mass [kg]');
ylabel(axBenefit, 'Gain [milli-g / kg saved]');
title(axBenefit, 'Marginal G gained per kg of mass removed');
legend(axBenefit, 'Location', 'best');

linkaxes([axAccel, axBenefit], 'x');
[~, tireName, tireExt] = fileparts(tireFilePath);
plotTitle = sgtitle(sprintf( ...
    ['Weight savings diminishing returns | %s%s | surface mu %.3f | ' ...
     'track %.2f m, CG %.2f m, front %.0f%%, LLTD %.0f%%'], ...
    tireName, tireExt, surfaceMu, loadTransfer.trackWidth, ...
    loadTransfer.cgHeight, 100 * loadTransfer.staticFrontWeight, ...
    100 * loadTransfer.frontLoadTransferDistribution), ...
    'Interpreter', 'none');
set(plotTitle, 'Color', 'k', 'FontWeight', 'bold');
end

function styleAxis(ax)
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

function writeFigureToFile(fig, outputFile)
[folder, ~, ~] = fileparts(outputFile);
if ~isempty(folder) && ~exist(folder, 'dir')
    mkdir(folder);
end

if exist('exportgraphics', 'file') == 2
    exportgraphics(fig, outputFile, 'Resolution', 160, ...
        'BackgroundColor', 'white');
else
    saveas(fig, outputFile);
end
end

function value = onOff(flag, trueValue, falseValue)
if flag
    value = trueValue;
else
    value = falseValue;
end
end

function tf = isOptionName(value, optionNames)
tf = false;
if ~(ischar(value) || (isstring(value) && isscalar(value)))
    return;
end
tf = any(strcmpi(char(value), optionNames));
end

function tf = validateMassRange(value)
validateattributes(value, {'numeric'}, ...
    {'vector', 'real', 'finite', 'positive'}, mfilename, 'MassRangeKg');
tf = true;
end

function tf = validateSlipAngles(value)
validateattributes(value, {'numeric'}, ...
    {'vector', 'real', 'finite'}, mfilename, 'EnvelopeSlipAnglesDeg');
tf = true;
end

function tf = validateNormalLoads(value)
validateattributes(value, {'numeric'}, ...
    {'vector', 'real', 'finite', 'nonnegative'}, mfilename, 'EnvelopeLoadsN');
tf = true;
end

function tf = validateSurfaceMu(value)
if isempty(value)
    tf = true;
    return;
end
validateattributes(value, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'nonnegative'}, mfilename, 'SurfaceMu');
tf = true;
end

function tf = validatePositiveScalar(value)
validateattributes(value, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'positive'}, mfilename);
tf = true;
end

function tf = validateNonnegativeScalar(value)
validateattributes(value, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'nonnegative'}, mfilename);
tf = true;
end

function tf = validateUnitScalar(value)
validateattributes(value, {'numeric'}, ...
    {'scalar', 'real', 'finite', '>=', 0, '<=', 1}, mfilename);
tf = true;
end

function tf = validateUnitScalarOrEmpty(value)
if isempty(value)
    tf = true;
    return;
end
tf = validateUnitScalar(value);
end

function tf = validateScalarLogical(value)
validateattributes(value, {'logical', 'numeric'}, ...
    {'scalar'}, mfilename);
tf = true;
end
