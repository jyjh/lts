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
parser.addParameter('MassRangeKg', [180 280], @validateMassRange);
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

for i = 1:nMass
    [sustainedG(i), detail] = solveSkidpad(massKg(i), envelope, loadTransfer);
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
    benefitSPerKgSaved = smoothMarginal(dTdm, nMass);
    dGdKg = gradient(sustainedG, massKg);
    benefitGPerKgSaved = -smoothMarginal(dGdKg, nMass);
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
    'rearCapacityN', rearCapacityN);
end

function out = smoothMarginal(derivative, nMass)
% Numerical derivative of a near-constant curve is float-noise dominated.
% Centered moving average + coarse round to suppress spurious oscillation.
out = movingAverage(derivative, max(5, ceil(nMass / 20)));
out = round(out, 6);
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
% Kneedle elbow on the LAP-TIME-vs-mass curve (endpoints excluded). Lap time
% is the honest diminishing-returns metric: the 1/sqrt(G) compression in T
% means each kg saved shaves less lap time off as the car gets lighter,
% whereas raw G itself has accelerating returns to lightness.
mass = sweep.massKg(:);
lapTime = sweep.lapTimeS(:);

if isempty(mass)
    knee = emptyKnee();
    return;
elseif numel(mass) < 3
    knee = kneeFromIndex(sweep, 1, 'insufficient-samples');
    return;
end

x = normalizeRange(mass);
y = normalizeRange(lapTime);
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
if numel(sweep.benefitSPerKgSaved) >= numel(mass)
    benefitS = sweep.benefitSPerKgSaved(idx);
else
    benefitS = NaN;
end
if numel(sweep.benefitGPerKgSaved) >= numel(mass)
    benefitG = sweep.benefitGPerKgSaved(idx);
else
    benefitG = NaN;
end
knee = struct( ...
    'method', method, ...
    'index', idx, ...
    'massKg', mass(idx), ...
    'accelG', sweep.sustainedG(idx), ...
    'accelMps2', sweep.sustainedMps2(idx), ...
    'lapTimeS', sweep.lapTimeS(idx), ...
    'benefitSPerKgSaved', benefitS, ...
    'benefitGPerKgSaved', benefitG);
end

function knee = emptyKnee()
knee = struct( ...
    'method', 'empty', ...
    'index', NaN, ...
    'massKg', NaN, ...
    'accelG', NaN, ...
    'accelMps2', NaN, ...
    'lapTimeS', NaN, ...
    'benefitSPerKgSaved', NaN, ...
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

function out = curveLinearity(massKg, signal)
% Fit a quadratic to (mass, signal) and report the quadratic coefficient and
% a linear R^2. The quadratic sign tells us the curvature direction: for a
% G-vs-mass sweep, c >= 0 means ACCELERATING returns to lightness (lighter
% pays off progressively more per kg); for a lap-time sweep, c >= 0 means
% DIMINISHING returns (each kg saved shaves less time off as the car gets
% lighter). The curvature is the honest signal here, not the knee pick.
mass = massKg(:);
sig = signal(:);
out = struct('quadCoeff', NaN, 'linearFitR2', NaN);
finite = isfinite(mass) & isfinite(sig);
mass = mass(finite);
sig = sig(finite);
if numel(mass) < 3
    return;
end
p2 = polyfit(mass, sig, 2);
out.quadCoeff = p2(1);
p1 = polyfit(mass, sig, 1);
residuals = sig - polyval(p1, mass);
ssRes = sum(residuals.^2);
ssTot = sum((sig - mean(sig)).^2);
if ssTot > eps
    out.linearFitR2 = max(0, 1 - ssRes / ssTot);
else
    out.linearFitR2 = 1;
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

function printReport(tireFilePath, surfaceMu, loadTransfer, massKg, sweep, ...
        knee, referenceMassKg)
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

[refIdx, refLimit] = indexAtMass(referenceMassKg, massKg, sweep);
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

gCurv = curveLinearity(massKg, sweep.sustainedG);
tCurv = curveLinearity(massKg, sweep.lapTimeS);
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
end

function [idx, limit] = indexAtMass(targetMassKg, massKg, sweep)
idx = NaN;
[~, found] = min(abs(massKg(:) - targetMassKg));
if isempty(found) || ~isfinite(found)
    limit = '';
    return;
end
idx = found;
limit = char(sweep.limitedBy(idx));
end

function fig = plotCurve(sweep, knee, referenceMassKg, tireFilePath, ...
        surfaceMu, loadTransfer, visibleState)
fig = figure('Name', 'Weight savings diminishing returns (lap time)', ...
    'Color', 'w', 'Visible', visibleState);
fig.Position = [120 60 1120 900];
set(fig, ...
    'DefaultTextColor', 'k', ...
    'DefaultAxesColor', 'w', ...
    'DefaultAxesXColor', 'k', ...
    'DefaultAxesYColor', 'k', ...
    'DefaultLegendColor', 'w', ...
    'DefaultLegendTextColor', 'k');

layout = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'loose');
mass = sweep.massKg;
accel = sweep.sustainedG;
lapTime = sweep.lapTimeS;
benefitMsPerKg = 1000 * sweep.benefitSPerKgSaved;
blue = [0.07 0.29 0.67];
green = [0.10 0.50 0.28];
kneeColor = [0.80 0.12 0.10];
refColor = [0.45 0.45 0.45];

% --- Tile 1: sustained G vs mass (accelerating returns to lightness) ---
axAccel = nexttile(layout);
plot(axAccel, mass, accel, 'Color', blue, 'LineWidth', 1.8, ...
    'DisplayName', 'Sustained a_y');
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
        'MarkerEdgeColor', 'w', 'HandleVisibility', 'off');
end
hold(axAccel, 'off');
styleAxis(axAccel);
grid(axAccel, 'on');
ylabel(axAccel, 'Sustained a_y [g]');
title(axAccel, 'Skidpad G vs mass (convex: lighter pays off progressively more per kg)');
legend(axAccel, 'Location', 'best');

% --- Tile 2: lap time vs mass (where diminishing returns live) ---
axLap = nexttile(layout);
plot(axLap, mass, lapTime, 'Color', blue, 'LineWidth', 1.8, ...
    'DisplayName', 'Lap time');
hold(axLap, 'on');
if isfinite(referenceMassKg)
    xline(axLap, referenceMassKg, ':', 'Color', refColor, ...
        'LineWidth', 1.1, 'HandleVisibility', 'off');
end
if isfinite(knee.massKg) && isfinite(knee.lapTimeS)
    xline(axLap, knee.massKg, '--', 'Color', kneeColor, ...
        'LineWidth', 1.1, 'HandleVisibility', 'off');
    plot(axLap, knee.massKg, knee.lapTimeS, 'o', 'MarkerSize', 8, ...
        'LineWidth', 1.5, 'MarkerFaceColor', kneeColor, ...
        'MarkerEdgeColor', 'w', 'DisplayName', 'Diminishing-returns knee');
end
hold(axLap, 'off');
styleAxis(axLap);
grid(axLap, 'on');
ylabel(axLap, 'FSAE skidpad lap time [s]');
title(axLap, 'Lap time vs mass (concave: each kg saved shaves less time off as car gets lighter)');
legend(axLap, 'Location', 'best');

% --- Tile 3: marginal lap-time benefit per kg saved (the key panel) ---
axBenefit = nexttile(layout);
plot(axBenefit, mass, benefitMsPerKg, 'Color', green, 'LineWidth', 1.6, ...
    'DisplayName', 'Marginal lap-time gain');
hold(axBenefit, 'on');
yline(axBenefit, 0, ':', 'Color', refColor, 'HandleVisibility', 'off');
if isfinite(referenceMassKg)
    xline(axBenefit, referenceMassKg, ':', 'Color', refColor, ...
        'LineWidth', 1.1, 'HandleVisibility', 'off');
end
if isfinite(knee.massKg) && isfinite(knee.benefitSPerKgSaved)
    xline(axBenefit, knee.massKg, '--', 'Color', kneeColor, ...
        'LineWidth', 1.1, 'HandleVisibility', 'off');
    plot(axBenefit, knee.massKg, 1000 * knee.benefitSPerKgSaved, ...
        'o', 'MarkerSize', 8, 'LineWidth', 1.5, ...
        'MarkerFaceColor', kneeColor, 'MarkerEdgeColor', 'w', ...
        'DisplayName', 'Diminishing-returns knee');
end
hold(axBenefit, 'off');
styleAxis(axBenefit);
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
