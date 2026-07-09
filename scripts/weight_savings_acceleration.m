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

defaultTirFile = '43105_18x7.5_10_R25B_7_theoretical.tir';
optionNames = {'MassRangeKg', 'MassStepKg', 'EnvelopeSlipRatios', ...
    'EnvelopeLoadsN', 'SurfaceMu', 'Wheelbase', 'CgHeight', ...
    'StaticFrontWeight', 'DistanceM', 'ReferenceMassKg', ...
    'CgDropPerKgSavedM', 'CoupledCg', 'UsePowertrainLimit', ...
    'PowertrainMatFile', 'PowertrainEfficiency', 'MotorRotorInertia', ...
    'OutputFile', 'SavePlot', 'ShowFigure', 'CloseFigure'};

if nargin < 1 || isempty(tirFile)
    tirFile = defaultTirFile;
elseif isOptionName(tirFile, optionNames)
    varargin = [{tirFile}, varargin];
    tirFile = defaultTirFile;
end

scriptDir = fileparts(mfilename('fullpath'));
repoRoot = fileparts(scriptDir);
addpath(fullfile(repoRoot, 'src'));

defaultOutput = fullfile(repoRoot, 'exports', ...
    'weight_savings_acceleration.png');

parser = inputParser;
parser.FunctionName = 'weight_savings_acceleration';
parser.addParameter('MassRangeKg', [180 280], @validateMassRange);
parser.addParameter('MassStepKg', 0.5, @validatePositiveScalar);
parser.addParameter('EnvelopeSlipRatios', linspace(0, 1.0, 161), ...
    @validateSlipRatios);
parser.addParameter('EnvelopeLoadsN', 100:10:2500, @validateNormalLoads);
parser.addParameter('SurfaceMu', [], @validateSurfaceMu);
parser.addParameter('Wheelbase', 1.528, @validatePositiveScalar);
parser.addParameter('CgHeight', 0.256, @validateNonnegativeScalar);
parser.addParameter('StaticFrontWeight', 0.5095, @validateUnitScalar);
parser.addParameter('DistanceM', 75.0, @validatePositiveScalar);
parser.addParameter('ReferenceMassKg', 264, @validateNonnegativeScalar);
parser.addParameter('CgDropPerKgSavedM', 0.001, @validateNonnegativeScalar);
parser.addParameter('CoupledCg', false, @validateScalarLogical);
parser.addParameter('UsePowertrainLimit', false, @validateScalarLogical);
parser.addParameter('PowertrainMatFile', '', @(x) ischar(x) || isstring(x));
parser.addParameter('PowertrainEfficiency', 0.90, ...
    @validateUnitScalarOrEmpty);
parser.addParameter('MotorRotorInertia', 0.07, @validateNonnegativeScalar);
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
surfaceMu = opts.SurfaceMu;
if isempty(surfaceMu)
    surfaceMu = tire.surfaceMuReference;
end

powertrain = [];
if opts.UsePowertrainLimit
    powertrain = buildPowertrain(opts);
end

tireFilePath = tire.tireConstants.tirFilePath;
envelope = buildLongitudinalEnvelope(tire, envelopeLoadsN, slipRatios, ...
    surfaceMu);
massKg = massGrid(massRange, opts.MassStepKg);
sweep = sweepMass(massKg, envelope, loadTransfer, opts.DistanceM, ...
    powertrain);
kneePoint = findKnee(sweep);

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
kneePointCoupled = emptyKnee();
coupledCgHeightM = [];
if opts.CoupledCg
    coupledTransfer = loadTransfer;
    coupledTransfer.cgHeight = coupledCgHeight(massKg, ...
        loadTransfer.cgHeight, opts.ReferenceMassKg, opts.CgDropPerKgSavedM);
    coupledCgHeightM = coupledTransfer.cgHeight;
    sweepCoupled = sweepMass(massKg, envelope, coupledTransfer, ...
        opts.DistanceM, powertrain);
    kneePointCoupled = findKnee(sweepCoupled);
end

printReport(tireFilePath, surfaceMu, loadTransfer, massKg, sweep, ...
    kneePoint, opts.ReferenceMassKg, opts.DistanceM, ...
    opts.CgDropPerKgSavedM, sweepCoupled, kneePointCoupled, powertrain);

visibleState = onOff(opts.ShowFigure, 'on', 'off');
outputFile = char(opts.OutputFile);
fig = [];
figCoupled = [];
if opts.SavePlot
    fig = plotCurve(sweep, kneePoint, opts.ReferenceMassKg, opts.DistanceM, ...
        tireFilePath, surfaceMu, loadTransfer, visibleState, powertrain);
    writeFigureToFile(fig, outputFile);
    if opts.CoupledCg
        coupledFile = coupledOutputFile(outputFile);
        figCoupled = plotCoupledCgCurve(sweep, kneePoint, sweepCoupled, ...
            kneePointCoupled, opts.ReferenceMassKg, opts.CgDropPerKgSavedM, ...
            opts.DistanceM, tireFilePath, surfaceMu, loadTransfer, ...
            visibleState, powertrain);
        writeFigureToFile(figCoupled, coupledFile);
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

function loadTransfer = loadTransferOptions(opts)
loadTransfer = struct( ...
    'wheelbase', opts.Wheelbase, ...
    'cgHeight', opts.CgHeight, ...
    'staticFrontWeight', opts.StaticFrontWeight);
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

function envelope = buildLongitudinalEnvelope(tire, loadsN, slipRatios, ...
        surfaceMu)
% Peak longitudinal force vs normal load from a batched Pacejka sweep over
% slip ratio (pure longitudinal: alpha = gamma = phit = 0). peakForce(Fz) =
% max_kappa |Fx|, with the slip ratio that produces it.
Fx = computeLongitudinalForceGrid(tire, loadsN, slipRatios, surfaceMu);
[peakForceN, slipIdx] = max(abs(Fx), [], 2);
peakForceN = peakForceN(:);
kappaAtPeak = slipRatios(slipIdx).';
envelope = struct( ...
    'loadsN', loadsN(:), ...
    'peakForceN', peakForceN, ...
    'kappaAtPeak', kappaAtPeak, ...
    'slipRatios', slipRatios, ...
    'longitudinalForceN', Fx);
end

function Fx = computeLongitudinalForceGrid(tire, normalLoads, slipRatios, ...
        surfaceMu)
normalLoads = normalLoads(:);
Fx = zeros(numel(normalLoads), numel(slipRatios));
surfaceScale = tireSurfaceScale(tire, surfaceMu);
active = normalLoads > 0;
if ~any(active)
    return;
end

activeLoads = normalLoads(active);
kappas = slipRatios(:).';
[loadGrid, kappaGrid] = ndgrid(activeLoads, kappas);
nRows = numel(loadGrid);
FxActive = zeros(size(loadGrid));
maxRowsPerCall = 100000;

for startIdx = 1:maxRowsPerCall:nRows
    endIdx = min(startIdx + maxRowsPerCall - 1, nRows);
    idx = startIdx:endIdx;
    chunkLoads = loadGrid(idx);
    chunkKappas = kappaGrid(idx);
    chunkLoads = chunkLoads(:);
    chunkKappas = chunkKappas(:);
    % mfeval input columns: [Fz, kappa, alpha, gamma, phit, Vx, P]. Pure
    % longitudinal -> alpha = gamma = phit = 0. Output column 1 = Fx.
    inputsMF = [ ...
        chunkLoads, ...
        chunkKappas, ...
        zeros(numel(idx), 1), ...
        zeros(numel(idx), 1), ...
        zeros(numel(idx), 1), ...
        repmat(tire.tireConstants.refVelocity, numel(idx), 1), ...
        repmat(tire.tireConstants.nomPressure, numel(idx), 1)];
    warnState = warning('off', 'Solver:Limits:Exceeded');
    cleanups = onCleanup(@() warning(warnState));
    outputs = mfeval(tire.tireConstants.params, inputsMF, 111);
    clear cleanups;
    FxActive(idx) = outputs(:, 1) * surfaceScale;
end

Fx(active, :) = FxActive;
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
    benefitSPerKgSaved = smoothMarginal(dTdm, nMass);
    dGdKg = gradient(launchG, massKg);
    benefitGPerKgSaved = -smoothMarginal(dGdKg, nMass);
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
g = 9.80665;
ax = 0;
rearNormalN = 0;
rearPeakMu = 0;
FtractionRear = 0;
Fmotor = Inf;
Fdrive = 0;

for iter = 1:12 %#ok<NASGU>
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

function knee = findKnee(sweep)
% Kneedle elbow on the TIME-vs-mass curve (endpoints excluded). Time is the
% honest diminishing-returns metric: the 1/sqrt(ax) compression means each kg
% saved shaves less time off as the car gets lighter, whereas raw launch g
% itself has accelerating returns to lightness.
mass = sweep.massKg(:);
eventTime = sweep.timeToDistanceS(:);

if isempty(mass)
    knee = emptyKnee();
    return;
elseif numel(mass) < 3
    knee = kneeFromIndex(sweep, 1, 'insufficient-samples');
    return;
end

x = normalizeRange(mass);
y = normalizeRange(eventTime);
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
    'launchG', sweep.launchG(idx), ...
    'launchAxMps2', sweep.launchAxMps2(idx), ...
    'timeToDistanceS', sweep.timeToDistanceS(idx), ...
    'benefitSPerKgSaved', benefitS, ...
    'benefitGPerKgSaved', benefitG);
end

function knee = emptyKnee()
knee = struct( ...
    'method', 'empty', ...
    'index', NaN, ...
    'massKg', NaN, ...
    'launchG', NaN, ...
    'launchAxMps2', NaN, ...
    'timeToDistanceS', NaN, ...
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
% a linear R^2. For a launch-g sweep, c >= 0 means ACCELERATING returns to
% lightness (lighter pays off progressively more per kg); for a time sweep,
% c >= 0 means DIMINISHING returns (each kg saved shaves less time off as the
% car gets lighter).
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

function out = smoothMarginal(derivative, nMass)
% Numerical derivative of a near-constant curve is float-noise dominated.
% Centered moving average + coarse round to suppress spurious oscillation.
out = movingAverage(derivative, max(5, ceil(nMass / 20)));
out = round(out, 6);
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

[refIdx, refLimit] = indexAtMass(referenceMassKg, massKg, sweep);
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

gCurv = curveLinearity(massKg, sweep.launchG);
tCurv = curveLinearity(massKg, sweep.timeToDistanceS);
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

% --- Optional coupled-CG sweep: 1 mm of CG drop per kg saved, anchored at
% the reference mass. NOTE the sign flip vs skidpad: for RWD launch, lowering
% the CG reduces rear load transfer, so the driven axle is LESS planted and
% peak traction DROPS. The coupled curve is therefore WORSE (slower) than the
% fixed-CG curve at low mass -- the honest trade-off between a low CG and a
% traction-friendly launch. Skipped entirely when CoupledCg is off. ---
if ~isempty(sweepCoupled)
fprintf('\n--- Coupled CG (%.1f mm drop per kg saved, anchored at %.1f kg) ---\n', ...
    1000 * cgDropPerKgSavedM, referenceMassKg);
if isfinite(kneeCoupled.massKg)
    fprintf('>> Coupled diminishing-return mass: %.1f kg', kneeCoupled.massKg);
    if isfinite(kneeCoupled.timeToDistanceS)
        fprintf(' (%.3f s, %.3f g', kneeCoupled.timeToDistanceS, ...
            kneeCoupled.launchG);
    end
    if isfinite(kneeCoupled.benefitSPerKgSaved)
        fprintf(', %.4f s/kg saved', kneeCoupled.benefitSPerKgSaved);
    end
    fprintf(', %s)\n', kneeCoupled.method);
else
    fprintf('>> Coupled diminishing-return mass: not available (too few samples)\n');
end
if isfinite(knee.massKg) && isfinite(kneeCoupled.massKg)
    deltaKg = kneeCoupled.massKg - knee.massKg;
    if abs(deltaKg) < 0.05
        fprintf('   Coupled-CG knee is unchanged from the fixed-CG knee (%.1f kg).\n', ...
            knee.massKg);
    elseif deltaKg > 0
        fprintf('   Coupled-CG knee shifts HEAVIER by %.1f kg (%.1f -> %.1f kg).\n', ...
            deltaKg, knee.massKg, kneeCoupled.massKg);
    else
        fprintf('   Coupled-CG knee shifts LIGHTER by %.1f kg (%.1f -> %.1f kg).\n', ...
            abs(deltaKg), knee.massKg, kneeCoupled.massKg);
    end
end
tCurvCoupled = curveLinearity(massKg, sweepCoupled.timeToDistanceS);
fprintf('   Coupled time: quad coeff %+.3e  (>=0 = diminishing returns as car gets lighter)\n', ...
    tCurvCoupled.quadCoeff);
endpts = [1, numel(massKg)];
for k = 1:numel(endpts)
    i = endpts(k);
    fprintf('   Coupled at %.0f kg (CG %.0f mm): %.3f g, %.3f s', ...
        massKg(i), 1000 * sweepCoupled.cgHeightM(i), ...
        sweepCoupled.launchG(i), sweepCoupled.timeToDistanceS(i));
    if isfinite(sweepCoupled.benefitSPerKgSaved(i))
        fprintf(', %.4f s/kg', sweepCoupled.benefitSPerKgSaved(i));
    end
    fprintf(' (%s)\n', sweepCoupled.limitedBy(i));
end
end
end

function label = limitModeLabel(powertrain)
if isempty(powertrain)
    label = 'grip-limited (powertrain cap off)';
else
    label = 'grip- or power-limited (powertrain cap on)';
end
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

function fig = plotCurve(sweep, knee, referenceMassKg, distanceM, ...
        tireFilePath, surfaceMu, loadTransfer, visibleState, powertrain)
fig = figure('Name', 'Weight savings diminishing returns (acceleration)', ...
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
launchG = sweep.launchG;
eventTime = sweep.timeToDistanceS;
benefitMsPerKg = 1000 * sweep.benefitSPerKgSaved;
blue = [0.07 0.29 0.67];
green = [0.10 0.50 0.28];
kneeColor = [0.80 0.12 0.10];
refColor = [0.45 0.45 0.45];

% --- Tile 1: launch g vs mass (accelerating returns to lightness) ---
axAccel = nexttile(layout);
plot(axAccel, mass, launchG, 'Color', blue, 'LineWidth', 1.8, ...
    'DisplayName', 'Launch a_x');
hold(axAccel, 'on');
if isfinite(referenceMassKg)
    xline(axAccel, referenceMassKg, ':', 'Color', refColor, ...
        'LineWidth', 1.1, 'HandleVisibility', 'off');
end
if isfinite(knee.massKg)
    xline(axAccel, knee.massKg, '--', 'Color', kneeColor, ...
        'LineWidth', 1.1, 'HandleVisibility', 'off');
    plot(axAccel, knee.massKg, knee.launchG, 'o', 'MarkerSize', 8, ...
        'LineWidth', 1.5, 'MarkerFaceColor', kneeColor, ...
        'MarkerEdgeColor', 'w', 'HandleVisibility', 'off');
end
hold(axAccel, 'off');
styleAxis(axAccel);
grid(axAccel, 'on');
ylabel(axAccel, 'Launch a_x [g]');
title(axAccel, 'Grip-limited launch g vs mass (convex: lighter pays off progressively more per kg)');
legend(axAccel, 'Location', 'best');

% --- Tile 2: 75 m time vs mass (where diminishing returns live) ---
axTime = nexttile(layout);
plot(axTime, mass, eventTime, 'Color', blue, 'LineWidth', 1.8, ...
    'DisplayName', sprintf('%.0f m time', distanceM));
hold(axTime, 'on');
if isfinite(referenceMassKg)
    xline(axTime, referenceMassKg, ':', 'Color', refColor, ...
        'LineWidth', 1.1, 'HandleVisibility', 'off');
end
if isfinite(knee.massKg) && isfinite(knee.timeToDistanceS)
    xline(axTime, knee.massKg, '--', 'Color', kneeColor, ...
        'LineWidth', 1.1, 'HandleVisibility', 'off');
    plot(axTime, knee.massKg, knee.timeToDistanceS, 'o', 'MarkerSize', 8, ...
        'LineWidth', 1.5, 'MarkerFaceColor', kneeColor, ...
        'MarkerEdgeColor', 'w', 'DisplayName', 'Diminishing-returns knee');
end
hold(axTime, 'off');
styleAxis(axTime);
grid(axTime, 'on');
ylabel(axTime, sprintf('Time to %.0f m [s]', distanceM));
title(axTime, 'Acceleration time vs mass (concave: each kg saved shaves less time off as car gets lighter)');
legend(axTime, 'Location', 'best');

% --- Tile 3: marginal time benefit per kg saved (the key panel) ---
axBenefit = nexttile(layout);
plot(axBenefit, mass, benefitMsPerKg, 'Color', green, 'LineWidth', 1.6, ...
    'DisplayName', 'Marginal time gain');
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
fig = figure('Name', 'Weight savings: fixed CG vs coupled CG (acceleration)', ...
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
launchG = sweep.launchG;
launchGCoupled = sweepCoupled.launchG;
eventTime = sweep.timeToDistanceS;
eventTimeCoupled = sweepCoupled.timeToDistanceS;
benefitMsPerKg = 1000 * sweep.benefitSPerKgSaved;
benefitMsPerKgCoupled = 1000 * sweepCoupled.benefitSPerKgSaved;
blue = [0.07 0.29 0.67];
orange = [0.95 0.55 0.10];
green = [0.10 0.50 0.28];
teal = [0.00 0.55 0.55];
kneeColor = [0.80 0.12 0.10];
kneeCoupledColor = [0.55 0.20 0.65];
refColor = [0.45 0.45 0.45];

% --- Tile 1: launch g vs mass ---
axAccel = nexttile(layout);
plot(axAccel, mass, launchG, 'Color', blue, 'LineWidth', 1.8, ...
    'DisplayName', 'Launch a_x (fixed CG)');
hold(axAccel, 'on');
plot(axAccel, mass, launchGCoupled, 'Color', orange, 'LineWidth', 1.8, ...
    'DisplayName', 'Launch a_x (CG -1 mm/kg)');
if isfinite(referenceMassKg)
    xline(axAccel, referenceMassKg, ':', 'Color', refColor, ...
        'LineWidth', 1.1, 'HandleVisibility', 'off');
end
if isfinite(knee.massKg)
    plot(axAccel, knee.massKg, knee.launchG, 'o', 'MarkerSize', 8, ...
        'LineWidth', 1.5, 'MarkerFaceColor', kneeColor, ...
        'MarkerEdgeColor', 'w', 'DisplayName', 'Fixed-CG knee');
end
if isfinite(kneeCoupled.massKg)
    plot(axAccel, kneeCoupled.massKg, kneeCoupled.launchG, 's', ...
        'MarkerSize', 8, 'LineWidth', 1.5, ...
        'MarkerFaceColor', kneeCoupledColor, 'MarkerEdgeColor', 'w', ...
        'DisplayName', 'Coupled-CG knee');
end
hold(axAccel, 'off');
styleAxis(axAccel);
grid(axAccel, 'on');
ylabel(axAccel, 'Launch a_x [g]');
title(axAccel, 'Launch g vs mass (lower CG reduces rear load transfer, hurts traction)');
legend(axAccel, 'Location', 'best');

% --- Tile 2: time vs mass ---
axTime = nexttile(layout);
plot(axTime, mass, eventTime, 'Color', blue, 'LineWidth', 1.8, ...
    'DisplayName', sprintf('Time (fixed CG)'));
hold(axTime, 'on');
plot(axTime, mass, eventTimeCoupled, 'Color', orange, 'LineWidth', 1.8, ...
    'DisplayName', sprintf('Time (CG -1 mm/kg)'));
if isfinite(referenceMassKg)
    xline(axTime, referenceMassKg, ':', 'Color', refColor, ...
        'LineWidth', 1.1, 'HandleVisibility', 'off');
end
if isfinite(knee.massKg) && isfinite(knee.timeToDistanceS)
    plot(axTime, knee.massKg, knee.timeToDistanceS, 'o', 'MarkerSize', 8, ...
        'LineWidth', 1.5, 'MarkerFaceColor', kneeColor, ...
        'MarkerEdgeColor', 'w', 'DisplayName', 'Fixed-CG knee');
end
if isfinite(kneeCoupled.massKg) && isfinite(kneeCoupled.timeToDistanceS)
    plot(axTime, kneeCoupled.massKg, kneeCoupled.timeToDistanceS, 's', ...
        'MarkerSize', 8, 'LineWidth', 1.5, ...
        'MarkerFaceColor', kneeCoupledColor, 'MarkerEdgeColor', 'w', ...
        'DisplayName', 'Coupled-CG knee');
end
hold(axTime, 'off');
styleAxis(axTime);
grid(axTime, 'on');
ylabel(axTime, sprintf('Time to %.0f m [s]', distanceM));
title(axTime, 'Time vs mass (coupled CG is slower at low mass: less rear planting)');
legend(axTime, 'Location', 'best');

% --- Tile 3: marginal time benefit per kg saved ---
axBenefit = nexttile(layout);
plot(axBenefit, mass, benefitMsPerKg, 'Color', green, 'LineWidth', 1.6, ...
    'DisplayName', 'Marginal gain (fixed CG)');
hold(axBenefit, 'on');
plot(axBenefit, mass, benefitMsPerKgCoupled, 'Color', teal, ...
    'LineWidth', 1.6, 'DisplayName', 'Marginal gain (CG -1 mm/kg)');
yline(axBenefit, 0, ':', 'Color', refColor, 'HandleVisibility', 'off');
if isfinite(referenceMassKg)
    xline(axBenefit, referenceMassKg, ':', 'Color', refColor, ...
        'LineWidth', 1.1, 'HandleVisibility', 'off');
end
if isfinite(knee.massKg) && isfinite(knee.benefitSPerKgSaved)
    plot(axBenefit, knee.massKg, 1000 * knee.benefitSPerKgSaved, ...
        'o', 'MarkerSize', 8, 'LineWidth', 1.5, ...
        'MarkerFaceColor', kneeColor, 'MarkerEdgeColor', 'w', ...
        'DisplayName', 'Fixed-CG knee');
end
if isfinite(kneeCoupled.massKg) && isfinite(kneeCoupled.benefitSPerKgSaved)
    plot(axBenefit, kneeCoupled.massKg, ...
        1000 * kneeCoupled.benefitSPerKgSaved, 's', 'MarkerSize', 8, ...
        'LineWidth', 1.5, 'MarkerFaceColor', kneeCoupledColor, ...
        'MarkerEdgeColor', 'w', 'DisplayName', 'Coupled-CG knee');
end
hold(axBenefit, 'off');
styleAxis(axBenefit);
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

function cgHeight = coupledCgHeight(massKg, referenceCgHeight, ...
        referenceMassKg, cgDropPerKgSavedM)
% Per-kg CG schedule: for every kg of mass saved below ReferenceMassKg the
% CG drops by CgDropPerKgSavedM (default 1 mm). Anchored at the reference
% mass so the coupled sweep reproduces the baseline there. CG is not
% permitted to go negative.
kgSaved = max(referenceMassKg - massKg(:), 0);
cgHeight = max(referenceCgHeight - kgSaved .* cgDropPerKgSavedM, 0);
end

function coupledFile = coupledOutputFile(baseFile)
% Mirror the base figure path with a _coupled_cg suffix before the extension.
[folder, name, ext] = fileparts(baseFile);
coupledFile = fullfile(folder, [name '_coupled_cg' ext]);
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

function tf = validateSlipRatios(value)
validateattributes(value, {'numeric'}, ...
    {'vector', 'real', 'finite', 'nonnegative'}, mfilename, ...
    'EnvelopeSlipRatios');
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
