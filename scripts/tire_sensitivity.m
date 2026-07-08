function result = tire_sensitivity(tirFile, varargin)
%TIRE_SENSITIVITY Plot tire force load sensitivity for a Pacejka .tir file.
%
%   result = tire_sensitivity()
%   result = tire_sensitivity('43105_18x7.5_10_R25B_7.tir')
%   result = tire_sensitivity(tirFile, 'NormalLoads', 200:50:1800, ...
%       'SlipAnglesDeg', 0:2:12, 'SlipRatios', 0.02:0.02:0.12)
%
% The lateral plot stacks one panel per slip angle with zero longitudinal
% slip. The longitudinal plot stacks one panel per slip ratio with zero slip
% angle. Both sweeps use the tire file's reference speed and pressure. The
% acceleration-vs-mass plot uses a simple steady-state bicycle/skidpad load
% transfer model and highlights a diminishing-returns point.

defaultTirFile = '43105_18x7.5_10_R25B_7_theoretical.tir';
optionNames = {'NormalLoads', 'SlipAnglesDeg', 'SlipRatios', 'SurfaceMu', ...
    'OutputFile', 'LateralOutputFile', 'LongitudinalOutputFile', ...
    'MassAccelerationOutputFile', 'HighlightSkidpadPoint', ...
    'HighlightDiminishingReturnsPoint', 'TrackWidth', 'CgHeight', ...
    'StaticFrontWeight', 'LateralLoadTransferDistribution', ...
    'MassStepKg', 'MassSlipAnglesDeg', 'SavePlot', 'ShowFigure', ...
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

defaultLateralOutput = fullfile(repoRoot, 'exports', ...
    'tire_lateral_sensitivity.png');
defaultLongitudinalOutput = fullfile(repoRoot, 'exports', ...
    'tire_longitudinal_sensitivity.png');
defaultMassAccelerationOutput = fullfile(repoRoot, 'exports', ...
    'tire_acceleration_vs_mass.png');

parser = inputParser;
parser.FunctionName = 'tire_sensitivity';
parser.addParameter('NormalLoads', 200:50:1000, @validateNormalLoads);
parser.addParameter('SlipAnglesDeg', 0:2:12, @validateSlipAngles);
parser.addParameter('SlipRatios', 0.02:0.02:0.12, @validateSlipRatios);
parser.addParameter('SurfaceMu', [], @validateSurfaceMu);
parser.addParameter('OutputFile', '', @(x) ischar(x) || isstring(x));
parser.addParameter('LateralOutputFile', defaultLateralOutput, ...
    @(x) ischar(x) || isstring(x));
parser.addParameter('LongitudinalOutputFile', defaultLongitudinalOutput, ...
    @(x) ischar(x) || isstring(x));
parser.addParameter('MassAccelerationOutputFile', defaultMassAccelerationOutput, ...
    @(x) ischar(x) || isstring(x));
parser.addParameter('HighlightSkidpadPoint', false, @validateScalarLogical);
parser.addParameter('HighlightDiminishingReturnsPoint', true, ...
    @validateScalarLogical);
parser.addParameter('TrackWidth', 1.21, @validatePositiveScalar);
parser.addParameter('CgHeight', 0.30, @validateNonnegativeScalar);
parser.addParameter('StaticFrontWeight', 0.50, @validateUnitScalar);
parser.addParameter('LateralLoadTransferDistribution', [], @validateUnitScalarOrEmpty);
parser.addParameter('MassStepKg', 0.1, @validatePositiveScalar);
parser.addParameter('MassSlipAnglesDeg', 0:0.1:14, @validateSlipAngles);
parser.addParameter('SavePlot', true, @validateScalarLogical);
parser.addParameter('ShowFigure', true, @validateScalarLogical);
parser.addParameter('CloseFigure', false, @validateScalarLogical);
parser.parse(varargin{:});
opts = parser.Results;
loadTransfer = loadTransferOptions(opts);

tirFile = char(tirFile);
normalLoads = unique(opts.NormalLoads(:), 'sorted');
slipAnglesDeg = opts.SlipAnglesDeg(:).';
massSlipAnglesDeg = opts.MassSlipAnglesDeg(:).';
slipRatios = opts.SlipRatios(:).';
if isempty(normalLoads)
    error('tire_sensitivity:EmptyNormalLoads', ...
        'NormalLoads must contain at least one value.');
end
if isempty(slipAnglesDeg)
    error('tire_sensitivity:EmptySlipAngles', ...
        'SlipAnglesDeg must contain at least one value.');
end
if isempty(massSlipAnglesDeg)
    error('tire_sensitivity:EmptyMassSlipAngles', ...
        'MassSlipAnglesDeg must contain at least one value.');
end
if isempty(slipRatios)
    error('tire_sensitivity:EmptySlipRatios', ...
        'SlipRatios must contain at least one value.');
end

tire = lts.components.Tire.PacejkaTire(tirFile);
surfaceMu = opts.SurfaceMu;
if isempty(surfaceMu)
    surfaceMu = tire.surfaceMuReference;
end

Fy = computeLateralForceGrid(tire, normalLoads, slipAnglesDeg, surfaceMu);
muY = Fy ./ max(normalLoads, eps);
Fx = computeLongitudinalForceGrid(tire, normalLoads, slipRatios, surfaceMu);
muX = Fx ./ max(normalLoads, eps);
skidpadPoint = computeSkidpadPoint(normalLoads, slipAnglesDeg, Fy, muY);
massCurveMassKg = massGridFromNormalLoads(normalLoads, opts.MassStepKg);
massCurveLoads = massCurveMassKg * 9.80665 / 4;
massCurveFy = computeLateralForceGrid( ...
    tire, massCurveLoads, massSlipAnglesDeg, surfaceMu);
massAcceleration = computeMassAccelerationCurve( ...
    massCurveLoads, massSlipAnglesDeg, massCurveFy, loadTransfer, ...
    opts.MassStepKg);
diminishingReturnsPoint = findDiminishingReturnsPoint(massAcceleration);

fprintf('\n=== Tire Force Sensitivity ===\n');
fprintf('Tire file:       %s\n', tire.tireConstants.tirFilePath);
fprintf('Surface mu:      %.3f\n', surfaceMu);
fprintf('Normal load:     %.0f to %.0f N (%d points)\n', ...
    min(normalLoads), max(normalLoads), numel(normalLoads));
fprintf('Slip angles:     %s deg\n', formatNumberList(slipAnglesDeg, '%.1f'));
fprintf('Slip ratios:     %s\n', formatNumberList(slipRatios, '%.3f'));
fprintf('Peak |Fy|:       %.0f N\n', max(abs(Fy(:))));
fprintf('Peak |mu_y|:     %.3f\n', max(abs(muY(:))));
fprintf('Peak |Fx|:       %.0f N\n', max(abs(Fx(:))));
fprintf('Peak |mu_x|:     %.3f\n', max(abs(muX(:))));
fprintf('Best equal-load tire point: %.0f N/tire, %.1f kg/tire, %.1f kg equal-load car, %.3f g at %.1f deg\n', ...
    skidpadPoint.normalLoadN, skidpadPoint.cornerMassKg, ...
    skidpadPoint.equalLoadVehicleMassKg, skidpadPoint.lateralAccelG, ...
    skidpadPoint.slipAngleDeg);
fprintf('Load transfer:   track %.3f m, CG %.3f m, static front %.1f%%, front LLTD %.1f%%\n', ...
    loadTransfer.trackWidth, loadTransfer.cgHeight, ...
    100 * loadTransfer.staticFrontWeight, ...
    100 * loadTransfer.frontLoadTransferDistribution);
fprintf('Mass sweep:      %.1f to %.1f kg in %.3f kg steps (%d points)\n', ...
    min(massAcceleration.massKg), max(massAcceleration.massKg), ...
    opts.MassStepKg, numel(massAcceleration.massKg));
fprintf('Mass slip grid:  %.1f to %.1f deg in %.3f deg steps (%d points)\n', ...
    min(massSlipAnglesDeg), max(massSlipAnglesDeg), ...
    representativeStep(massSlipAnglesDeg), numel(massSlipAnglesDeg));
fprintf('Diminishing-return point: %.1f kg car, %.3f g, %.3f milli-g/kg saved (%s)\n', ...
    diminishingReturnsPoint.massKg, diminishingReturnsPoint.accelG, ...
    1000 * diminishingReturnsPoint.benefitGPerKgSaved, ...
    diminishingReturnsPoint.method);

visibleState = onOff(opts.ShowFigure, 'on', 'off');
if opts.HighlightSkidpadPoint
    lateralHighlight = skidpadPoint;
else
    lateralHighlight = [];
end
lateralFig = plotSensitivityStack( ...
    normalLoads, slipAnglesDeg, Fy, muY, ...
    tire.tireConstants.tirFilePath, surfaceMu, visibleState, ...
    'Tire lateral sensitivity', ...
    'Lateral force vs normal load', ...
    'Fy [kN]', ...
    'Slip angle %.1f deg | peak |mu_y| %.3f', ...
    lateralHighlight);
longitudinalFig = plotSensitivityStack( ...
    normalLoads, slipRatios, Fx, muX, ...
    tire.tireConstants.tirFilePath, surfaceMu, visibleState, ...
    'Tire longitudinal sensitivity', ...
    'Longitudinal force vs normal load', ...
    'Fx [kN]', ...
    'Slip ratio %.3f | peak |mu_x| %.3f', ...
    []);
if opts.HighlightDiminishingReturnsPoint
    massHighlight = diminishingReturnsPoint;
else
    massHighlight = [];
end
massAccelerationFig = plotMassAccelerationCurve( ...
    massAcceleration, massHighlight, tire.tireConstants.tirFilePath, ...
    surfaceMu, loadTransfer, visibleState);

lateralOutputFile = char(opts.LateralOutputFile);
if ~isempty(opts.OutputFile)
    lateralOutputFile = char(opts.OutputFile);
end
longitudinalOutputFile = char(opts.LongitudinalOutputFile);
massAccelerationOutputFile = char(opts.MassAccelerationOutputFile);
if opts.SavePlot
    writeFigureToFile(lateralFig, lateralOutputFile);
    fprintf('Saved lateral tire sensitivity plot: %s\n', lateralOutputFile);
    writeFigureToFile(longitudinalFig, longitudinalOutputFile);
    fprintf('Saved longitudinal tire sensitivity plot: %s\n', ...
        longitudinalOutputFile);
    writeFigureToFile(massAccelerationFig, massAccelerationOutputFile);
    fprintf('Saved acceleration-vs-mass plot: %s\n', ...
        massAccelerationOutputFile);
end

result = struct( ...
    'tirFile', tire.tireConstants.tirFilePath, ...
    'surfaceMu', surfaceMu, ...
    'normalLoadsN', normalLoads, ...
    'slipAnglesDeg', slipAnglesDeg, ...
    'massSlipAnglesDeg', massSlipAnglesDeg, ...
    'slipRatios', slipRatios, ...
    'lateralForceN', Fy, ...
    'lateralMu', muY, ...
    'longitudinalForceN', Fx, ...
    'longitudinalMu', muX, ...
    'skidpadPoint', skidpadPoint, ...
    'loadTransfer', loadTransfer, ...
    'massAcceleration', massAcceleration, ...
    'diminishingReturnsPoint', diminishingReturnsPoint, ...
    'outputFile', lateralOutputFile, ...
    'lateralOutputFile', lateralOutputFile, ...
    'longitudinalOutputFile', longitudinalOutputFile, ...
    'massAccelerationOutputFile', massAccelerationOutputFile, ...
    'figure', lateralFig, ...
    'lateralFigure', lateralFig, ...
    'longitudinalFigure', longitudinalFig, ...
    'massAccelerationFigure', massAccelerationFig);

if opts.CloseFigure
    close(lateralFig);
    close(longitudinalFig);
    close(massAccelerationFig);
    result.figure = [];
    result.lateralFigure = [];
    result.longitudinalFigure = [];
    result.massAccelerationFigure = [];
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
    outputs = mfeval(tire.tireConstants.params, inputsMF, 111);
    FyActive(idx) = -outputs(:, 2) * surfaceScale;
end

Fy(active, :) = FyActive;
end

function point = computeSkidpadPoint(normalLoads, slipAnglesDeg, Fy, muY)
g = 9.80665;
[lateralAccelG, linearIdx] = max(abs(muY(:)));
[loadIdx, slipIdx] = ind2sub(size(muY), linearIdx);
normalLoad = normalLoads(loadIdx);
point = struct( ...
    'normalLoadN', normalLoad, ...
    'forceN', Fy(loadIdx, slipIdx), ...
    'slipAngleDeg', slipAnglesDeg(slipIdx), ...
    'loadIndex', loadIdx, ...
    'slipIndex', slipIdx, ...
    'lateralAccelG', lateralAccelG, ...
    'cornerMassKg', normalLoad / g, ...
    'equalLoadVehicleMassKg', 4 * normalLoad / g);
end

function curve = computeMassAccelerationCurve(normalLoads, slipAnglesDeg, Fy, ...
        loadTransfer, massStepKg)
g = 9.80665;
[peakForceByLoad, bestSlipIdx] = max(abs(Fy), [], 2);
massKg = massGridFromNormalLoads(normalLoads, massStepKg);
normalLoadsForMass = massKg * g / 4;
bestAccelG = zeros(size(massKg));
bestAccelMps2 = zeros(size(massKg));
bestForceN = zeros(size(massKg));
bestSlipAnglesDeg = zeros(size(massKg));
cornerLoadsN = zeros(numel(massKg), 4);
frontCapacityN = zeros(size(massKg));
rearCapacityN = zeros(size(massKg));
capacityLimitedBy = strings(size(massKg));
bestSlipByLoad = slipAnglesDeg(bestSlipIdx);

for i = 1:numel(massKg)
    [bestAccelG(i), detail] = solveLoadTransferSkidpadMass( ...
        massKg(i), normalLoads, peakForceByLoad, bestSlipByLoad, ...
        loadTransfer);
    bestAccelMps2(i) = bestAccelG(i) * g;
    bestForceN(i) = detail.totalCapacityN;
    bestSlipAnglesDeg(i) = detail.meanSlipAngleDeg;
    cornerLoadsN(i, :) = detail.cornerLoadsN(:).';
    frontCapacityN(i) = detail.frontCapacityN;
    rearCapacityN(i) = detail.rearCapacityN;
    capacityLimitedBy(i) = string(detail.limitedBy);
end

if numel(massKg) >= 2
    dAccelGdKg = gradient(bestAccelG, massKg);
    benefitGPerKgSaved = -dAccelGdKg;
else
    dAccelGdKg = NaN(size(massKg));
    benefitGPerKgSaved = NaN(size(massKg));
end

if numel(massKg) >= 3
    d2AccelGdKg2 = gradient(dAccelGdKg, massKg);
else
    d2AccelGdKg2 = NaN(size(massKg));
end

curve = struct( ...
    'massKg', massKg, ...
    'normalLoadsN', normalLoadsForMass, ...
    'massStepKg', massStepKg, ...
    'bestAccelG', bestAccelG, ...
    'bestAccelMps2', bestAccelMps2, ...
    'bestForceN', bestForceN, ...
    'bestSlipAnglesDeg', bestSlipAnglesDeg, ...
    'cornerLoadsN', cornerLoadsN, ...
    'frontCapacityN', frontCapacityN, ...
    'rearCapacityN', rearCapacityN, ...
    'capacityLimitedBy', capacityLimitedBy, ...
    'loadTransfer', loadTransfer, ...
    'dAccelGdKg', dAccelGdKg, ...
    'benefitGPerKgSaved', benefitGPerKgSaved, ...
    'd2AccelGdKg2', d2AccelGdKg2);
end

function massKg = massGridFromNormalLoads(normalLoads, massStepKg)
g = 9.80665;
minMass = 4 * min(normalLoads) / g;
maxMass = 4 * max(normalLoads) / g;
massKg = (minMass:massStepKg:maxMass).';
if isempty(massKg)
    massKg = [minMass; maxMass];
elseif massKg(end) < maxMass
    massKg(end + 1, 1) = maxMass;
end
massKg = unique(massKg, 'stable');
end

function [ayG, detail] = solveLoadTransferSkidpadMass( ...
        massKg, loadGridN, peakForceGridN, bestSlipGridDeg, loadTransfer)
maxEqualLoadMu = max(peakForceGridN ./ max(loadGridN, eps));
upper = max(0.5, 1.5 * maxEqualLoadMu);
while loadTransferResidual(upper, massKg, loadGridN, peakForceGridN, ...
        bestSlipGridDeg, loadTransfer) > 0 && upper < 5
    upper = upper * 1.5;
end

lower = 0;
for iter = 1:50 %#ok<NASGU>
    mid = 0.5 * (lower + upper);
    residual = loadTransferResidual(mid, massKg, loadGridN, ...
        peakForceGridN, bestSlipGridDeg, loadTransfer);
    if residual >= 0
        lower = mid;
    else
        upper = mid;
    end
end

ayG = lower;
detail = loadTransferCapacityAtAy( ...
    massKg, ayG, loadGridN, peakForceGridN, bestSlipGridDeg, loadTransfer);
end

function residual = loadTransferResidual(ayG, massKg, loadGridN, ...
        peakForceGridN, bestSlipGridDeg, loadTransfer)
detail = loadTransferCapacityAtAy( ...
    massKg, ayG, loadGridN, peakForceGridN, bestSlipGridDeg, loadTransfer);
residual = detail.capacityG - ayG;
end

function detail = loadTransferCapacityAtAy(massKg, ayG, loadGridN, ...
        peakForceGridN, bestSlipGridDeg, loadTransfer)
g = 9.80665;
cornerLoads = bicycleCornerLoads(massKg, ayG, loadTransfer);
[cornerCapacity, cornerSlipDeg] = tireCapacityForLoads( ...
    cornerLoads, loadGridN, peakForceGridN, bestSlipGridDeg);

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
if sum(cornerCapacity) > 0
    meanSlipAngleDeg = sum(cornerSlipDeg .* cornerCapacity) / ...
        sum(cornerCapacity);
else
    meanSlipAngleDeg = NaN;
end

detail = struct( ...
    'capacityG', capacityG, ...
    'totalCapacityN', totalCapacity, ...
    'frontCapacityN', frontCapacity, ...
    'rearCapacityN', rearCapacity, ...
    'cornerCapacityN', cornerCapacity, ...
    'cornerLoadsN', cornerLoads, ...
    'meanSlipAngleDeg', meanSlipAngleDeg, ...
    'limitedBy', limitedBy);
end

function loads = bicycleCornerLoads(massKg, ayG, loadTransfer)
g = 9.80665;
W = massKg * g;
frontAxleStatic = W * loadTransfer.staticFrontWeight;
rearAxleStatic = W * (1 - loadTransfer.staticFrontWeight);
totalTransfer = W * ayG * loadTransfer.cgHeight / loadTransfer.trackWidth;
frontTransfer = totalTransfer * ...
    loadTransfer.frontLoadTransferDistribution;
rearTransfer = totalTransfer * ...
    (1 - loadTransfer.frontLoadTransferDistribution);

frontInside = max((frontAxleStatic - frontTransfer) / 2, 0);
frontOutside = frontAxleStatic - frontInside;
rearInside = max((rearAxleStatic - rearTransfer) / 2, 0);
rearOutside = rearAxleStatic - rearInside;
loads = [frontInside; frontOutside; rearInside; rearOutside];
end

function [capacity, slipAngleDeg] = tireCapacityForLoads( ...
        loads, loadGridN, peakForceGridN, bestSlipGridDeg)
capacity = zeros(size(loads));
slipAngleDeg = zeros(size(loads));
minLoad = min(loadGridN);
maxLoad = max(loadGridN);
for i = 1:numel(loads)
    Fz = loads(i);
    if Fz <= 0
        capacity(i) = 0;
        slipAngleDeg(i) = NaN;
        continue;
    end
    capacity(i) = interp1(loadGridN, peakForceGridN, Fz, ...
        'linear', 'extrap');
    capacity(i) = max(capacity(i), 0);
    clippedFz = min(max(Fz, minLoad), maxLoad);
    slipAngleDeg(i) = interp1(loadGridN, bestSlipGridDeg, clippedFz, ...
        'nearest', 'extrap');
end
end

function point = findDiminishingReturnsPoint(curve)
mass = curve.massKg(:);
benefit = curve.benefitGPerKgSaved(:);
d2 = curve.d2AccelGdKg2(:);

if isempty(mass)
    point = emptyDiminishingReturnsPoint();
    return;
elseif numel(mass) == 1
    point = pointFromMass(curve, mass(1), 'insufficient-samples');
    return;
end

crossingMasses = secondDerivativeCrossingMasses(mass, d2);
if ~isempty(crossingMasses)
    crossingBenefits = interp1(mass, benefit, crossingMasses, ...
        'linear', 'extrap');
    [~, bestCrossingIdx] = max(crossingBenefits);
    point = pointFromMass(curve, crossingMasses(bestCrossingIdx), ...
        'inflection');
    return;
end

kneeIdx = kneeIndex(mass, curve.bestAccelG(:));
point = pointFromMass(curve, mass(kneeIdx), 'knee-fallback');
end

function masses = secondDerivativeCrossingMasses(mass, d2)
masses = [];
if numel(mass) < 3
    return;
end

for i = 1:(numel(mass) - 1)
    if ~all(isfinite([d2(i), d2(i + 1)]))
        continue;
    end
    if d2(i) == 0
        masses(end + 1) = mass(i); %#ok<AGROW>
    elseif d2(i + 1) == 0
        masses(end + 1) = mass(i + 1); %#ok<AGROW>
    elseif d2(i) * d2(i + 1) < 0
        t = -d2(i) / (d2(i + 1) - d2(i));
        masses(end + 1) = mass(i) + t * (mass(i + 1) - mass(i)); %#ok<AGROW>
    end
end

masses = unique(masses);
end

function idx = kneeIndex(mass, accel)
n = numel(mass);
if n <= 2
    idx = 1;
    return;
end

x = normalizeRange(mass);
y = normalizeRange(accel);
p1 = [x(1), y(1)];
p2 = [x(end), y(end)];
lineVec = p2 - p1;
lineNorm = hypot(lineVec(1), lineVec(2));
if lineNorm <= eps
    idx = ceil(n / 2);
    return;
end

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

function point = pointFromMass(curve, massKg, method)
g = 9.80665;
mass = curve.massKg(:);
[~, idx] = min(abs(mass - massKg));
point = struct( ...
    'method', method, ...
    'index', idx, ...
    'massKg', massKg, ...
    'normalLoadN', massKg * g / 4, ...
    'accelG', interp1(mass, curve.bestAccelG(:), massKg, ...
        'linear', 'extrap'), ...
    'accelMps2', interp1(mass, curve.bestAccelMps2(:), massKg, ...
        'linear', 'extrap'), ...
    'benefitGPerKgSaved', interp1(mass, ...
        curve.benefitGPerKgSaved(:), massKg, 'linear', 'extrap'), ...
    'bestSlipAngleDeg', curve.bestSlipAnglesDeg(idx));
end

function point = emptyDiminishingReturnsPoint()
point = struct( ...
    'method', 'empty', ...
    'index', NaN, ...
    'massKg', NaN, ...
    'normalLoadN', NaN, ...
    'accelG', NaN, ...
    'accelMps2', NaN, ...
    'benefitGPerKgSaved', NaN, ...
    'bestSlipAngleDeg', NaN);
end

function Fx = computeLongitudinalForceGrid(tire, normalLoads, slipRatios, surfaceMu)
normalLoads = normalLoads(:);
Fx = zeros(numel(normalLoads), numel(slipRatios));
surfaceScale = tireSurfaceScale(tire, surfaceMu);
active = normalLoads > 0;
for j = 1:numel(slipRatios)
    if any(active)
        nActive = nnz(active);
        inputsMF = [ ...
            normalLoads(active), ...
            repmat(slipRatios(j), nActive, 1), ...
            zeros(nActive, 1), ...
            zeros(nActive, 1), ...
            zeros(nActive, 1), ...
            repmat(tire.tireConstants.refVelocity, nActive, 1), ...
            repmat(tire.tireConstants.nomPressure, nActive, 1)];
        outputs = mfeval(tire.tireConstants.params, inputsMF, 111);
        Fx(active, j) = outputs(:, 1) * surfaceScale;
    end
end
end

function scale = tireSurfaceScale(tire, surfaceMu)
if isempty(surfaceMu) || ~isfinite(surfaceMu)
    surfaceMu = tire.surfaceMuReference;
end
scale = max(surfaceMu, 0) / max(tire.surfaceMuReference, eps);
end

function fig = plotMassAccelerationCurve(curve, highlight, tireFilePath, ...
        surfaceMu, loadTransfer, visibleState)
fig = figure('Name', 'Tire acceleration vs mass sensitivity', ...
    'Color', 'w', 'Visible', visibleState);
fig.Position = [120 80 1150 820];
set(fig, ...
    'DefaultTextColor', 'k', ...
    'DefaultAxesColor', 'w', ...
    'DefaultAxesXColor', 'k', ...
    'DefaultAxesYColor', 'k', ...
    'DefaultLegendColor', 'w', ...
    'DefaultLegendTextColor', 'k');

layout = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', ...
    'Padding', 'loose');
mass = curve.massKg;
accel = curve.bestAccelG;
benefitMilliGPerKg = 1000 * curve.benefitGPerKgSaved;

axAccel = nexttile(layout);
plot(axAccel, mass, accel, 'Color', [0.07 0.29 0.67], ...
    'LineWidth', 1.8, 'DisplayName', 'Best sampled acceleration');
hold(axAccel, 'on');
if ~isempty(highlight)
    xline(axAccel, highlight.massKg, '--', 'Color', [0.80 0.12 0.10], ...
        'LineWidth', 1.1, 'HandleVisibility', 'off');
    plot(axAccel, highlight.massKg, highlight.accelG, 'o', ...
        'MarkerSize', 8, 'LineWidth', 1.5, ...
        'MarkerFaceColor', [0.80 0.12 0.10], ...
        'MarkerEdgeColor', 'w', 'DisplayName', highlight.method);
end
hold(axAccel, 'off');
styleAxis(axAccel);
grid(axAccel, 'on');
ylabel(axAccel, 'Best a_y [g]');
title(axAccel, 'Best steady-state skidpad acceleration vs mass');
legend(axAccel, 'Location', 'best');

axBenefit = nexttile(layout);
plot(axBenefit, mass, benefitMilliGPerKg, 'Color', [0.10 0.50 0.28], ...
    'LineWidth', 1.6, 'DisplayName', 'Marginal gain');
hold(axBenefit, 'on');
yline(axBenefit, 0, ':', 'Color', [0.45 0.45 0.45], ...
    'HandleVisibility', 'off');
if ~isempty(highlight)
    xline(axBenefit, highlight.massKg, '--', 'Color', [0.80 0.12 0.10], ...
        'LineWidth', 1.1, 'HandleVisibility', 'off');
    plot(axBenefit, highlight.massKg, ...
        1000 * highlight.benefitGPerKgSaved, 'o', ...
        'MarkerSize', 8, 'LineWidth', 1.5, ...
        'MarkerFaceColor', [0.80 0.12 0.10], ...
        'MarkerEdgeColor', 'w', 'DisplayName', highlight.method);
end
hold(axBenefit, 'off');
styleAxis(axBenefit);
grid(axBenefit, 'on');
xlabel(axBenefit, 'Vehicle mass [kg]');
ylabel(axBenefit, 'Gain [milli-g/kg saved]');
title(axBenefit, 'Marginal acceleration gain from saving 1 kg');
legend(axBenefit, 'Location', 'best');

linkaxes([axAccel, axBenefit], 'x');
[~, tireName, tireExt] = fileparts(tireFilePath);
plotTitle = sgtitle(sprintf( ...
    ['Acceleration vs mass | %s%s | surface mu %.3f | ' ...
     'track %.2f m, CG %.2f m, front %.0f%%, LLTD %.0f%%'], ...
    tireName, tireExt, surfaceMu, loadTransfer.trackWidth, ...
    loadTransfer.cgHeight, 100 * loadTransfer.staticFrontWeight, ...
    100 * loadTransfer.frontLoadTransferDistribution), ...
    'Interpreter', 'none');
set(plotTitle, 'Color', 'k', 'FontWeight', 'bold');
end

function fig = plotSensitivityStack(normalLoads, slipValues, force, mu, ...
        tireFilePath, surfaceMu, visibleState, figureName, titlePrefix, ...
        forceLabel, tileTitleFormat, highlight)
nSlips = numel(slipValues);
heightPx = min(1600, max(620, 155 * nSlips + 150));
fig = figure('Name', figureName, 'Color', 'w', ...
    'Visible', visibleState);
fig.Position = [100 50 1150 heightPx];
set(fig, ...
    'DefaultTextColor', 'k', ...
    'DefaultAxesColor', 'w', ...
    'DefaultAxesXColor', 'k', ...
    'DefaultAxesYColor', 'k', ...
    'DefaultLegendColor', 'w', ...
    'DefaultLegendTextColor', 'k');

layout = tiledlayout(fig, nSlips, 1, ...
    'TileSpacing', 'compact', 'Padding', 'loose');
axesList = gobjects(nSlips, 1);
normalLoadKN = normalLoads / 1000;
forceKN = force / 1000;
color = [0.07 0.29 0.67];
[yLower, yUpper] = sharedForceLimits(forceKN);

for j = 1:nSlips
    ax = nexttile(layout);
    axesList(j) = ax;
    plot(ax, normalLoadKN, forceKN(:, j), 'Color', color, 'LineWidth', 1.6);
    hold(ax, 'on');
    yline(ax, 0, ':', 'Color', [0.45 0.45 0.45], ...
        'HandleVisibility', 'off');
    if ~isempty(highlight)
        xline(ax, highlight.normalLoadN / 1000, '--', ...
            'Color', [0.80 0.12 0.10], 'LineWidth', 1.0, ...
            'HandleVisibility', 'off');
        if j == highlight.slipIndex
            plot(ax, highlight.normalLoadN / 1000, highlight.forceN / 1000, ...
                'o', 'MarkerSize', 7, 'LineWidth', 1.5, ...
                'MarkerFaceColor', [0.80 0.12 0.10], ...
                'MarkerEdgeColor', 'w');
        end
    end
    hold(ax, 'off');
    styleAxis(ax);
    grid(ax, 'on');
    xlim(ax, axisLimits(normalLoadKN));
    ylim(ax, [yLower, yUpper]);
    ylabel(ax, forceLabel);
    tileTitle = sprintf(tileTitleFormat, slipValues(j), max(abs(mu(:, j))));
    if ~isempty(highlight) && j == highlight.slipIndex
        tileTitle = sprintf('%s | best %.3f g, %.1f kg car', ...
            tileTitle, highlight.lateralAccelG, ...
            highlight.equalLoadVehicleMassKg);
    end
    title(ax, tileTitle);
    if j < nSlips
        ax.XTickLabel = [];
    else
        xlabel(ax, 'Normal force Fz [kN]');
    end
end

linkaxes(axesList, 'x');
[~, tireName, tireExt] = fileparts(tireFilePath);
plotTitle = sgtitle(sprintf( ...
    '%s | %s%s | surface mu %.3f', ...
    titlePrefix, tireName, tireExt, surfaceMu), 'Interpreter', 'none');
set(plotTitle, 'Color', 'k', 'FontWeight', 'bold');
end

function [lower, upper] = sharedForceLimits(forceKN)
finiteForce = forceKN(isfinite(forceKN));
if isempty(finiteForce)
    lower = -1;
    upper = 1;
    return;
end

minForce = min(finiteForce);
maxForce = max(finiteForce);
if minForce >= 0
    lower = 0;
    upper = max(maxForce * 1.08, 1e-3);
elseif maxForce <= 0
    lower = min(minForce * 1.08, -1e-3);
    upper = 0;
else
    limit = max(abs(finiteForce)) * 1.08;
    limit = max(limit, 1e-3);
    lower = -limit;
    upper = limit;
end
end

function limits = axisLimits(values)
minValue = min(values);
maxValue = max(values);
if minValue == maxValue
    pad = max(abs(minValue) * 0.05, 0.1);
    limits = [minValue - pad, maxValue + pad];
else
    limits = [minValue, maxValue];
end
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

function text = formatNumberList(values, formatSpec)
parts = arrayfun(@(v) sprintf(formatSpec, v), values, 'UniformOutput', false);
text = strjoin(parts, ', ');
end

function step = representativeStep(values)
values = values(:);
if numel(values) < 2
    step = NaN;
else
    step = median(diff(values));
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

function tf = validateNormalLoads(value)
validateattributes(value, {'numeric'}, ...
    {'vector', 'real', 'finite', 'nonnegative'}, mfilename, 'NormalLoads');
tf = true;
end

function tf = validateSlipAngles(value)
validateattributes(value, {'numeric'}, ...
    {'vector', 'real', 'finite'}, mfilename, 'SlipAnglesDeg');
tf = true;
end

function tf = validateSlipRatios(value)
validateattributes(value, {'numeric'}, ...
    {'vector', 'real', 'finite', 'positive', '<=', 1}, ...
    mfilename, 'SlipRatios');
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
