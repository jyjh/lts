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
%
% Shared force-grid, skidpad, and plot machinery lives in the +wsc package
% next to this script.

defaultTirFile = '43105_18x7.5_10_R25B_7_theoretical.tir';
optionNames = {'NormalLoads', 'SlipAnglesDeg', 'SlipRatios', 'OutputFile', ...
    'LateralOutputFile', 'LongitudinalOutputFile', ...
    'MassAccelerationOutputFile', 'HighlightSkidpadPoint', ...
    'HighlightDiminishingReturnsPoint', 'TrackWidth', 'CgHeight', ...
    'StaticFrontWeight', 'LateralLoadTransferDistribution', ...
    'MassStepKg', 'MassSlipAnglesDeg', 'SavePlot', 'ShowFigure', ...
    'CloseFigure'};
[tirFile, varargin] = wsc.resolveTireArg(tirFile, defaultTirFile, ...
    optionNames, varargin{:});
wsc.addScriptPaths();

repoRoot = fileparts(fileparts(mfilename('fullpath')));
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
parser.addParameter('LateralOutputFile', defaultLateralOutput, ...
    @(x) ischar(x) || isstring(x));
parser.addParameter('LongitudinalOutputFile', defaultLongitudinalOutput, ...
    @(x) ischar(x) || isstring(x));
parser.addParameter('MassAccelerationOutputFile', defaultMassAccelerationOutput, ...
    @(x) ischar(x) || isstring(x));
parser.addParameter('HighlightSkidpadPoint', false, @wsc.validateScalarLogical);
parser.addParameter('HighlightDiminishingReturnsPoint', true, ...
    @wsc.validateScalarLogical);
parser.addParameter('TrackWidth', 1.21, @wsc.validatePositiveScalar);
parser.addParameter('CgHeight', 0.30, @wsc.validateNonnegativeScalar);
parser.addParameter('StaticFrontWeight', 0.50, @wsc.validateUnitScalar);
parser.addParameter('LateralLoadTransferDistribution', [], @wsc.validateUnitScalarOrEmpty);
parser.addParameter('MassStepKg', 0.1, @wsc.validatePositiveScalar);
parser.addParameter('MassSlipAnglesDeg', 0:0.1:14, @validateSlipAngles);
wsc.addFigureOptions(parser, '');
parser.parse(varargin{:});
opts = parser.Results;
loadTransfer = wsc.skidpadLoadTransfer(opts);

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
surfaceMu = 1.0;

Fy = wsc.lateralForceGrid(tire, normalLoads, slipAnglesDeg);
muY = Fy ./ max(normalLoads, eps);
Fx = wsc.longitudinalForceGrid(tire, normalLoads, slipRatios);
muX = Fx ./ max(normalLoads, eps);
skidpadPoint = computeSkidpadPoint(normalLoads, slipAnglesDeg, Fy, muY);
massCurveMassKg = massGridFromNormalLoads(normalLoads, opts.MassStepKg);
massCurveLoads = massCurveMassKg * 9.80665 / 4;
massCurveFy = wsc.lateralForceGrid( ...
    tire, massCurveLoads, massSlipAnglesDeg);
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

visibleState = wsc.onOff(opts.ShowFigure, 'on', 'off');
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
    wsc.writeFigureToFile(lateralFig, lateralOutputFile);
    fprintf('Saved lateral tire sensitivity plot: %s\n', lateralOutputFile);
    wsc.writeFigureToFile(longitudinalFig, longitudinalOutputFile);
    fprintf('Saved longitudinal tire sensitivity plot: %s\n', ...
        longitudinalOutputFile);
    wsc.writeFigureToFile(massAccelerationFig, massAccelerationOutputFile);
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
    [bestAccelG(i), detail] = wsc.solveSkidpadCapacity( ...
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

kneeIdx = wsc.kneedleIndex(mass, curve.bestAccelG(:));
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

function fig = plotMassAccelerationCurve(curve, highlight, tireFilePath, ...
        surfaceMu, loadTransfer, visibleState)
colors = wsc.plotColors();
fig = figure('Name', 'Tire acceleration vs mass sensitivity', ...
    'Color', 'w', 'Visible', visibleState);
fig.Position = [120 80 1150 820];
wsc.styleFigure(fig);

layout = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', ...
    'Padding', 'loose');
mass = curve.massKg;
accel = curve.bestAccelG;
benefitMilliGPerKg = 1000 * curve.benefitGPerKgSaved;

axAccel = nexttile(layout);
plot(axAccel, mass, accel, 'Color', colors.blue, ...
    'LineWidth', 1.8, 'DisplayName', 'Best sampled acceleration');
hold(axAccel, 'on');
if ~isempty(highlight)
    wsc.kneeMarker(axAccel, highlight.massKg, highlight.accelG, ...
        colors.knee, highlight.method);
end
hold(axAccel, 'off');
wsc.styleAxis(axAccel);
grid(axAccel, 'on');
ylabel(axAccel, 'Best a_y [g]');
title(axAccel, 'Best steady-state skidpad acceleration vs mass');
legend(axAccel, 'Location', 'best');

axBenefit = nexttile(layout);
plot(axBenefit, mass, benefitMilliGPerKg, 'Color', colors.green, ...
    'LineWidth', 1.6, 'DisplayName', 'Marginal gain');
hold(axBenefit, 'on');
wsc.zeroLine(axBenefit, colors.ref);
if ~isempty(highlight)
    wsc.kneeMarker(axBenefit, highlight.massKg, ...
        1000 * highlight.benefitGPerKgSaved, colors.knee, ...
        highlight.method);
end
hold(axBenefit, 'off');
wsc.styleAxis(axBenefit);
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
colors = wsc.plotColors();
nSlips = numel(slipValues);
heightPx = min(1600, max(620, 155 * nSlips + 150));
fig = figure('Name', figureName, 'Color', 'w', ...
    'Visible', visibleState);
fig.Position = [100 50 1150 heightPx];
wsc.styleFigure(fig);

layout = tiledlayout(fig, nSlips, 1, ...
    'TileSpacing', 'compact', 'Padding', 'loose');
axesList = gobjects(nSlips, 1);
normalLoadKN = normalLoads / 1000;
forceKN = force / 1000;
[yLower, yUpper] = sharedForceLimits(forceKN);

for j = 1:nSlips
    ax = nexttile(layout);
    axesList(j) = ax;
    plot(ax, normalLoadKN, forceKN(:, j), 'Color', colors.blue, ...
        'LineWidth', 1.6);
    hold(ax, 'on');
    wsc.zeroLine(ax, colors.ref);
    if ~isempty(highlight)
        xline(ax, highlight.normalLoadN / 1000, '--', ...
            'Color', colors.knee, 'LineWidth', 1.0, ...
            'HandleVisibility', 'off');
        if j == highlight.slipIndex
            plot(ax, highlight.normalLoadN / 1000, highlight.forceN / 1000, ...
                'o', 'MarkerSize', 7, 'LineWidth', 1.5, ...
                'MarkerFaceColor', colors.knee, ...
                'MarkerEdgeColor', 'w');
        end
    end
    hold(ax, 'off');
    wsc.styleAxis(ax);
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
