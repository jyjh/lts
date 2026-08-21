function varargout = validate_racing_line(varargin)
% VALIDATE_RACING_LINE Overlay the generated racing line on the track limits.
%
% Builds the real VehicleManager + DriverModel used by the simulator, asks
% the driver to build its open-loop lap plan (the same path the simulator
% would drive), then plots the resulting racing line over the cone-derived
% track-limits corridor and reports geometric validation checks.
%
% This is a geometry validator only: it does not run the physics loop, so it
% is fast and has no dependency on tire/warmup state.
%
% Example:
%   validate_racing_line                            % 2026 enduro, R25, defaults
%   validate_racing_line('Track','skidpad')         % a TestTrack
%   validate_racing_line('RacingLineOffsetFraction',0.8)
%   validate_racing_line('SaveFigure','exports/racing_line.png')
%   result = validate_racing_line('Track','2026enduro');

scriptDir = fileparts(mfilename('fullpath'));
repoRoot = fileparts(scriptDir);
addpath(fullfile(repoRoot, 'src'));

parser = inputParser;
parser.addParameter('Track', '2026enduro', @(x) ischar(x) || isstring(x));
parser.addParameter('VehicleConfig', @lts.vehicles.R25);
parser.addParameter('RacingLineOffsetFraction', [], @(x) isempty(x) || ...
    (isnumeric(x) && isscalar(x) && x >= 0 && x <= 1));
parser.addParameter('ApexPhase', [], @(x) isempty(x) || ...
    (isnumeric(x) && isscalar(x) && x > 0 && x < 1));
parser.addParameter('CurvatureSmoothDistance', [], @(x) isempty(x) || ...
    (isnumeric(x) && isscalar(x) && x > 0));
parser.addParameter('OffsetSmoothDistance', [], @(x) isempty(x) || ...
    (isnumeric(x) && isscalar(x) && x > 0));
parser.addParameter('SaveFigure', '', @(x) ischar(x) || isstring(x));
parser.addParameter('ShowPlots', true, @(x) islogical(x) || isnumeric(x));
parser.parse(varargin{:});
opts = parser.Results;

dt = 0.001;  % Only feeds suspension warmup; geometry is dt-independent.

track = loadValidatorTrack(opts.Track, repoRoot);
config = loadVehicleConfig(opts.VehicleConfig);
vehicle = lts.vehicle.VehicleManager.fromConfig(config, track, dt, 'Verbose', false);

driver = lts.driver.DriverModel(vehicle);
driver = applyDriverKnobs(driver, opts);

trackData = buildTrackData(track);
initialState = lts.simulation.VehicleState('s', 0, 'speed', 0.1);
driver = driver.prepareForSimulation(initialState, trackData, dt);
profile = driver.inputProfile;

line = extractLine(trackData, profile);
checks = validateLine(trackData, line);

printReport(track, driver, opts, line, checks);
fig = [];
if opts.ShowPlots || ~isempty(char(opts.SaveFigure))
    fig = plotValidation(track, line, checks, driver.racingLineOffsetFraction);
end

saveFigurePath = char(opts.SaveFigure);
if ~isempty(saveFigurePath)
    writeFigurePng(fig, saveFigurePath, repoRoot);
end

if nargout > 0
    varargout{1} = struct( ...
        'track', track, ...
        'line', line, ...
        'checks', checks, ...
        'profile', profile, ...
        'trackData', trackData);
end
end

% ========================================================================
%  Track / vehicle loading (mirrors run_simulation / investigate_lateral_g)
% ========================================================================

function track = loadValidatorTrack(trackSpec, repoRoot)
if isa(trackSpec, 'lts.components.Track')
    track = trackSpec;
    return;
end

trackText = char(trackSpec);
if strcmpi(trackText, '2026enduro')
    track = lts.components.WaypointTrack.loadMat( ...
        fullfile(repoRoot, 'tracks', ...
        'endurance_track_grid_25ft_from_matlab_smoothed.mat'));
elseif endsWith(lower(trackText), '.mat')
    track = lts.components.WaypointTrack.loadMat(trackText);
elseif endsWith(lower(trackText), '.csv')
    track = lts.components.WaypointTrack.fromCsv(trackText);
else
    track = lts.components.TestTrack(trackText);
end
end

function config = loadVehicleConfig(configSpec)
if isa(configSpec, 'function_handle')
    config = configSpec();
elseif isa(configSpec, 'lts.vehicle.VehicleConfig')
    config = configSpec;
elseif ischar(configSpec) || isstring(configSpec)
    name = char(configSpec);
    if contains(name, '.')
        fn = str2func(name);
    else
        fn = str2func(['lts.vehicles.' name]);
    end
    config = fn();
else
    config = configSpec;
end
end

function driver = applyDriverKnobs(driver, opts)
% APPLYDRIVERKNOBS Override racing-line parameters before the plan is built.
% These are the same knobs HierarchicalOptimizer sweeps; setting them here
% lets the user validate a different line than the default.
if ~isempty(opts.RacingLineOffsetFraction)
    driver.racingLineOffsetFraction = opts.RacingLineOffsetFraction;
end
if ~isempty(opts.ApexPhase)
    driver.apexPhase = opts.ApexPhase;
end
if ~isempty(opts.CurvatureSmoothDistance)
    driver.racingLineCurvatureSmoothDistance = opts.CurvatureSmoothDistance;
end
if ~isempty(opts.OffsetSmoothDistance)
    driver.racingLineOffsetSmoothDistance = opts.OffsetSmoothDistance;
end
end

% ========================================================================
%  trackData construction (mirrors Simulator.simulate at Simulator.m:449-526)
% ========================================================================

function trackData = buildTrackData(track)
points = track.getTrackPoints();
curvature = track.getCurvature();
heading = track.getHeading();
curvature = curvature(:);
heading = heading(:);
mu = ones(size(curvature));
trackWidth = track.getTrackWidth();
[leftHalfWidth, rightHalfWidth] = track.getTrackSideWidths();

dx = diff(points(:, 1));
dy = diff(points(:, 2));
arcLen = [0; cumsum(sqrt(dx.^2 + dy.^2))];

if isprop(track, 'Closed')
    closedLoop = logical(track.Closed);
else
    closedLoop = norm(points(1, :) - points(end, :)) <= 0.05;
end

% Representative scalar half-width: mean of the per-side corridor, matching
% Simulator.simulate's trackHalfWidth (used as a fallback search radius).
trackHalfWidth = mean(leftHalfWidth + rightHalfWidth) / 2;

trackData = struct( ...
    'points', points, ...
    'arcLen', arcLen, ...
    'curvature', curvature, ...
    'mu', mu, ...
    'heading', heading, ...
    'length', arcLen(end), ...
    'trackWidth', trackWidth, ...
    'trackHalfWidth', trackHalfWidth, ...
    'trackLeftHalfWidth', leftHalfWidth(:), ...
    'trackRightHalfWidth', rightHalfWidth(:), ...
    'closedLoop', closedLoop, ...
    'baseTrackLength', arcLen(end), ...
    'totalLaps', 1, ...
    'lapBreakS', [0; arcLen(end)], ...
    'nPts', size(points, 1));
end

% ========================================================================
%  Line extraction
% ========================================================================

function line = extractLine(trackData, profile)
% EXTRACTLINE Reconstruct the racing-line polyline from the planned profile.
%
% buildRacingLine stores targetLateralError as the offset along the
% centerline normal (positive = left, negative = right). The polyline is
% rebuilt exactly as in DriverInputPlanner.buildRacingLine (:441-444):
%   linePoints = centerPoints + offset .* [-sin(h), cos(h)]
centerPoints = trackData.points;
centerHeading = trackData.heading(:);
targetOffset = profile.targetLateralError(:);

heading = unwrap(centerHeading);
normalX = -sin(heading);
normalY = cos(heading);
linePoints = centerPoints + [targetOffset .* normalX, targetOffset .* normalY];

lineDiff = diff(linePoints(:, 1)).^2 + diff(linePoints(:, 2)).^2;
lineArcLen = [0; cumsum(sqrt(lineDiff))];

line.centerPoints = centerPoints;
line.centerArcLen = trackData.arcLen(:);
line.centerCurvature = trackData.curvature(:);
line.centerHeading = centerHeading;
line.points = linePoints;
line.arcLen = lineArcLen;
line.targetOffset = targetOffset;
line.lineHeading = profile.lineHeading(:);
line.lineCurvature = profile.lineCurvature(:);
line.lineS = profile.lineS(:);
line.leftHalfWidth = trackData.trackLeftHalfWidth(:);
line.rightHalfWidth = trackData.trackRightHalfWidth(:);
line.closedLoop = trackData.closedLoop;
end

% ========================================================================
%  Validation checks
% ========================================================================

function checks = validateLine(trackData, line)
% VALIDATELINE Geometric checks: corridor containment, monotonic arc length,
% curvature reduction. Returns a struct of pass/fail flags and statistics.
targetOffset = line.targetOffset;
leftHalf = line.leftHalfWidth;
rightHalf = line.rightHalfWidth;

% Corridor containment. The line is offset along the centerline normal, so
% the per-waypoint corridor bounds apply directly to targetOffset.
leftMargin = leftHalf - targetOffset;     % >= 0 => inside on the left
rightMargin = rightHalf + targetOffset;   % >= 0 => inside on the right
minLeftMargin = min(leftMargin);
minRightMargin = min(rightMargin);
violations = nnz((leftMargin < -1e-9) | (rightMargin < -1e-9));

% Monotonic, finite arc length for the reconstructed polyline.
ds = diff(line.arcLen);
arcMonotonic = all(isfinite(ds)) && all(ds >= 0);

% Curvature comparison (absolute peaks). The racing line is expected to
% reduce peak curvature on corner apexes; report the reduction.
centerPeakKappa = max(abs(line.centerCurvature));
linePeakKappa = max(abs(line.lineCurvature));
if centerPeakKappa > eps
    curvatureReductionPct = 100 * (centerPeakKappa - linePeakKappa) / centerPeakKappa;
else
    curvatureReductionPct = 0;
end

% Detect whether the line differs from the centerline at all. If the planner
% bailed (racing disabled, steady-circle, no corners) the offset is all zero.
lineIsCenterline = max(abs(targetOffset)) < 1e-9;

% Apex detection: local extrema of targetOffset within sign-definite runs.
apexIdx = findLocalExtrema(targetOffset);

checks.minLeftMargin = minLeftMargin;
checks.minRightMargin = minRightMargin;
checks.violations = violations;
checks.corridorPassed = (violations == 0);
checks.arcMonotonic = arcMonotonic;
checks.centerPeakKappa = centerPeakKappa;
checks.linePeakKappa = linePeakKappa;
checks.curvatureReductionPct = curvatureReductionPct;
checks.lineIsCenterline = lineIsCenterline;
checks.numApexes = numel(apexIdx);
checks.apexIdx = apexIdx;
end

function idx = findLocalExtrema(values)
% FINDLOCALEXTREMA Indices of strict local extrema (sign changes in diff).
idx = [];
d = diff(values(:).');
tol = 1e-9;
nz = find(abs(d) > tol);
if isempty(nz)
    return;
end
% Sign of each non-zero difference.
s = sign(d(nz));
% A local extremum sits between a sign change in consecutive non-zero diffs.
flip = find(diff(s) ~= 0);
% Map back to the original index (the sample before the sign flip).
idx = nz(flip);
end

% ========================================================================
%  Console report
% ========================================================================

function printReport(track, driver, opts, line, checks)
fprintf('=== Racing Line Validation ===\n\n');
fprintf('Track: %s\n', trackName(track));
fprintf('  Length:        %.2f m\n', line.centerArcLen(end));
fprintf('  Waypoints:     %d\n', numel(line.centerArcLen));
fprintf('  Closed loop:   %d\n', line.closedLoop);
if isprop(track, 'getDirection') && ismethod(track, 'getDirection')
    fprintf('  Direction:     %s\n', track.getDirection());
end
fprintf('Vehicle:         %s (track width %.2f m)\n', ...
    vehicleName(driver), driver.vehicleManager.trackWidth);
fprintf('Offset fraction: %.2f\n', driver.racingLineOffsetFraction);
fprintf('Apex phase:      %.2f\n', driver.apexPhase);
fprintf('\n');

if checks.lineIsCenterline
    fprintf('Racing line:     IDENTICAL to centerline (no corners detected,\n');
    fprintf('                 racing disabled, or steady-circle track).\n');
else
    fprintf('Racing line:     generated (%d apex point(s) detected)\n', checks.numApexes);
end
fprintf('\n');

fprintf('Curvature:\n');
fprintf('  Centerline peak |k|: %.4f 1/m (radius %.1f m)\n', ...
    checks.centerPeakKappa, radiusFromKappa(checks.centerPeakKappa));
fprintf('  Racing line peak |k|: %.4f 1/m (radius %.1f m)\n', ...
    checks.linePeakKappa, radiusFromKappa(checks.linePeakKappa));
fprintf('  Peak reduction:       %.1f%%\n', checks.curvatureReductionPct);
fprintf('\n');

fprintf('Corridor containment:\n');
fprintf('  Min left margin:   %.3f m\n', checks.minLeftMargin);
fprintf('  Min right margin:  %.3f m\n', checks.minRightMargin);
fprintf('  Out-of-bounds pts: %d\n', checks.violations);
if checks.corridorPassed
    fprintf('  Status:            PASS (line stays within track limits)\n');
else
    fprintf('  Status:            FAIL (%d point(s) outside the corridor)\n', ...
        checks.violations);
end
fprintf('\n');

if ~checks.arcMonotonic
    warning('validate_racing_line:NonMonotonicArc', ...
        'Reconstructed line arc length is non-monotonic or non-finite.');
end

if ~isempty(opts.SaveFigure)
    fprintf('Figure saved:    %s\n', char(opts.SaveFigure));
end
end

function name = trackName(track)
if isprop(track, 'Name') && ~isempty(track.Name)
    name = char(track.Name);
elseif isprop(track, 'trackType')
    name = char(track.trackType);
else
    name = class(track);
end
end

function name = vehicleName(driver)
vm = driver.vehicleManager;
if isprop(vm, 'name') && ~isempty(vm.name)
    name = char(vm.name);
else
    name = class(vm);
end
end

function r = radiusFromKappa(k)
if k > eps
    r = 1 / k;
else
    r = Inf;
end
end

% ========================================================================
%  Plotting
% ========================================================================

function fig = plotValidation(track, line, checks, offsetFraction)
% PLOTVALIDATION Three panels: track map overlay, curvature vs. station,
% and lateral offset vs. station with corridor bands. Returns the figure.
fig = figure('Name', 'Racing Line Validation', 'Color', 'w', ...
    'Position', [100 100 1400 900]);

% --- Panel 1: track map with racing line overlay (large) -----------
ax1 = subplot(2, 2, [1 3]);
plotTrackMap(ax1, line, checks);
title(ax1, sprintf('%s — Racing Line Overlay (offset frac %.2f)', ...
    trackName(track), offsetFraction));

% --- Panel 2: curvature vs. station -------------------------------
ax2 = subplot(2, 2, 2);
plotCurvature(ax2, line);
title(ax2, 'Curvature vs. Station');

% --- Panel 3: lateral offset vs. station with corridor -----------
ax3 = subplot(2, 2, 4);
plotLateralOffset(ax3, line, checks);
title(ax3, 'Lateral Offset vs. Station');

% Link only the X (station) limits of the two station-vs-scalar panels.
% linkaxes is avoided because panel 1 contains a surface object whose ZData
% linkaxes tries to reconcile, which throws a shape error.
xl = [0 max([line.centerArcLen(end), 0])];
xlim(ax2, xl); xlim(ax3, xl);
sgtitle(fig, sprintf('Racing Line Validation — %s', trackName(track)), ...
    'FontSize', 13, 'FontWeight', 'bold');
end

function plotTrackMap(ax, line, checks)
% PLOTTTRACKMAP Filled track-limits corridor + centerline + racing line.
centerPoints = line.centerPoints;
leftHalf = line.leftHalfWidth;
rightHalf = line.rightHalfWidth;
heading = unwrap(line.centerHeading);
normalX = -sin(heading);
normalY = cos(heading);

% Boundary polylines (per-waypoint asymmetric corridor). Positive offset
% (left of centerline) uses the +normal direction, bounded by LeftWidth.
leftBoundary = centerPoints + [leftHalf .* normalX, leftHalf .* normalY];
rightBoundary = centerPoints - [rightHalf .* normalX, rightHalf .* normalY];

corridorX = [leftBoundary(:, 1); flipud(rightBoundary(:, 1))];
corridorY = [leftBoundary(:, 2); flipud(rightBoundary(:, 2))];

axes(ax); hold(ax, 'on');
patch(corridorX, corridorY, [0.85 0.88 0.92], ...
    'EdgeColor', 'none', 'HandleVisibility', 'off');

% Track-limit edges.
plot(ax, leftBoundary(:, 1), leftBoundary(:, 2), '-', ...
    'Color', [0.45 0.45 0.45], 'LineWidth', 1.0, 'HandleVisibility', 'off');
plot(ax, rightBoundary(:, 1), rightBoundary(:, 2), '-', ...
    'Color', [0.45 0.45 0.45], 'LineWidth', 1.0, 'HandleVisibility', 'off');

% Centerline.
plot(ax, centerPoints(:, 1), centerPoints(:, 2), '--', ...
    'Color', [0.6 0.6 0.6], 'LineWidth', 1.0, 'DisplayName', 'Centerline');

% Racing line. Drawn as a solid line, with sparse markers colored by local
% |curvature| so the map reads at a glance while staying export-safe (no
% 3D surface, which throws ZData warnings under print/saveas).
absKappa = abs(line.lineCurvature(:));
linePoints = line.points;
plot(ax, linePoints(:, 1), linePoints(:, 2), '-', ...
    'Color', [0.85 0.1 0.1], 'LineWidth', 2.0, 'DisplayName', 'Racing line');

% Sparse curvature-colored markers along the line (every Nth point).
nPts = size(linePoints, 1);
if numel(absKappa) == nPts && any(isfinite(absKappa))
    step = max(1, floor(nPts / 120));
    sampleIdx = 1:step:nPts;
    scatter(ax, linePoints(sampleIdx, 1), linePoints(sampleIdx, 2), ...
        25, absKappa(sampleIdx), 'filled', 'MarkerEdgeColor', 'k', ...
        'HandleVisibility', 'off');
    colormap(ax, parula);
    cb = colorbar(ax);
    cb.Label.String = '|Racing-line curvature| [1/m]';
    set(ax, 'CLim', [0 max(max(absKappa), 1e-6)]);
end

% Apex points.
if ~isempty(checks.apexIdx)
    plot(ax, linePoints(checks.apexIdx, 1), linePoints(checks.apexIdx, 2), ...
        'o', 'MarkerEdgeColor', [0.0 0.4 0.8], 'MarkerFaceColor', [0.4 0.7 1.0], ...
        'MarkerSize', 6, 'HandleVisibility', 'off');
end

% Start / finish.
plot(ax, centerPoints(1, 1), centerPoints(1, 2), 'ks', ...
    'MarkerSize', 10, 'MarkerFaceColor', 'k', 'DisplayName', 'Start/Finish');

axis(ax, 'equal'); grid(ax, 'on');
xlabel(ax, 'X [m]'); ylabel(ax, 'Y [m]');
legend(ax, 'Location', 'best');
end

function plotCurvature(ax, line)
s = line.centerArcLen;
centerK = line.centerCurvature;
lineK = line.lineCurvature;

axes(ax); hold(ax, 'on'); grid(ax, 'on');
plot(ax, s, centerK, '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.0, ...
    'DisplayName', 'Centerline');
plot(ax, s, lineK, '-', 'Color', [0.85 0.1 0.1], 'LineWidth', 1.5, ...
    'DisplayName', 'Racing line');
xlabel(ax, 'Station [m]'); ylabel(ax, 'Curvature [1/m]');
legend(ax, 'Location', 'best');
end

function plotLateralOffset(ax, line, checks)
% PLOTLATERALOFFSET Offset vs. station with the per-side corridor as bands.
s = line.centerArcLen;
offset = line.targetOffset;
leftLimit = line.leftHalfWidth;
rightLimit = -line.rightHalfWidth;

axes(ax); hold(ax, 'on'); grid(ax, 'on');

% Drivable corridor band (right boundary negative).
patch(ax, [s; flipud(s)], [leftLimit; flipud(rightLimit)], ...
    [0.85 0.92 0.85], 'EdgeColor', 'none', 'HandleVisibility', 'off');
plot(ax, s, leftLimit, '-', 'Color', [0.4 0.6 0.4], 'LineWidth', 1.0, ...
    'HandleVisibility', 'off');
plot(ax, s, rightLimit, '-', 'Color', [0.4 0.6 0.4], 'LineWidth', 1.0, ...
    'HandleVisibility', 'off');
plot(ax, s, zeros(size(s)), ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 0.5, ...
    'HandleVisibility', 'off');
plot(ax, s, offset, '-', 'Color', [0.85 0.1 0.1], 'LineWidth', 1.5, ...
    'DisplayName', 'Racing line offset');

% Apex points.
if ~isempty(checks.apexIdx)
    plot(ax, s(checks.apexIdx), offset(checks.apexIdx), 'o', ...
        'MarkerEdgeColor', [0.0 0.4 0.8], 'MarkerFaceColor', [0.4 0.7 1.0], ...
        'MarkerSize', 5, 'HandleVisibility', 'off');
end

xlabel(ax, 'Station [m]'); ylabel(ax, 'Lateral offset [m]');
legend(ax, 'Location', 'best');
ylimPadded = max([1, max(abs(leftLimit)), max(abs(rightLimit))]) * 1.2;
ylim(ax, [-ylimPadded ylimPadded]);
end

function writeFigurePng(fig, relOrAbsPath, repoRoot)
path = char(relOrAbsPath);
if ~is_absolute_path(path)
    path = fullfile(repoRoot, path);
end
[folder, ~, ~] = fileparts(path);
if ~isempty(folder) && ~exist(folder, 'dir')
    mkdir(folder);
end
if isempty(fig) || ~isgraphics(fig)
    error('validate_racing_line:NoFigure', ...
        'No figure to save (ShowPlots=false and no figure was created).');
end
saveas(fig, path);
fprintf('Saved figure: %s\n', path);
end

function tf = is_absolute_path(p)
tf = ~isempty(p) && (p(1) == '/' || p(1) == '\' || (length(p) >= 2 && p(2) == ':'));
end
