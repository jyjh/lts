function metrics = plot_correlation_position_overlay(replayCsv, simCsv, outputPng, options)
%PLOT_CORRELATION_POSITION_OVERLAY Overlay raw GPS path and simulated path.
%
%   metrics = plot_correlation_position_overlay(replayCsv, simCsv, outputPng)
%   converts raw replay GPS latitude/longitude to a local East/North frame,
%   rotates the simulator X/Y path by the first logged GPS course, and writes
%   an overlay plus position-error plot.
%
%   metrics = plot_correlation_position_overlay(..., 'RawTimeOffsetS', dt)
%   compares sim time zero against raw GPS time dt. This is useful when fast
%   logged controls appear time-advanced relative to GPS position/course.

arguments
    replayCsv (1, 1) string
    simCsv (1, 1) string
    outputPng (1, 1) string = "exports/correlation_position_overlay.png"
    options.RawTimeOffsetS (1, 1) double = 0
    options.InitialHeadingMode (1, 1) string = "gps_course"
end

raw = readtable(replayCsv, 'VariableNamingRule', 'preserve');
sim = readtable(simCsv, 'VariableNamingRule', 'preserve');

rawTime = raw.("time_s");
lat = raw.("gps_lat_deg");
lon = raw.("gps_lon_deg");
validGps = isfinite(rawTime) & isfinite(lat) & isfinite(lon);
if nnz(validGps) < 2
    error("plot_correlation_position_overlay:MissingGps", ...
        "Replay CSV must contain at least two finite GPS latitude/longitude samples.");
end

rawTime = rawTime(validGps);
lat = lat(validGps);
lon = lon(validGps);
rawTimeOffsetS = max(0, double(options.RawTimeOffsetS));
if rawTimeOffsetS >= rawTime(end) - rawTime(1)
    error("plot_correlation_position_overlay:InvalidRawTimeOffset", ...
        "RawTimeOffsetS must be smaller than the replay GPS duration.");
end
rawStartTime = rawTime(1) + rawTimeOffsetS;

lat0 = lat(1);
lon0 = lon(1);
earthRadiusM = 6378137.0;
rawEast = deg2rad(lon - lon0) .* earthRadiusM .* cosd(lat0);
rawNorth = deg2rad(lat - lat0) .* earthRadiusM;

initialHeadingMode = lower(options.InitialHeadingMode);
if initialHeadingMode == "path_heading"
    initialCourse = pathHeadingAtTime(rawTime, rawEast, rawNorth, rawStartTime, 0.30);
elseif initialHeadingMode == "gps_course"
    initialCourse = gpsCourseHeadingAtTime(raw, validGps, rawTime, rawStartTime);
else
    error("plot_correlation_position_overlay:InvalidHeadingMode", ...
        "InitialHeadingMode must be 'gps_course' or 'path_heading'.");
end
if ~isfinite(initialCourse)
    initialCourse = pathHeadingAtTime(rawTime, rawEast, rawNorth, rawStartTime, 0.30);
end
if ~isfinite(initialCourse)
    initialCourse = atan2(rawNorth(end) - rawNorth(1), rawEast(end) - rawEast(1));
end

rawEast0 = interpFinite(rawTime, rawEast, rawStartTime);
rawNorth0 = interpFinite(rawTime, rawNorth, rawStartTime);
rawEast = rawEast - rawEast0;
rawNorth = rawNorth - rawNorth0;
rawPlotMask = rawTime >= rawStartTime;
rawPlotEast = [0; rawEast(rawPlotMask)];
rawPlotNorth = [0; rawNorth(rawPlotMask)];

simTime = sim.("Time (s)");
simX = sim.("X");
simY = sim.("Y");
validSim = isfinite(simTime) & isfinite(simX) & isfinite(simY);
simTime = simTime(validSim);
simX = simX(validSim);
simY = simY(validSim);
if numel(simTime) < 2
    error("plot_correlation_position_overlay:MissingSimPosition", ...
        "Simulation CSV must contain at least two finite X/Y samples.");
end

simX = simX - simX(1);
simY = simY - simY(1);
simEast = simX .* cos(initialCourse) - simY .* sin(initialCourse);
simNorth = simX .* sin(initialCourse) + simY .* cos(initialCourse);

queryRawTime = simTime + rawStartTime;
rawEastAtSim = interp1(rawTime, rawEast, queryRawTime, 'linear', NaN);
rawNorthAtSim = interp1(rawTime, rawNorth, queryRawTime, 'linear', NaN);
posErrEast = simEast - rawEastAtSim;
posErrNorth = simNorth - rawNorthAtSim;
posErrMag = hypot(posErrEast, posErrNorth);
validErr = isfinite(posErrMag);

metrics = struct();
metrics.outputPng = char(outputPng);
metrics.replayCsv = char(replayCsv);
metrics.simCsv = char(simCsv);
metrics.initialCourseDeg = rad2deg(initialCourse);
metrics.initialHeadingMode = char(initialHeadingMode);
metrics.rawTimeOffsetS = rawTimeOffsetS;
metrics.sampleCount = nnz(validErr);
metrics.positionRmseM = sqrt(mean(posErrMag(validErr).^2));
metrics.positionMeanM = mean(posErrMag(validErr));
metrics.positionMaxM = max(posErrMag(validErr));
metrics.finalPositionErrorM = posErrMag(find(validErr, 1, 'last'));

fig = figure('Color', 'w', 'Position', [100, 100, 1300, 850]);
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

axPath = nexttile(layout, 1, [2, 1]);
styleAxis(axPath);
plot(axPath, rawPlotEast, rawPlotNorth, 'Color', [0.05 0.05 0.05], ...
    'LineWidth', 1.5, 'DisplayName', 'Raw GPS path');
hold(axPath, 'on');
plot(axPath, simEast, simNorth, 'Color', [0.85 0.10 0.10], ...
    'LineWidth', 1.2, 'DisplayName', 'Simulation path');
plot(axPath, rawPlotEast(1), rawPlotNorth(1), 'o', 'MarkerSize', 7, ...
    'MarkerFaceColor', [0.05 0.05 0.05], 'MarkerEdgeColor', 'none', ...
    'DisplayName', 'Raw start');
plot(axPath, simEast(1), simNorth(1), 's', 'MarkerSize', 7, ...
    'MarkerFaceColor', [0.85 0.10 0.10], 'MarkerEdgeColor', 'none', ...
    'DisplayName', 'Sim start');
plot(axPath, rawPlotEast(end), rawPlotNorth(end), '^', 'MarkerSize', 7, ...
    'MarkerFaceColor', [0.05 0.05 0.05], 'MarkerEdgeColor', 'none', ...
    'DisplayName', 'Raw end');
plot(axPath, simEast(end), simNorth(end), 'd', 'MarkerSize', 7, ...
    'MarkerFaceColor', [0.85 0.10 0.10], 'MarkerEdgeColor', 'none', ...
    'DisplayName', 'Sim end');
hold(axPath, 'off');
axis(axPath, 'equal');
axis(axPath, 'tight');
grid(axPath, 'on');
xlabel(axPath, 'East from aligned raw GPS start (m)');
ylabel(axPath, 'North from aligned raw GPS start (m)');
title(axPath, sprintf('Raw GPS vs simulation path | raw offset %.3f s | heading %.1f deg', ...
    metrics.rawTimeOffsetS, metrics.initialCourseDeg));
legend(axPath, 'Location', 'bestoutside');

axErr = nexttile(layout);
styleAxis(axErr);
plot(axErr, simTime(validErr), posErrMag(validErr), 'Color', [0.10 0.25 0.75], ...
    'LineWidth', 1.2, 'DisplayName', 'Position error magnitude');
grid(axErr, 'on');
xlabel(axErr, 'Time (s)');
ylabel(axErr, 'Position difference (m)');
title(axErr, sprintf('Error magnitude: RMSE %.1f m, mean %.1f m, max %.1f m', ...
    metrics.positionRmseM, metrics.positionMeanM, metrics.positionMaxM));
legend(axErr, 'Location', 'best');

axComponents = nexttile(layout);
styleAxis(axComponents);
plot(axComponents, simTime(validErr), posErrEast(validErr), ...
    'Color', [0.10 0.55 0.35], 'LineWidth', 1.0, ...
    'DisplayName', 'East error');
hold(axComponents, 'on');
plot(axComponents, simTime(validErr), posErrNorth(validErr), ...
    'Color', [0.70 0.35 0.10], 'LineWidth', 1.0, ...
    'DisplayName', 'North error');
yline(axComponents, 0, ':', 'Color', [0.45 0.45 0.45], ...
    'HandleVisibility', 'off');
hold(axComponents, 'off');
grid(axComponents, 'on');
xlabel(axComponents, 'Time (s)');
ylabel(axComponents, 'Signed error (m)');
title(axComponents, sprintf('Components | final error %.1f m', ...
    metrics.finalPositionErrorM));
legend(axComponents, 'Location', 'best');

% Add a faint magnitude trace behind the component plot for quick scale checks.
hold(axErr, 'on');
plot(axErr, simTime(validErr), abs(posErrEast(validErr)), '--', ...
    'Color', [0.10 0.55 0.35], 'LineWidth', 0.9, ...
    'DisplayName', '|East error|');
plot(axErr, simTime(validErr), abs(posErrNorth(validErr)), '--', ...
    'Color', [0.70 0.35 0.10], 'LineWidth', 0.9, ...
    'DisplayName', '|North error|');
hold(axErr, 'off');
legend(axErr, 'Location', 'best');

outputDir = fileparts(outputPng);
if strlength(outputDir) > 0 && ~isfolder(outputDir)
    mkdir(outputDir);
end
styleFigure(fig);
exportgraphics(fig, outputPng, 'Resolution', 160, 'BackgroundColor', 'white');
close(fig);

fprintf('Position overlay written: %s\n', outputPng);
fprintf('Initial heading: %.3f deg (%s)\n', ...
    metrics.initialCourseDeg, metrics.initialHeadingMode);
fprintf('Raw GPS time offset: %.3f s\n', metrics.rawTimeOffsetS);
fprintf('Samples: %d\n', metrics.sampleCount);
fprintf('Position error RMSE: %.3f m | mean: %.3f m | max: %.3f m | final: %.3f m\n', ...
    metrics.positionRmseM, metrics.positionMeanM, metrics.positionMaxM, ...
    metrics.finalPositionErrorM);
end

function value = firstFinite(tableData, name)
value = NaN;
if ~ismember(name, tableData.Properties.VariableNames)
    return;
end
values = tableData.(name);
idx = find(isfinite(values), 1, 'first');
if ~isempty(idx)
    value = values(idx);
end
end

function heading = gpsCourseHeadingAtTime(tableData, validGps, rawTime, queryTime)
heading = NaN;
if ~ismember("gps_course_rad", tableData.Properties.VariableNames)
    return;
end

course = tableData.("gps_course_rad");
course = course(validGps);
course = pi / 2 - course;
course = unwrapFinite(course);
heading = interpFinite(rawTime, course, queryTime);
end

function heading = pathHeadingAtTime(time, east, north, queryTime, windowS)
halfWindow = windowS / 2;
t0 = max(time(1), queryTime - halfWindow);
t1 = min(time(end), queryTime + halfWindow);
east0 = interpFinite(time, east, t0);
north0 = interpFinite(time, north, t0);
east1 = interpFinite(time, east, t1);
north1 = interpFinite(time, north, t1);
if hypot(east1 - east0, north1 - north0) > 0.05
    heading = atan2(north1 - north0, east1 - east0);
else
    heading = NaN;
end
end

function values = unwrapFinite(values)
values = double(values(:));
finiteMask = isfinite(values);
if any(finiteMask)
    values(finiteMask) = unwrap(values(finiteMask));
end
end

function value = interpFinite(x, y, query)
value = NaN;
x = double(x(:));
y = double(y(:));
keep = isfinite(x) & isfinite(y);
if ~any(keep)
    return;
end
x = x(keep);
y = y(keep);
[x, ia] = unique(x, 'stable');
y = y(ia);
if numel(x) == 1
    value = y(1);
else
    value = interp1(x, y, query, 'linear', NaN);
end
end

function styleAxis(ax)
set(ax, 'Color', 'w', ...
    'XColor', [0.10 0.10 0.10], ...
    'YColor', [0.10 0.10 0.10], ...
    'GridColor', [0.72 0.72 0.72], ...
    'MinorGridColor', [0.84 0.84 0.84], ...
    'FontName', 'Arial');
end

function styleFigure(fig)
axesHandles = findall(fig, 'Type', 'axes');
for i = 1:numel(axesHandles)
    styleAxis(axesHandles(i));
end

textHandles = findall(fig, 'Type', 'text');
set(textHandles, 'Color', [0.10 0.10 0.10]);

legendHandles = findall(fig, 'Type', 'legend');
set(legendHandles, 'Color', 'w', ...
    'TextColor', [0.10 0.10 0.10], ...
    'EdgeColor', [0.75 0.75 0.75]);
end
