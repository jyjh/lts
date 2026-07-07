function metrics = plot_correlation_position_overlay(replayCsv, simCsv, outputPng)
%PLOT_CORRELATION_POSITION_OVERLAY Overlay raw GPS path and simulated path.
%
%   metrics = plot_correlation_position_overlay(replayCsv, simCsv, outputPng)
%   converts raw replay GPS latitude/longitude to a local East/North frame,
%   rotates the simulator X/Y path by the first logged GPS course, and writes
%   an overlay plus position-error plot.

arguments
    replayCsv (1, 1) string
    simCsv (1, 1) string
    outputPng (1, 1) string = "exports/correlation_position_overlay.png"
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

lat0 = lat(1);
lon0 = lon(1);
earthRadiusM = 6378137.0;
rawEast = deg2rad(lon - lon0) .* earthRadiusM .* cosd(lat0);
rawNorth = deg2rad(lat - lat0) .* earthRadiusM;

initialCourse = firstFinite(raw, "gps_course_rad");
if ~isfinite(initialCourse)
    initialCourse = atan2(rawNorth(end) - rawNorth(1), rawEast(end) - rawEast(1));
end

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

rawEastAtSim = interp1(rawTime, rawEast, simTime, 'linear', NaN);
rawNorthAtSim = interp1(rawTime, rawNorth, simTime, 'linear', NaN);
posErrEast = simEast - rawEastAtSim;
posErrNorth = simNorth - rawNorthAtSim;
posErrMag = hypot(posErrEast, posErrNorth);
validErr = isfinite(posErrMag);

metrics = struct();
metrics.outputPng = char(outputPng);
metrics.replayCsv = char(replayCsv);
metrics.simCsv = char(simCsv);
metrics.initialCourseDeg = rad2deg(initialCourse);
metrics.sampleCount = nnz(validErr);
metrics.positionRmseM = sqrt(mean(posErrMag(validErr).^2));
metrics.positionMeanM = mean(posErrMag(validErr));
metrics.positionMaxM = max(posErrMag(validErr));
metrics.finalPositionErrorM = posErrMag(find(validErr, 1, 'last'));

fig = figure('Color', 'w', 'Position', [100, 100, 1300, 850]);
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

axPath = nexttile(layout, 1, [2, 1]);
styleAxis(axPath);
plot(axPath, rawEast, rawNorth, 'Color', [0.05 0.05 0.05], ...
    'LineWidth', 1.5, 'DisplayName', 'Raw GPS path');
hold(axPath, 'on');
plot(axPath, simEast, simNorth, 'Color', [0.85 0.10 0.10], ...
    'LineWidth', 1.2, 'DisplayName', 'Simulation path');
plot(axPath, rawEast(1), rawNorth(1), 'o', 'MarkerSize', 7, ...
    'MarkerFaceColor', [0.05 0.05 0.05], 'MarkerEdgeColor', 'none', ...
    'DisplayName', 'Raw start');
plot(axPath, simEast(1), simNorth(1), 's', 'MarkerSize', 7, ...
    'MarkerFaceColor', [0.85 0.10 0.10], 'MarkerEdgeColor', 'none', ...
    'DisplayName', 'Sim start');
plot(axPath, rawEast(end), rawNorth(end), '^', 'MarkerSize', 7, ...
    'MarkerFaceColor', [0.05 0.05 0.05], 'MarkerEdgeColor', 'none', ...
    'DisplayName', 'Raw end');
plot(axPath, simEast(end), simNorth(end), 'd', 'MarkerSize', 7, ...
    'MarkerFaceColor', [0.85 0.10 0.10], 'MarkerEdgeColor', 'none', ...
    'DisplayName', 'Sim end');
hold(axPath, 'off');
axis(axPath, 'equal');
axis(axPath, 'tight');
grid(axPath, 'on');
xlabel(axPath, 'East from raw GPS start (m)');
ylabel(axPath, 'North from raw GPS start (m)');
title(axPath, sprintf('Raw GPS vs simulation path | initial GPS course %.1f deg', ...
    metrics.initialCourseDeg));
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
fprintf('Initial course: %.3f deg\n', metrics.initialCourseDeg);
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
