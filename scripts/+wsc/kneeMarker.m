function kneeMarker(ax, x, y, color, displayName, shape)
%KNEEMARKER Dashed vertical knee line plus a filled marker at (x, y).
%   Skipped unless both coordinates are finite. An empty displayName hides
%   the marker from the legend; shape defaults to a circle ('s' for the
%   coupled-CG knees).
if ~(isfinite(x) && isfinite(y))
    return;
end
if nargin < 6 || isempty(shape)
    shape = 'o';
end
xline(ax, x, '--', 'Color', color, 'LineWidth', 1.1, ...
    'HandleVisibility', 'off');
if isempty(displayName)
    plot(ax, x, y, shape, 'MarkerSize', 8, 'LineWidth', 1.5, ...
        'MarkerFaceColor', color, 'MarkerEdgeColor', 'w', ...
        'HandleVisibility', 'off');
else
    plot(ax, x, y, shape, 'MarkerSize', 8, 'LineWidth', 1.5, ...
        'MarkerFaceColor', color, 'MarkerEdgeColor', 'w', ...
        'DisplayName', displayName);
end
end
