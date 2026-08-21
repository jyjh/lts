function kneePoint(ax, x, y, color, displayName, shape)
%KNEEPOINT Filled knee marker only (no vertical line), for overlays with
%   two knees where paired dashed lines would clutter. Skipped unless both
%   coordinates are finite; empty displayName hides it from the legend.
if ~(isfinite(x) && isfinite(y))
    return;
end
if nargin < 6 || isempty(shape)
    shape = 'o';
end
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
