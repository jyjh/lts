function referenceLine(ax, x, color)
%REFERENCELINE Dotted vertical reference marker (skipped when not finite).
if isfinite(x)
    xline(ax, x, ':', 'Color', color, 'LineWidth', 1.1, ...
        'HandleVisibility', 'off');
end
end
