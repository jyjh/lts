function zeroLine(ax, color)
%ZEROLINE Dotted horizontal zero line, hidden from the legend.
yline(ax, 0, ':', 'Color', color, 'HandleVisibility', 'off');
end
