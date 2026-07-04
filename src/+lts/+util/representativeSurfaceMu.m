function mu = representativeSurfaceMu(track)
% REPRESENTATIVESURFACEMU Return a scalar track friction value.
mu = 1.2;
if isempty(track)
    return;
end

try
    points = track.getTrackPoints();
    if size(points, 2) >= 4
        values = points(:, 4);
        finiteValues = values(isfinite(values));
        if ~isempty(finiteValues)
            mu = median(finiteValues);
        end
    end
catch
    % Keep the procedural dry-track default when metadata is unavailable.
end
end
