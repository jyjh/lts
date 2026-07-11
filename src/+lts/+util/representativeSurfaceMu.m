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
catch err
    % Keep the procedural dry-track default, but surface the failure. Every
    % lts.components.Track implements getTrackPoints, so an error here means
    % the track object misbehaved rather than that metadata is simply absent.
    warning('lts_util_representativeSurfaceMu:SurfaceMuUnavailable', ...
        'Could not read track surface friction; using default mu=%.2f. Cause: %s', ...
        mu, err.message);
end
end
