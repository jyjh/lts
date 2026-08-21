classdef TrackReference
    % Track tiling, lookup, and world-position projection.

    methods (Static)
        function laps = warmupLaps(track)
            laps = 0;
            if ismethod(track, 'getWarmupLaps')
                laps = track.getWarmupLaps();
            elseif isprop(track, 'warmupLaps')
                laps = track.warmupLaps;
            end
            laps = max(0, round(laps));
        end

        function laps = recordedLaps(track)
            laps = 1;
            if ismethod(track, 'getRecordedLaps')
                laps = track.getRecordedLaps();
            elseif isprop(track, 'recordedLaps')
                laps = track.recordedLaps;
            end
            laps = max(1, round(laps));
        end

        function closed = isClosedLoop(track, points)
            closed = norm(points(1, :) - points(end, :)) <= 0.05;
            if ismethod(track, 'isClosedLoop')
                closed = track.isClosedLoop();
            elseif isprop(track, 'closedLoop')
                closed = track.closedLoop;
            elseif isprop(track, 'Closed')
                closed = track.Closed;
            end
            closed = logical(closed);
        end

        function [points, curvature, mu, heading] = repeatClosed( ...
                points, curvature, mu, heading, lapCount)
            curvature = curvature(:);
            mu = mu(:);
            heading = heading(:);
            lapCount = max(1, round(lapCount));

            if norm(points(1, :) - points(end, :)) > 0.05
                points = [points; points(1, :)];
                curvature = [curvature; curvature(1)];
                mu = [mu; mu(1)];
                heading = [heading; heading(1)];
            end
            if lapCount > 1
                points = [points; repmat(points(2:end, :), lapCount - 1, 1)];
                curvature = [curvature; repmat(curvature(2:end), lapCount - 1, 1)];
                mu = [mu; repmat(mu(2:end), lapCount - 1, 1)];
                heading = [heading; repmat(heading(2:end), lapCount - 1, 1)];
            end
        end

        function values = repeatClosedColumn(values, hasClosurePoint, lapCount)
            values = values(:);
            lapCount = max(1, round(lapCount));
            if ~hasClosurePoint
                values = [values; values(1)];
            end
            if lapCount > 1
                values = [values; repmat(values(2:end), lapCount - 1, 1)];
            end
        end

        function ref = projectToReference(x, y, trackData, previousIdx)
            if nargin < 4 || isempty(previousIdx) || previousIdx < 1
                previousIdx = 1;
            end

            cached = isfield(trackData, 'segmentVectors') && ...
                isfield(trackData, 'segmentLengths') && ...
                isfield(trackData, 'segmentInvLen2');
            if cached
                nSegments = size(trackData.segmentVectors, 1);
            else
                nSegments = trackData.nPts - 1;
            end
            previousIdx = max(1, min(previousIdx, trackData.nPts));
            searchStart = max(1, min(previousIdx - 10, nSegments));
            searchEnd = min(nSegments, max(previousIdx + 80, searchStart));

            [bestDist2, bestIdx, bestT, bestPoint] = ...
                lts.simulation.TrackReference.nearestSegmentProjection( ...
                    x, y, trackData, searchStart, searchEnd, cached);
            hitBoundary = (bestIdx == searchStart && searchStart > 1) || ...
                (bestIdx == searchEnd && searchEnd < nSegments);
            tooFar = bestDist2 > max(2 * trackData.trackHalfWidth, 5)^2;
            if (hitBoundary || tooFar) && (searchStart > 1 || searchEnd < nSegments)
                [~, bestIdx, bestT, bestPoint] = ...
                    lts.simulation.TrackReference.nearestSegmentProjection( ...
                        x, y, trackData, 1, nSegments, cached);
            end

            if cached
                segmentLength = trackData.segmentLengths(bestIdx);
                segment = trackData.segmentVectors(bestIdx, :);
            else
                segmentLength = trackData.arcLen(bestIdx + 1) - trackData.arcLen(bestIdx);
                segment = trackData.points(bestIdx + 1, :) - trackData.points(bestIdx, :);
            end
            refS = max(0, min(trackData.length, ...
                trackData.arcLen(bestIdx) + bestT * max(segmentLength, 0)));
            if segmentLength > eps
                refHeading = atan2(segment(2), segment(1));
            else
                refHeading = trackData.heading(bestIdx);
            end

            nextIdx = min(bestIdx + 1, trackData.nPts);
            curvature = (1 - bestT) * trackData.curvature(bestIdx) + ...
                bestT * trackData.curvature(nextIdx);
            mu = (1 - bestT) * trackData.mu(bestIdx) + bestT * trackData.mu(nextIdx);
            [left, right] = lts.simulation.TrackReference.sideHalfWidthsInterp( ...
                trackData, bestIdx, nextIdx, bestT);
            refIdx = min(bestIdx + double(bestT > 0.5), trackData.nPts);
            ref = lts.simulation.TrackReference.makeReference( ...
                refIdx, refS, bestPoint, refHeading, curvature, mu, ...
                x, y, trackData, left, right);
        end

        function [left, right] = sideHalfWidthsAt(trackData, idx)
            if isfield(trackData, 'trackLeftHalfWidth') && ...
                    isfield(trackData, 'trackRightHalfWidth') && ...
                    ~isempty(trackData.trackLeftHalfWidth)
                idx = max(1, min(idx, numel(trackData.trackLeftHalfWidth)));
                left = trackData.trackLeftHalfWidth(idx);
                right = trackData.trackRightHalfWidth(idx);
            else
                left = trackData.trackHalfWidth;
                right = left;
            end
        end

        function [left, right] = sideHalfWidthsInterp(trackData, idx0, idx1, t)
            if isfield(trackData, 'trackLeftHalfWidth') && ...
                    isfield(trackData, 'trackRightHalfWidth') && ...
                    ~isempty(trackData.trackLeftHalfWidth)
                l = trackData.trackLeftHalfWidth;
                r = trackData.trackRightHalfWidth;
                idx0 = max(1, min(idx0, numel(l)));
                idx1 = max(1, min(idx1, numel(l)));
                left = (1 - t) * l(idx0) + t * l(idx1);
                right = (1 - t) * r(idx0) + t * r(idx1);
            else
                left = trackData.trackHalfWidth;
                right = left;
            end
        end

        function margin = sideMargin(left, right, lateralError, fallback)
            if lateralError >= 0
                halfWidth = left;
            else
                halfWidth = right;
            end
            if isempty(halfWidth) || ~all(isfinite(halfWidth)) || any(halfWidth <= 0)
                halfWidth = fallback;
            end
            margin = halfWidth - abs(lateralError);
        end

        function trackData = precomputeSegments(trackData)
            vectors = diff(trackData.points, 1, 1);
            lengths = hypot(vectors(:, 1), vectors(:, 2));
            invLen2 = zeros(size(lengths));
            valid = lengths > eps;
            invLen2(valid) = 1 ./ lengths(valid).^2;
            trackData.segmentVectors = vectors;
            trackData.segmentLengths = lengths;
            trackData.segmentInvLen2 = invLen2;
        end

        function [bestDist2, bestIdx, bestT, bestPoint] = ...
                nearestSegmentProjection(x, y, trackData, searchStart, searchEnd, cached)
            indices = searchStart:searchEnd;
            p0 = trackData.points(indices, :);
            if cached
                vectors = trackData.segmentVectors(indices, :);
                invLen2 = trackData.segmentInvLen2(indices);
            else
                vectors = trackData.points(indices + 1, :) - p0;
                len2 = sum(vectors.^2, 2);
                invLen2 = zeros(size(len2));
                valid = len2 > eps;
                invLen2(valid) = 1 ./ len2(valid);
            end

            offset = [x - p0(:, 1), y - p0(:, 2)];
            t = sum(offset .* vectors, 2) .* invLen2(:);
            t(invLen2 <= 0) = 0;
            t = lts.util.saturate(t);
            points = p0 + t .* vectors;
            dist2 = sum(([x, y] - points).^2, 2);
            [bestDist2, relativeIdx] = min(dist2);
            bestIdx = indices(relativeIdx);
            bestT = t(relativeIdx);
            bestPoint = points(relativeIdx, :);
        end

        function ref = makeReference(idx, s, point, heading, curvature, mu, ...
                x, y, trackData, left, right)
            offset = [x, y] - point;
            lateralError = offset(1) * -sin(heading) + offset(2) * cos(heading);
            margin = lts.simulation.TrackReference.sideMargin( ...
                left, right, lateralError, trackData.trackHalfWidth);
            ref = struct( ...
                'idx', idx, 's', s, 'x', point(1), 'y', point(2), ...
                'heading', heading, 'curvature', curvature, 'mu', mu, ...
                'lateralError', lateralError, ...
                'trackWidth', trackData.trackWidth, ...
                'trackHalfWidth', trackData.trackHalfWidth, ...
                'leftHalfWidth', left, 'rightHalfWidth', right, ...
                'trackLimitMargin', margin, 'onTrack', margin >= -1e-9);
        end
    end
end
