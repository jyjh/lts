classdef TrackReference
    % TRACKREFERENCE Helpers for route tiling, reference lookup, and projection.

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

        function [points, curvature, mu, heading] = repeatClosed(points, curvature, mu, heading, lapCount)
            curvature = curvature(:);
            mu = mu(:);
            heading = heading(:);
            if lapCount <= 1
                return;
            end

            basePoints = points;
            baseCurvature = curvature;
            baseMu = mu;
            baseHeading = heading;
            repeatStartIdx = 1;
            hasClosurePoint = norm(basePoints(1, :) - basePoints(end, :)) <= 0.05;
            if hasClosurePoint
                repeatStartIdx = 2;
            end

            for lapIdx = 2:lapCount %#ok<NASGU>
                points = [points; basePoints(repeatStartIdx:end, :)]; %#ok<AGROW>
                curvature = [curvature; baseCurvature(repeatStartIdx:end)]; %#ok<AGROW>
                mu = [mu; baseMu(repeatStartIdx:end)]; %#ok<AGROW>
                heading = [heading; baseHeading(repeatStartIdx:end)]; %#ok<AGROW>
            end

            if lapCount > 1 && ~hasClosurePoint
                points = [points; basePoints(1, :)]; %#ok<AGROW>
                curvature = [curvature; baseCurvature(1)]; %#ok<AGROW>
                mu = [mu; baseMu(1)]; %#ok<AGROW>
                heading = [heading; baseHeading(1)]; %#ok<AGROW>
            end
        end

        function ref = referenceAtProgress(s, x, y, trackData)
            s = max(0, min(trackData.length, s));
            idx = find(trackData.arcLen <= s, 1, 'last');
            if isempty(idx)
                idx = 1;
            end
            idx = max(1, min(idx, trackData.nPts));

            if idx < trackData.nPts && trackData.arcLen(idx+1) > trackData.arcLen(idx)
                s0 = trackData.arcLen(idx);
                s1 = trackData.arcLen(idx+1);
                t = (s - s0) / max(s1 - s0, eps);
                refPoint = (1 - t) * trackData.points(idx, :) + ...
                    t * trackData.points(idx+1, :);
                refHeading = trackData.heading(idx);
                refCurvature = trackData.curvature(idx);
                refMu = trackData.mu(idx);
            else
                refPoint = trackData.points(idx, :);
                refHeading = trackData.heading(idx);
                refCurvature = trackData.curvature(idx);
                refMu = trackData.mu(idx);
            end

            dx = x - refPoint(1);
            dy = y - refPoint(2);
            lateralError = dx * (-sin(refHeading)) + dy * cos(refHeading);
            trackHalfWidth = trackData.trackHalfWidth;
            trackLimitMargin = trackHalfWidth - abs(lateralError);
            onTrack = trackLimitMargin >= -1e-9;

            ref = struct( ...
                'idx', idx, ...
                's', s, ...
                'x', refPoint(1), ...
                'y', refPoint(2), ...
                'heading', refHeading, ...
                'curvature', refCurvature, ...
                'mu', refMu, ...
                'lateralError', lateralError, ...
                'trackWidth', trackData.trackWidth, ...
                'trackHalfWidth', trackHalfWidth, ...
                'trackLimitMargin', trackLimitMargin, ...
                'onTrack', onTrack);
        end

        function ref = projectToReference(x, y, trackData, previousIdx)
            % PROJECTTOREFERENCE Project world position to nearest track segment.
            %
            % The local search window is a speed optimization that assumes the
            % car advances mostly forward along the route. If the best local
            % segment is suspiciously far away or lies on the search boundary,
            % a full-track scan recovers from spins, large timesteps, or a
            % badly initialized reference index.
            if nargin < 4 || isempty(previousIdx) || previousIdx < 1
                previousIdx = 1;
            end

            nSegments = max(trackData.nPts - 1, 1);
            hasSegmentCache = isfield(trackData, 'segmentVectors') && ...
                isfield(trackData, 'segmentLengths') && ...
                isfield(trackData, 'segmentInvLen2');
            if hasSegmentCache
                nSegments = max(size(trackData.segmentVectors, 1), 1);
            end
            previousIdx = max(1, min(previousIdx, trackData.nPts));
            backWindow = 10;
            forwardWindow = 80;
            searchStart = max(1, min(previousIdx - backWindow, nSegments));
            searchEnd = min(nSegments, max(previousIdx + forwardWindow, searchStart));

            bestDist2 = inf;
            bestIdx = searchStart;
            bestT = 0;
            bestPoint = trackData.points(bestIdx, :);

            queryPoint = [x, y];
            for segIdx = searchStart:searchEnd
                p0 = trackData.points(segIdx, :);
                if hasSegmentCache
                    v = trackData.segmentVectors(segIdx, :);
                    invLen2 = trackData.segmentInvLen2(segIdx);
                else
                    p1 = trackData.points(segIdx + 1, :);
                    v = p1 - p0;
                    len2 = dot(v, v);
                    if len2 > eps
                        invLen2 = 1 / len2;
                    else
                        invLen2 = 0;
                    end
                end
                if invLen2 <= 0
                    t = 0;
                    projectedPoint = p0;
                else
                    t = dot(queryPoint - p0, v) * invLen2;
                    t = max(0, min(1, t));
                    projectedPoint = p0 + t * v;
                end

                dist2 = sum((queryPoint - projectedPoint).^2);
                if dist2 < bestDist2
                    bestDist2 = dist2;
                    bestIdx = segIdx;
                    bestT = t;
                    bestPoint = projectedPoint;
                end
            end

            localHitBoundary = (bestIdx == searchStart && searchStart > 1) || ...
                (bestIdx == searchEnd && searchEnd < nSegments);
            fallbackDistance = max(2 * trackData.trackHalfWidth, 5);
            if (localHitBoundary || bestDist2 > fallbackDistance^2) && ...
                    (searchStart > 1 || searchEnd < nSegments)
                bestDist2 = inf;
                bestIdx = 1;
                bestT = 0;
                bestPoint = trackData.points(1, :);
                for segIdx = 1:nSegments
                    p0 = trackData.points(segIdx, :);
                    if hasSegmentCache
                        v = trackData.segmentVectors(segIdx, :);
                        invLen2 = trackData.segmentInvLen2(segIdx);
                    else
                        p1 = trackData.points(segIdx + 1, :);
                        v = p1 - p0;
                        len2 = dot(v, v);
                        if len2 > eps
                            invLen2 = 1 / len2;
                        else
                            invLen2 = 0;
                        end
                    end
                    if invLen2 <= 0
                        t = 0;
                        projectedPoint = p0;
                    else
                        t = dot(queryPoint - p0, v) * invLen2;
                        t = max(0, min(1, t));
                        projectedPoint = p0 + t * v;
                    end

                    dist2 = sum((queryPoint - projectedPoint).^2);
                    if dist2 < bestDist2
                        bestDist2 = dist2;
                        bestIdx = segIdx;
                        bestT = t;
                        bestPoint = projectedPoint;
                    end
                end
            end

            if hasSegmentCache
                segmentLength = trackData.segmentLengths(bestIdx);
            else
                segmentLength = trackData.arcLen(bestIdx + 1) - trackData.arcLen(bestIdx);
            end
            refS = trackData.arcLen(bestIdx) + bestT * max(segmentLength, 0);
            refS = max(0, min(trackData.length, refS));

            if segmentLength > eps
                if hasSegmentCache
                    v = trackData.segmentVectors(bestIdx, :);
                else
                    p0 = trackData.points(bestIdx, :);
                    p1 = trackData.points(bestIdx + 1, :);
                    v = p1 - p0;
                end
                refHeading = atan2(v(2), v(1));
            else
                refHeading = trackData.heading(bestIdx);
            end

            interpIdx = min(bestIdx + 1, trackData.nPts);
            refCurvature = (1 - bestT) * trackData.curvature(bestIdx) + ...
                bestT * trackData.curvature(interpIdx);
            refMu = (1 - bestT) * trackData.mu(bestIdx) + ...
                bestT * trackData.mu(interpIdx);

            dx = x - bestPoint(1);
            dy = y - bestPoint(2);
            lateralError = dx * (-sin(refHeading)) + dy * cos(refHeading);
            refIdx = min(bestIdx + double(bestT > 0.5), trackData.nPts);
            trackHalfWidth = trackData.trackHalfWidth;
            trackLimitMargin = trackHalfWidth - abs(lateralError);
            onTrack = trackLimitMargin >= -1e-9;

            ref = struct( ...
                'idx', refIdx, ...
                's', refS, ...
                'x', bestPoint(1), ...
                'y', bestPoint(2), ...
                'heading', refHeading, ...
                'curvature', refCurvature, ...
                'mu', refMu, ...
                'lateralError', lateralError, ...
                'trackWidth', trackData.trackWidth, ...
                'trackHalfWidth', trackHalfWidth, ...
                'trackLimitMargin', trackLimitMargin, ...
                'onTrack', onTrack);
        end

        function trackData = precomputeSegments(trackData)
            if ~isfield(trackData, 'points') || size(trackData.points, 1) < 2
                trackData.segmentVectors = zeros(0, 2);
                trackData.segmentLengths = zeros(0, 1);
                trackData.segmentInvLen2 = zeros(0, 1);
                return;
            end
            segmentVectors = diff(trackData.points, 1, 1);
            segmentLengths = hypot(segmentVectors(:,1), segmentVectors(:,2));
            len2 = segmentLengths.^2;
            segmentInvLen2 = zeros(size(len2));
            valid = len2 > eps;
            segmentInvLen2(valid) = 1 ./ len2(valid);
            trackData.segmentVectors = segmentVectors;
            trackData.segmentLengths = segmentLengths;
            trackData.segmentInvLen2 = segmentInvLen2;
        end
    end
end
