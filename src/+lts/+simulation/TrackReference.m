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
            lapCount = max(1, round(lapCount));

            basePoints = points;
            baseCurvature = curvature;
            baseMu = mu;
            baseHeading = heading;
            hasClosurePoint = norm(basePoints(1, :) - basePoints(end, :)) <= 0.05;
            if ~hasClosurePoint
                % Track preprocessing removes a duplicate endpoint. Restore
                % it here so the final-to-initial segment is simulated even
                % for a single requested lap.
                basePoints = [basePoints; basePoints(1, :)];
                baseCurvature = [baseCurvature; baseCurvature(1)];
                baseMu = [baseMu; baseMu(1)];
                baseHeading = [baseHeading; baseHeading(1)];
            end

            points = basePoints;
            curvature = baseCurvature;
            mu = baseMu;
            heading = baseHeading;
            for lapIdx = 2:lapCount %#ok<NASGU>
                points = [points; basePoints(2:end, :)]; %#ok<AGROW>
                curvature = [curvature; baseCurvature(2:end)]; %#ok<AGROW>
                mu = [mu; baseMu(2:end)]; %#ok<AGROW>
                heading = [heading; baseHeading(2:end)]; %#ok<AGROW>
            end
        end

        function tiled = repeatClosedColumn(values, nBasePoints, hasClosurePoint, lapCount)
            % REPEATCLOSEDCOLUMN Tile a per-waypoint column across laps with the
            % same closure-point + repeat rule as repeatClosed, so any
            % per-waypoint track attribute (e.g. left/right half-widths) stays
            % index-aligned with the tiled centerline points.
            %
            %   values           - Nx1 base-lap values, one per base waypoint
            %   nBasePoints      - number of base waypoints (before closure
            %                      point restoration) matching points(1,:)
            %   hasClosurePoint  - true if the base centerline already closes
            %                      (matches repeatClosed's test)
            %   lapCount         - number of laps to tile
            values = values(:);
            lapCount = max(1, round(lapCount));
            base = values;
            if ~hasClosurePoint
                % Restore the removed closure point so tiling covers the
                % final-to-initial segment, exactly as repeatClosed does for
                % points/curvature/mu/heading.
                base = [base; base(1)];
            end
            tiled = base;
            for lapIdx = 2:lapCount %#ok<NASGU>
                tiled = [tiled; base(2:end)]; %#ok<AGROW>
            end
            tiled = tiled(:);
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
            [leftHalfWidth, rightHalfWidth] = ...
                lts.simulation.TrackReference.sideHalfWidthsAt(trackData, idx);
            trackLimitMargin = lts.simulation.TrackReference.sideMargin( ...
                leftHalfWidth, rightHalfWidth, lateralError, trackHalfWidth);
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
                'leftHalfWidth', leftHalfWidth, ...
                'rightHalfWidth', rightHalfWidth, ...
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

            [bestDist2, bestIdx, bestT, bestPoint] = ...
                lts.simulation.TrackReference.nearestSegmentProjection( ...
                    x, y, trackData, searchStart, searchEnd, hasSegmentCache);

            localHitBoundary = (bestIdx == searchStart && searchStart > 1) || ...
                (bestIdx == searchEnd && searchEnd < nSegments);
            fallbackDistance = max(2 * trackData.trackHalfWidth, 5);
            if (localHitBoundary || bestDist2 > fallbackDistance^2) && ...
                    (searchStart > 1 || searchEnd < nSegments)
                [bestDist2, bestIdx, bestT, bestPoint] = ...
                    lts.simulation.TrackReference.nearestSegmentProjection( ...
                        x, y, trackData, 1, nSegments, hasSegmentCache);
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
            [leftHalfWidth, rightHalfWidth] = ...
                lts.simulation.TrackReference.sideHalfWidthsInterp( ...
                    trackData, bestIdx, interpIdx, bestT);

            dx = x - bestPoint(1);
            dy = y - bestPoint(2);
            lateralError = dx * (-sin(refHeading)) + dy * cos(refHeading);
            refIdx = min(bestIdx + double(bestT > 0.5), trackData.nPts);
            trackHalfWidth = trackData.trackHalfWidth;
            trackLimitMargin = lts.simulation.TrackReference.sideMargin( ...
                leftHalfWidth, rightHalfWidth, lateralError, trackHalfWidth);
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
                'leftHalfWidth', leftHalfWidth, ...
                'rightHalfWidth', rightHalfWidth, ...
                'trackLimitMargin', trackLimitMargin, ...
                'onTrack', onTrack);
        end

        function [leftHalfWidth, rightHalfWidth] = sideHalfWidthsAt(trackData, idx)
            % SIDEHALFWIDTHSAT Per-side half-widths at a single waypoint index.
            % Returns the trackLeftHalfWidth/trackRightHalfWidth values when the
            % track data carries a per-waypoint corridor, else falls back to
            % the symmetric scalar trackHalfWidth on both sides. This keeps the
            % off-track margin well-defined for synthetic/test trackData that
            % only sets the scalar field.
            trackHalfWidth = trackData.trackHalfWidth;
            if isfield(trackData, 'trackLeftHalfWidth') && ...
                    isfield(trackData, 'trackRightHalfWidth') && ...
                    ~isempty(trackData.trackLeftHalfWidth)
                idx = max(1, min(idx, numel(trackData.trackLeftHalfWidth)));
                leftHalfWidth = trackData.trackLeftHalfWidth(idx);
                rightHalfWidth = trackData.trackRightHalfWidth(idx);
            else
                leftHalfWidth = trackHalfWidth;
                rightHalfWidth = trackHalfWidth;
            end
        end

        function [leftHalfWidth, rightHalfWidth] = sideHalfWidthsInterp( ...
                trackData, idx0, idx1, t)
            % SIDEHALFWIDTHSINTERP Linearly interpolated per-side half-widths
            % across segment [idx0, idx1] at parameter t, matching how
            % projectToReference interpolates curvature and mu.
            trackHalfWidth = trackData.trackHalfWidth;
            if isfield(trackData, 'trackLeftHalfWidth') && ...
                    isfield(trackData, 'trackRightHalfWidth') && ...
                    ~isempty(trackData.trackLeftHalfWidth)
                l = trackData.trackLeftHalfWidth;
                r = trackData.trackRightHalfWidth;
                idx0 = max(1, min(idx0, numel(l)));
                idx1 = max(1, min(idx1, numel(l)));
                leftHalfWidth = (1 - t) * l(idx0) + t * l(idx1);
                rightHalfWidth = (1 - t) * r(idx0) + t * r(idx1);
            else
                leftHalfWidth = trackHalfWidth;
                rightHalfWidth = trackHalfWidth;
            end
        end

        function margin = sideMargin(leftHalfWidth, rightHalfWidth, lateralError, fallback)
            % SIDEMARGIN Distance to the nearer edge for the side the car is on.
            % Convention (matches DriverModel): positive lateralError = left of
            % the reference line, so it is bounded by the left half-width;
            % negative by the right. Falls back to the symmetric scalar
            % half-width if a per-side value is missing, empty, or non-finite.
            if lateralError >= 0
                halfWidth = leftHalfWidth;
            else
                halfWidth = rightHalfWidth;
            end
            if isempty(halfWidth) || ~all(isfinite(halfWidth)) || any(halfWidth <= 0)
                halfWidth = fallback;
            end
            margin = halfWidth - abs(lateralError);
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

        function [bestDist2, bestIdx, bestT, bestPoint] = nearestSegmentProjection( ...
                x, y, trackData, searchStart, searchEnd, hasSegmentCache)
            idxRange = searchStart:searchEnd;
            p0 = trackData.points(idxRange, :);
            if hasSegmentCache
                v = trackData.segmentVectors(idxRange, :);
                invLen2 = trackData.segmentInvLen2(idxRange);
            else
                p1 = trackData.points(idxRange + 1, :);
                v = p1 - p0;
                len2 = sum(v.^2, 2);
                invLen2 = zeros(size(len2));
                validLen = len2 > eps;
                invLen2(validLen) = 1 ./ len2(validLen);
            end

            qx = x - p0(:, 1);
            qy = y - p0(:, 2);
            t = (qx .* v(:, 1) + qy .* v(:, 2)) .* invLen2(:);
            t(invLen2(:) <= 0) = 0;
            t = lts.util.saturate(t);

            projX = p0(:, 1) + t .* v(:, 1);
            projY = p0(:, 2) + t .* v(:, 2);
            dist2 = (x - projX).^2 + (y - projY).^2;

            [bestDist2, relIdx] = min(dist2);
            bestIdx = idxRange(relIdx);
            bestT = t(relIdx);
            bestPoint = [projX(relIdx), projY(relIdx)];
        end
    end
end
