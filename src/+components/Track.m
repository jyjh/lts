classdef (Abstract) Track
    % TRACK Abstract interface for track definitions
    % Provides track geometry and surface properties
    
    methods (Abstract)
        % Get track centerline as [x, y] waypoints [m]
        points = getTrackPoints(obj)
        
        % Get curvature at each waypoint [1/m]
        curvature = getCurvature(obj)
        
        % Get surface friction coefficient at each waypoint
        mu = getSurfaceFriction(obj)
        
        % Get total track length [m]
        length = getTotalLength(obj)
        
        % Get heading angle at each waypoint [rad]
        heading = getHeading(obj)
    end
    
    methods (Static)
        function points = resampleTrack(points, ds)
            % RESAMPLETRACK Resample track points to uniform spacing
            %   points  - Nx2 matrix of [x, y] waypoints
            %   ds      - desired spacing [m]

            points = components.Track.sanitizePointMatrix(points);
            if ~(isnumeric(ds) || islogical(ds)) || ~isreal(ds) ...
                    || ~isscalar(ds) || ~isfinite(ds) || ds <= 0
                error('Track:InvalidSpacing', ...
                    'Track resampling spacing must be a positive finite scalar.');
            end
            ds = double(ds);

            % Drop zero-length waypoint intervals before interpolation.
            % Duplicate arc-length samples make interp1 ambiguous, and a
            % repeated point has no physical effect on the centerline.
            [points, cumLen] = components.Track.prepareArcLengthSamples(points);
            if isempty(cumLen)
                return;
            end
            totalLen = cumLen(end);
            if size(points, 1) < 2 || totalLen <= eps
                return;
            end
            
            % Interpolate at uniform spacing and always preserve the endpoint.
            % The colon expression drops totalLen when it is not an exact
            % multiple of ds, which silently shortens open tracks.
            s_new = 0:ds:totalLen;
            if isempty(s_new) || abs(s_new(end) - totalLen) > eps(totalLen)
                s_new = [s_new, totalLen];
            end
            x_new = interp1(cumLen, points(:,1), s_new, 'linear');
            y_new = interp1(cumLen, points(:,2), s_new, 'linear');
            
            points = [x_new(:), y_new(:)];
        end
        
        function kappa = computeCurvature(points)
            % COMPUTECURVATURE Compute curvature from discrete waypoints
            %   Uses the signed circumcircle through each local point triplet.
            %   This keeps curvature geometric instead of dependent on waypoint
            %   index spacing, and treats explicitly closed tracks cyclically so
            %   the start/finish sample does not get a one-sided derivative dip.
            points = components.Track.sanitizePointMatrix(points);
            nPts = size(points, 1);
            if nPts < 3
                kappa = zeros(nPts, 1);
                return;
            end

            [uniquePoints, s, restoreIdx] = ...
                components.Track.prepareArcLengthSamples(points);
            if isempty(s) || s(end) <= eps
                kappa = zeros(nPts, 1);
                return;
            end
            nUnique = size(uniquePoints, 1);
            if nUnique < 3
                kappa = zeros(nPts, 1);
                return;
            end

            isClosed = components.Track.isSmoothClosedLoop(uniquePoints);
            if isClosed
                loopPoints = uniquePoints(1:end-1, :);
                nLoop = size(loopPoints, 1);
                loopKappa = zeros(nLoop, 1);
                for i = 1:nLoop
                    prevIdx = mod(i - 2, nLoop) + 1;
                    nextIdx = mod(i, nLoop) + 1;
                    loopKappa(i) = components.Track.signedTripletCurvature( ...
                        loopPoints(prevIdx, :), loopPoints(i, :), ...
                        loopPoints(nextIdx, :));
                end
                uniqueKappa = [loopKappa; loopKappa(1)];
            else
                uniqueKappa = zeros(nUnique, 1);
                for i = 2:nUnique-1
                    uniqueKappa(i) = components.Track.signedTripletCurvature( ...
                        uniquePoints(i-1, :), uniquePoints(i, :), ...
                        uniquePoints(i+1, :));
                end
                uniqueKappa(1) = uniqueKappa(2);
                uniqueKappa(end) = uniqueKappa(end-1);
            end
            uniqueKappa(~isfinite(uniqueKappa)) = 0;
            kappa = uniqueKappa(restoreIdx);
        end
        
        function theta = computeHeading(points)
            % COMPUTEHEADING Compute track tangent heading from waypoints.
            % Arc-length derivatives keep the tangent direction independent of
            % nonuniform waypoint spacing.
            points = components.Track.sanitizePointMatrix(points);
            nPts = size(points, 1);
            if nPts < 2
                theta = zeros(nPts, 1);
                return;
            end

            [uniquePoints, s, restoreIdx] = ...
                components.Track.prepareArcLengthSamples(points);
            if s(end) <= eps
                theta = zeros(nPts, 1);
                return;
            end
            if size(uniquePoints, 1) < 2
                theta = zeros(nPts, 1);
                return;
            end

            if components.Track.isSmoothClosedLoop(uniquePoints)
                loopPoints = uniquePoints(1:end-1, :);
                nLoop = size(loopPoints, 1);
                loopTheta = zeros(nLoop, 1);
                for i = 1:nLoop
                    prevIdx = mod(i - 2, nLoop) + 1;
                    nextIdx = mod(i, nLoop) + 1;
                    tangent = loopPoints(nextIdx, :) - loopPoints(prevIdx, :);
                    loopTheta(i) = atan2(tangent(2), tangent(1));
                end
                uniqueTheta = [loopTheta; loopTheta(1)];
            else
                dxds = gradient(uniquePoints(:,1), s);
                dyds = gradient(uniquePoints(:,2), s);
                uniqueTheta = atan2(dyds, dxds);
            end
            theta = uniqueTheta(restoreIdx);
        end

        function s = computeArcLength(points)
            % COMPUTEARCLENGTH Cumulative centerline distance at each waypoint.
            points = components.Track.sanitizePointMatrix(points);
            if isempty(points)
                s = zeros(0, 1);
                return;
            end
            dx = diff(points(:,1));
            dy = diff(points(:,2));
            segLen = sqrt(dx.^2 + dy.^2);
            s = [0; cumsum(segLen)];
        end
    end

    methods (Static, Access = private)
        function [points, s, restoreIdx] = prepareArcLengthSamples(points)
            % PREPAREARCLENGTHSAMPLES Remove zero-length waypoint intervals.
            %
            % Consecutive duplicate points are common in hand-edited or
            % imported tracks. They have no geometric length, so derivatives
            % with respect to arc length are undefined at those samples. Keep
            % one copy for the geometry calculation and map duplicate outputs
            % back to the original waypoint count.
            points = components.Track.sanitizePointMatrix(points);
            if isempty(points)
                s = zeros(0, 1);
                restoreIdx = zeros(0, 1);
                return;
            end

            sAll = components.Track.computeArcLength(points);
            keep = [true; diff(sAll) > eps];
            points = points(keep, :);
            s = sAll(keep);
            restoreIdx = cumsum(keep);
        end

        function points = sanitizePointMatrix(points)
            if ~(isnumeric(points) || islogical(points)) || ~isreal(points) ...
                    || ~ismatrix(points) || size(points, 2) < 2
                points = zeros(0, 2);
                return;
            end

            points = double(points(:, 1:2));
            nPts = size(points, 1);
            if nPts == 0
                return;
            end

            finiteRows = all(isfinite(points), 2);
            if all(finiteRows)
                return;
            end
            if ~any(finiteRows)
                points(:) = 0;
                return;
            end

            sampleIdx = (1:nPts)';
            validIdx = sampleIdx(finiteRows);
            for colIdx = 1:2
                column = points(:, colIdx);
                column(~finiteRows) = interp1(validIdx, column(finiteRows), ...
                    sampleIdx(~finiteRows), 'linear', 'extrap');
                column(~isfinite(column)) = 0;
                points(:, colIdx) = column;
            end
        end

        function kappa = signedTripletCurvature(p0, p1, p2)
            v01 = p1 - p0;
            v12 = p2 - p1;
            v02 = p2 - p0;
            side01 = hypot(v01(1), v01(2));
            side12 = hypot(v12(1), v12(2));
            side02 = hypot(v02(1), v02(2));
            denom = side01 * side12 * side02;
            if denom <= eps
                kappa = 0;
                return;
            end

            crossZ = v01(1) * v12(2) - v01(2) * v12(1);
            kappa = 2 * crossZ / denom;
        end

        function isClosed = isSmoothClosedLoop(points)
            nPts = size(points, 1);
            isClosed = false;
            if nPts <= 3
                return;
            end

            pointSpan = max(points, [], 1) - min(points, [], 1);
            closureTol = max(1e-9, 1e-9 * max(pointSpan));
            if norm(points(end, :) - points(1, :)) > closureTol
                return;
            end

            incomingTangent = points(end, :) - points(end-1, :);
            outgoingTangent = points(2, :) - points(1, :);
            tangentNorms = hypot(incomingTangent(1), incomingTangent(2)) ...
                * hypot(outgoingTangent(1), outgoingTangent(2));
            if tangentNorms <= eps
                return;
            end

            tangentAlignment = dot(incomingTangent, outgoingTangent) ...
                / tangentNorms;
            isClosed = tangentAlignment > cos(pi / 4);
        end
    end
end
