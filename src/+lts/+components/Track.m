classdef (Abstract) Track
    % TRACK Abstract interface for arbitrary waypoint-based tracks.
    %
    % Concrete implementations return a centerline as Nx2 [x, y] waypoints in
    % metres. Geometry helpers in this file are closed-loop aware and work for
    % image-imported, hand-traced, spline-generated, and legacy constant-radius
    % tracks.

    methods (Abstract)
        % Get track centerline as [x, y] waypoints [m]
        points = getTrackPoints(obj)

        % Get curvature at each waypoint [1/m]
        curvature = getCurvature(obj)

        % Compatibility channel fixed at one for every waypoint
        mu = getSurfaceFriction(obj)

        % Get total track length [m]
        length = getTotalLength(obj)

        % Get heading angle at each waypoint [rad]
        heading = getHeading(obj)

        % Get representative total track width [m] (scalar; one value per
        % waypoint for variable-width tracks is exposed via the optional
        % getTrackSideWidths method, which is not part of the abstract
        % contract but is implemented by WaypointTrack).
        width = getTrackWidth(obj)
    end

    methods (Static)
        % NOTE: the upstream Track.m also defines fromImage(), which delegates
        % to ImageTrackConverter.convert(). That image-conversion tool is not
        % part of this repo, so fromImage() is intentionally omitted here.

        function points = resampleTrack(points, ds, closed)
            % RESAMPLETRACK Resample track points to approximately uniform spacing.
            %   points = Track.resampleTrack(points, ds)
            %   points = Track.resampleTrack(points, ds, closed)
            %
            % Inputs:
            %   points  - Nx2 [x, y] waypoints
            %   ds      - desired spacing in the same units as points
            %   closed  - true for circuits; the final duplicate point is omitted
            if nargin < 3
                closed = false;
            end
            validateattributes(points, {'numeric'}, {'2d', 'ncols', 2, 'finite', 'real'});
            validateattributes(ds, {'numeric'}, {'scalar', 'positive', 'finite', 'real'});

            points = lts.components.Track.cleanPoints(points, closed);
            if size(points, 1) < 2
                return;
            end

            if closed
                p = [points; points(1,:)];
            else
                p = points;
            end

            s = lts.components.Track.cumulativeArcLength(p, false);
            totalLen = s(end);
            if totalLen <= eps
                points = p(1,:);
                return;
            end

            sNew = (0:ds:totalLen).';
            if closed && numel(sNew) > 1 && abs(sNew(end) - totalLen) < max(1e-9, 1e-9 * totalLen)
                sNew(end) = [];
            end
            if closed && isempty(sNew)
                sNew = 0;
            end

            xNew = interp1(s, p(:,1), sNew, 'linear');
            yNew = interp1(s, p(:,2), sNew, 'linear');
            points = [xNew, yNew];
        end

        function kappa = computeCurvature(points, closed)
            % COMPUTECURVATURE Compute signed curvature from discrete waypoints.
            %   Uses the circumcircle through neighboring waypoints, which is
            %   well behaved for non-constant-radius and nonuniformly-spaced data.
            %
            % Physics convention: curvature kappa = 1/R with sign from the
            % turn direction. The driver uses v_limit = sqrt(ay_limit/|kappa|);
            % the simulator stores curvature as reference telemetry only.
            if nargin < 2
                closed = false;
            end
            validateattributes(points, {'numeric'}, {'2d', 'ncols', 2, 'finite', 'real'});

            points = lts.components.Track.cleanPoints(points, closed);
            n = size(points, 1);
            kappa = zeros(n, 1);
            if n < 3
                return;
            end

            for i = 1:n
                if closed
                    iPrev = mod(i - 2, n) + 1;
                    iNext = mod(i, n) + 1;
                else
                    iPrev = max(i - 1, 1);
                    iNext = min(i + 1, n);
                    if iPrev == i || iNext == i
                        continue;
                    end
                end

                aVec = points(i,:)     - points(iPrev,:);
                bVec = points(iNext,:) - points(i,:);
                cVec = points(iNext,:) - points(iPrev,:);

                a = hypot(aVec(1), aVec(2));
                b = hypot(bVec(1), bVec(2));
                c = hypot(cVec(1), cVec(2));
                denom = a * b * c;
                if denom > eps
                    area2 = aVec(1) * bVec(2) - aVec(2) * bVec(1); % 2*signed area
                    kappa(i) = 2 * area2 / denom;
                end
            end

            if ~closed && n >= 3
                kappa(1) = kappa(2);
                kappa(end) = kappa(end-1);
            end
        end

        function theta = computeHeading(points, closed)
            % COMPUTEHEADING Compute unwrapped heading angle from waypoints.
            % Heading defines the local tangent used for projection telemetry
            % and driver path following. Vehicle yaw is still integrated from
            % tire yaw moment, so heading is not imposed on the car.
            if nargin < 2
                closed = false;
            end
            validateattributes(points, {'numeric'}, {'2d', 'ncols', 2, 'finite', 'real'});

            points = lts.components.Track.cleanPoints(points, closed);
            n = size(points, 1);
            theta = zeros(n, 1);
            if n < 2
                return;
            end

            dx = zeros(n, 1);
            dy = zeros(n, 1);
            if closed && n > 2
                for i = 1:n
                    iPrev = mod(i - 2, n) + 1;
                    iNext = mod(i, n) + 1;
                    dp = points(iNext,:) - points(iPrev,:);
                    dx(i) = dp(1);
                    dy(i) = dp(2);
                end
            else
                dx = gradient(points(:,1));
                dy = gradient(points(:,2));
            end
            theta = unwrap(atan2(dy, dx));
        end

        function s = cumulativeArcLength(points, closed)
            % CUMULATIVEARCLENGTH Return cumulative arc length at each waypoint.
            if nargin < 2
                closed = false;
            end
            validateattributes(points, {'numeric'}, {'2d', 'ncols', 2, 'finite', 'real'});
            points = lts.components.Track.cleanPoints(points, closed);
            if closed
                points = [points; points(1,:)];
            end
            if size(points, 1) < 2
                s = 0;
                return;
            end
            d = diff(points, 1, 1);
            seg = hypot(d(:,1), d(:,2));
            s = [0; cumsum(seg)];
        end

        function len = pathLength(points, closed)
            % PATHLENGTH Total length of an open path or closed circuit.
            if nargin < 2
                closed = false;
            end
            s = lts.components.Track.cumulativeArcLength(points, closed);
            len = s(end);
        end

        function points = cleanPoints(points, closed)
            % CLEANPOINTS Remove invalid rows and duplicate closure points.
            if nargin < 2
                closed = false;
            end
            if isempty(points)
                points = zeros(0, 2);
                return;
            end
            points = double(points);
            points = points(all(isfinite(points), 2), :);

            % Remove consecutive duplicate points.
            if size(points, 1) > 1
                keep = [true; hypot(diff(points(:,1)), diff(points(:,2))) > eps];
                points = points(keep, :);
            end

            % A closed track is represented without a duplicated final point.
            if closed && size(points, 1) > 2
                if hypot(points(end,1) - points(1,1), points(end,2) - points(1,2)) < 1e-9
                    points(end,:) = [];
                end
            end
        end
    end
end
