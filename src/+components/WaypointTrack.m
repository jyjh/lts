classdef WaypointTrack < components.Track
    % WAYPOINTTRACK Concrete Track backed by arbitrary [x, y] waypoints.
    %
    % Use this class for image-imported, CAD-imported, or hand-traced circuits.

    properties
        Points
        Width = 3.0
        Mu = 1.0
        Closed = true
        Name = 'WaypointTrack'
        SourceImage = ''
        Metadata = struct()
    end

    methods
        function obj = WaypointTrack(points, varargin)
            if nargin == 0
                points = zeros(0, 2);
            end

            ip = inputParser;
            ip.FunctionName = 'WaypointTrack';
            addRequired(ip, 'points', @(x) isnumeric(x) && size(x,2) == 2);
            addParameter(ip, 'Width', 3.0, @(x) isnumeric(x) && isscalar(x) && x > 0);
            addParameter(ip, 'Mu', 1.0, @(x) isnumeric(x) && all(x(:) > 0));
            addParameter(ip, 'Closed', true, @(x) islogical(x) || isnumeric(x));
            addParameter(ip, 'Name', 'WaypointTrack', @(x) ischar(x) || isstring(x));
            addParameter(ip, 'SourceImage', '', @(x) ischar(x) || isstring(x));
            addParameter(ip, 'Metadata', struct(), @isstruct);
            parse(ip, points, varargin{:});

            obj.Points = components.Track.cleanPoints(double(ip.Results.points), logical(ip.Results.Closed));
            obj.Width = double(ip.Results.Width);
            obj.Mu = double(ip.Results.Mu);
            obj.Closed = logical(ip.Results.Closed);
            obj.Name = char(ip.Results.Name);
            obj.SourceImage = char(ip.Results.SourceImage);
            obj.Metadata = ip.Results.Metadata;
        end

        function points = getTrackPoints(obj)
            points = obj.Points;
        end

        function curvature = getCurvature(obj)
            curvature = components.Track.computeCurvature(obj.Points, obj.Closed);
        end

        function mu = getSurfaceFriction(obj)
            n = size(obj.Points, 1);
            if isscalar(obj.Mu)
                mu = repmat(obj.Mu, n, 1);
            else
                mu = obj.Mu(:);
                if numel(mu) ~= n
                    error('WaypointTrack:InvalidMu', ...
                        'Mu must be scalar or have one value per waypoint.');
                end
            end
        end

        function length = getTotalLength(obj)
            length = components.Track.pathLength(obj.Points, obj.Closed);
        end

        function heading = getHeading(obj)
            heading = components.Track.computeHeading(obj.Points, obj.Closed);
        end

        function width = getTrackWidth(obj)
            width = obj.Width;
        end

        function s = getStation(obj)
            s = components.Track.cumulativeArcLength(obj.Points, obj.Closed);
            s = s(1:size(obj.Points,1));
        end

        function direction = getDirection(obj)
            % GETDIRECTION Effective travel direction of the stored waypoints.
            %
            % Returns 'clockwise', 'anticlockwise', or 'unknown' from the
            % signed area (shoelace) of obj.Points. This is the winding the
            % simulator actually drives, derived from geometry rather than a
            % metadata flag, so it reflects any direction override applied at
            % load time. Convention (y up): negative area = clockwise,
            % positive = anticlockwise, matching the exporter.
            direction = components.WaypointTrack.windingFromPoints(obj.Points);
        end

        function data = toStruct(obj)
            data = struct();
            data.name = obj.Name;
            data.points_m = obj.Points;
            data.width_m = obj.Width;
            data.mu = obj.Mu;
            data.closed = obj.Closed;
            data.source_image = obj.SourceImage;
            data.length_m = obj.getTotalLength();
            data.heading_rad = obj.getHeading();
            data.curvature_1pm = obj.getCurvature();
            data.station_m = obj.getStation();
            if isfield(obj.Metadata, 'direction')
                data.direction = obj.Metadata.direction;
            end
            if isfield(obj.Metadata, 'requested_direction')
                data.requested_direction = obj.Metadata.requested_direction;
            end
            data.metadata = obj.Metadata;
        end

        function exportCsv(obj, fileName)
            points = obj.getTrackPoints();
            T = table();
            T.s_m = obj.getStation();
            T.x_m = points(:,1);
            T.y_m = points(:,2);
            T.heading_rad = obj.getHeading();
            T.curvature_1pm = obj.getCurvature();
            T.mu = obj.getSurfaceFriction();
            writetable(T, fileName);
        end

        function saveMat(obj, fileName)
            track = obj.toStruct(); %#ok<NASGU>
            save(fileName, 'track');
        end

        function h = plot(obj, varargin)
            points = obj.getTrackPoints();
            h = plot(points(:,1), points(:,2), varargin{:});
            axis equal;
            grid on;
            xlabel('x [m]');
            ylabel('y [m]');
            title(obj.Name, 'Interpreter', 'none');
        end
    end

    methods (Static)
        function obj = loadMat(fileName, varargin)
            % LOADMAT Load a WaypointTrack from a .mat file produced by the
            % fsae track image tool.
            %
            %   track = WaypointTrack.loadMat(file)
            %   track = WaypointTrack.loadMat(file, 'Direction', 'anticlockwise')
            %
            % The exporter bakes the requested travel direction into points_m
            % ordering and also records it in a 'direction' field. By default
            % the stored order is taken as-is. Supply Direction to force the
            % car to run a particular way: 'clockwise'/'cw' or
            % 'anticlockwise'/'counterclockwise'/'ccw'/'acw'. If that conflicts
            % with the stored direction (or the file has no direction field),
            % the waypoints are reversed -- keeping the start/finish point
            % fixed for closed loops -- and a warning is emitted.

            ip = inputParser;
            ip.FunctionName = 'WaypointTrack.loadMat';
            addParameter(ip, 'Direction', '', @(x) ischar(x) || isstring(x));
            parse(ip, varargin{:});

            requested = components.WaypointTrack.normalizeDirection(ip.Results.Direction);

            S = load(fileName);
            if isfield(S, 'track')
                t = S.track;
            else
                error('WaypointTrack:InvalidMat', 'MAT file must contain a variable named track.');
            end

            % Recover the direction the file claims to be baked in.
            storedDirection = '';
            hasDirectionField = false;
            if isfield(t, 'direction') && ~isempty(t.direction)
                storedDirection = components.WaypointTrack.normalizeDirection(t.direction);
                hasDirectionField = true;
            elseif isfield(t, 'metadata') && isfield(t.metadata, 'direction') ...
                    && ~isempty(t.metadata.direction)
                storedDirection = components.WaypointTrack.normalizeDirection(t.metadata.direction);
                hasDirectionField = true;
            end

            points = double(t.points_m);
            closed = logical(t.closed);

            % Decide what direction we must end up in and whether to flip.
            % Priority: explicit Direction override > stored field >
            % (geometry-detected winding, used only for reporting).
            detected = components.WaypointTrack.windingFromPoints(points);
            if ~isempty(requested)
                target = requested;
            elseif ~isempty(storedDirection)
                target = storedDirection;
            else
                target = detected;
            end

            appliedFlip = false;
            if ~isempty(target) && ~strcmp(target, 'unknown')
                % Need to flip when the geometry currently disagrees with the
                % target. detected reflects the actual winding of `points`.
                if ~strcmp(detected, target)
                    if strcmp(detected, 'unknown')
                        % Degenerate (collinear / <3 pts): trust the requested
                        % label but warn, since geometry can't confirm it.
                        if ~isempty(requested)
                            warning('WaypointTrack:DirectionUndetectable', ...
                                ['Track geometry has no detectable winding; ' ...
                                 'cannot enforce Direction="%s". Leaving point ' ...
                                 'order unchanged.'], target);
                        end
                    else
                        points = components.WaypointTrack.reversePreserveStart( ...
                            points, closed);
                        appliedFlip = true;
                        if ~isempty(requested)
                            source = 'the requested Direction';
                        else
                            source = 'the stored direction field';
                        end
                        warning('WaypointTrack:DirectionReversed', ...
                            ['Reversed waypoint order to honor %s ("%s"). ' ...
                             'The points were previously oriented "%s".'], ...
                            source, target, detected);
                    end
                end
            end

            if ~hasDirectionField
                warning('WaypointTrack:NoDirectionField', ...
                    ['"%s" has no direction field; assuming "%s" from the ' ...
                     'point ordering. Re-export with a Direction option or ' ...
                     'pass ''Direction'' to loadMat to override.'], ...
                    fileName, target);
            end

            obj = components.WaypointTrack(points, ...
                'Width', t.width_m, ...
                'Mu', t.mu, ...
                'Closed', t.closed, ...
                'Name', t.name);
            if isfield(t, 'source_image')
                obj.SourceImage = t.source_image;
            end
            if isfield(t, 'metadata')
                obj.Metadata = t.metadata;
            end

            % Stamp the resolved direction so toStruct / reporting can see it.
            obj.Metadata.resolved_direction = target;
            obj.Metadata.resolved_direction_flipped = appliedFlip;
        end

        function obj = fromCsv(fileName, varargin)
            T = readtable(fileName);
            if all(ismember({'x_m', 'y_m'}, T.Properties.VariableNames))
                points = [T.x_m, T.y_m];
            elseif all(ismember({'x', 'y'}, T.Properties.VariableNames))
                points = [T.x, T.y];
            else
                error('WaypointTrack:InvalidCsv', ...
                    'CSV must contain x_m/y_m or x/y columns.');
            end
            obj = components.WaypointTrack(points, varargin{:});
        end

        function direction = windingFromPoints(points)
            % WINDINGFROMPOINTS Signed-area winding of an [x,y] point list.
            % Returns 'clockwise', 'anticlockwise', or 'unknown'. Convention
            % (y up): negative signed area = clockwise, positive =
            % anticlockwise. Mirrors ImageTrackConverter.inferDirection so the
            % consumer and exporter agree on what 'clockwise' means.
            direction = 'unknown';
            if size(points, 1) < 3
                return;
            end
            p = double(points(:,1:2));
            j = [2:size(p,1), 1];
            signedArea = 0.5 * sum(p(:,1) .* p(j,2) - p(j,1) .* p(:,2));
            scale = max(1, max(abs(p(:))));
            if abs(signedArea) <= 1e-12 * scale * scale
                return;
            end
            if signedArea < 0
                direction = 'clockwise';
            else
                direction = 'anticlockwise';
            end
        end

        function direction = normalizeDirection(direction)
            % NORMALIZEDIRECTION Map direction aliases to canonical strings.
            % Accepts clockwise/cw, anticlockwise/counterclockwise/acw/ccw,
            % and '' (empty = no override). Throws on anything else.
            if isempty(direction)
                direction = '';
                return;
            end
            d = lower(strtrim(char(direction)));
            d = strrep(d, '-', '');
            d = strrep(d, '_', '');
            d = strrep(d, ' ', '');
            switch d
                case {'clockwise', 'cw'}
                    direction = 'clockwise';
                case {'anticlockwise', 'counterclockwise', 'acw', 'ccw'}
                    direction = 'anticlockwise';
                case ''
                    direction = '';
                otherwise
                    error('WaypointTrack:InvalidDirection', ...
                        ['Direction must be ''clockwise'', ''anticlockwise'', ' ...
                         '''counterclockwise'', or empty.']);
            end
        end

        function points = reversePreserveStart(points, closed)
            % REVERSEPRESERVESTART Reverse point order, keeping the
            % start/finish point (row 1) fixed for closed loops. Mirrors
            % ImageTrackConverter.reversePreserveStart so a reversed track
            % starts at the same physical point as the original.
            if size(points, 1) < 2
                return;
            end
            if closed
                points = [points(1,:); flipud(points(2:end,:))];
            else
                points = flipud(points);
            end
        end
    end
end
