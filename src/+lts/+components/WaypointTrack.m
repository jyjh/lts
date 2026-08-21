classdef WaypointTrack < lts.components.Track
    % WAYPOINTTRACK Concrete Track backed by arbitrary [x, y] waypoints.
    %
    % Use this class for image-imported, CAD-imported, or hand-traced circuits.

    properties
        Points
        % Total track width [m]. Scalar for a constant-width track, or one
        % value per waypoint when the surface narrows/widens (cone-derived
        % corridors from the fsae track image tool). Per-side asymmetry is
        % carried by LeftWidth / RightWidth; Width is then the local total.
        Width = 3.0
        % Per-waypoint half-widths [m]. Empty [] means "symmetric: derive each
        % side from Width/2". When supplied, both must be present and have one
        % value per waypoint. Positive lateralError (car left of the
        % centerline) is bounded by LeftWidth, negative by RightWidth.
        LeftWidth = []
        RightWidth = []
        % Deprecated compatibility field. Surface friction variability is
        % intentionally unsupported; getSurfaceFriction always returns one.
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
            addParameter(ip, 'Width', 3.0, @(x) isnumeric(x) && all(x(:) > 0));
            addParameter(ip, 'LeftWidth', [], ...
                @(x) isempty(x) || (isnumeric(x) && all(x(:) > 0)));
            addParameter(ip, 'RightWidth', [], ...
                @(x) isempty(x) || (isnumeric(x) && all(x(:) > 0)));
            addParameter(ip, 'Mu', 1.0, @(x) isnumeric(x) && all(x(:) > 0));
            addParameter(ip, 'Closed', true, @(x) islogical(x) || isnumeric(x));
            addParameter(ip, 'Name', 'WaypointTrack', @(x) ischar(x) || isstring(x));
            addParameter(ip, 'SourceImage', '', @(x) ischar(x) || isstring(x));
            addParameter(ip, 'Metadata', struct(), @isstruct);
            parse(ip, points, varargin{:});

            obj.Points = lts.components.Track.cleanPoints(double(ip.Results.points), logical(ip.Results.Closed));
            obj.Width = double(ip.Results.Width);
            if ~isscalar(obj.Width)
                obj.Width = obj.Width(:);
            end
            obj.LeftWidth = double(ip.Results.LeftWidth(:));
            obj.RightWidth = double(ip.Results.RightWidth(:));
            % Accept legacy Mu input files/callers, but all surfaces use the
            % same unscaled tire model.
            obj.Mu = 1.0;
            obj.Closed = logical(ip.Results.Closed);
            obj.Name = char(ip.Results.Name);
            obj.SourceImage = char(ip.Results.SourceImage);
            obj.Metadata = ip.Results.Metadata;

            n = size(obj.Points, 1);
            if ~isscalar(obj.Width) && numel(obj.Width) ~= n
                error('WaypointTrack:InvalidWidth', ...
                    'Width must be scalar or have one value per waypoint.');
            end
            if xor(isempty(obj.LeftWidth), isempty(obj.RightWidth))
                error('WaypointTrack:IncompleteSideWidths', ...
                    'LeftWidth and RightWidth must either both be supplied or both be empty.');
            end
            if ~isempty(obj.LeftWidth) && ...
                    (numel(obj.LeftWidth) ~= n || numel(obj.RightWidth) ~= n)
                error('WaypointTrack:InvalidSideWidths', ...
                    'LeftWidth and RightWidth must have one value per waypoint.');
            end
        end

        function points = getTrackPoints(obj)
            points = obj.Points;
        end

        function curvature = getCurvature(obj)
            curvature = lts.components.Track.computeCurvature(obj.Points, obj.Closed);
        end

        function mu = getSurfaceFriction(obj)
            % Surface friction variability is intentionally unsupported; every
            % surface uses the same unscaled tire model. Returned as one value
            % per waypoint so width/mu consumers all see the same length.
            n = size(obj.Points, 1);
            mu = ones(n, 1);
        end

        function length = getTotalLength(obj)
            length = lts.components.Track.pathLength(obj.Points, obj.Closed);
        end

        function heading = getHeading(obj)
            heading = lts.components.Track.computeHeading(obj.Points, obj.Closed);
        end

        function width = getTrackWidth(obj)
            % GETTRACKWIDTH Representative total track width [m].
            % Returns the nominal Width when it is scalar. For a per-waypoint
            % corridor (vector Width, or LeftWidth/RightWidth set), returns the
            % mean total width so scalar-width callers (feasibility margins,
            % logging) get one sensible number. Per-waypoint / per-side callers
            % should use getTrackSideWidths instead.
            if ~isempty(obj.LeftWidth)
                width = mean(obj.LeftWidth + obj.RightWidth);
            elseif ~isscalar(obj.Width)
                width = mean(obj.Width);
            else
                width = obj.Width;
            end
        end

        function [leftWidth, rightWidth] = getTrackSideWidths(obj)
            % GETTRACKSIDEWIDTHS Per-waypoint left/right half-widths [m].
            % Returns Nx1 vectors. When LeftWidth/RightWidth are populated
            % (cone-derived corridor), they are returned as-is. Otherwise each
            % side is Width/2 -- scalar Width broadcast, or per-waypoint Width
            % halved elementwise.
            n = size(obj.Points, 1);
            if ~isempty(obj.LeftWidth)
                leftWidth = obj.LeftWidth;
                rightWidth = obj.RightWidth;
                return;
            end
            if isscalar(obj.Width)
                halfWidth = repmat(obj.Width / 2, n, 1);
            else
                halfWidth = obj.Width(:) / 2;
            end
            leftWidth = halfWidth;
            rightWidth = halfWidth;
        end

        function s = getStation(obj)
            s = lts.components.Track.cumulativeArcLength(obj.Points, obj.Closed);
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
            direction = lts.components.WaypointTrack.windingFromPoints(obj.Points);
        end

        function data = toStruct(obj)
            data = struct();
            data.name = obj.Name;
            data.points_m = obj.Points;
            data.width_m = obj.Width;
            if ~isempty(obj.LeftWidth)
                data.left_width_m = obj.LeftWidth;
                data.right_width_m = obj.RightWidth;
            end
            data.mu = 1.0;
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
            width = obj.getTrackWidth();
            if isscalar(width)
                T.width_m = repmat(width, height(T), 1);
            else
                T.width_m = width(:);
            end
            if ~isempty(obj.LeftWidth)
                T.left_width_m = obj.LeftWidth;
                T.right_width_m = obj.RightWidth;
            end
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

            requested = lts.components.WaypointTrack.normalizeDirection(ip.Results.Direction);

            % Validate the .mat before loading: load() reconstructs saved
            % objects by running their class constructor / loadobj, which can
            % execute arbitrary code. fileName is caller-supplied, so screen
            % it for non-data variables first (defense-in-depth).
            S = lts.util.loadMatSafe(fileName, 'WaypointTrack');
            if isfield(S, 'track')
                t = S.track;
            else
                error('WaypointTrack:InvalidMat', 'MAT file must contain a variable named track.');
            end

            % Recover the direction the file claims to be baked in.
            storedDirection = '';
            hasDirectionField = false;
            if isfield(t, 'direction') && ~isempty(t.direction)
                storedDirection = lts.components.WaypointTrack.normalizeDirection(t.direction);
                hasDirectionField = true;
            elseif isfield(t, 'metadata') && isfield(t.metadata, 'direction') ...
                    && ~isempty(t.metadata.direction)
                storedDirection = lts.components.WaypointTrack.normalizeDirection(t.metadata.direction);
                hasDirectionField = true;
            end

            points = double(t.points_m);
            closed = logical(t.closed);

            % Per-side half-widths are optional. When present they carry the
            % cone-derived asymmetric corridor and must track every reordering
            % of points_m (including the direction flip below).
            leftWidth = lts.components.WaypointTrack.optionalField(t, 'left_width_m');
            rightWidth = lts.components.WaypointTrack.optionalField(t, 'right_width_m');

            % Decide what direction we must end up in and whether to flip.
            % Priority: explicit Direction override > stored field >
            % (geometry-detected winding, used only for reporting).
            detected = lts.components.WaypointTrack.windingFromPoints(points);
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
                        points = lts.components.WaypointTrack.reversePreserveStart( ...
                            points, closed);
                        appliedFlip = true;
                        % Reversing travel direction swaps left and right: the
                        % half-width that bounded the left side now bounds the
                        % right side. Reverse the arrays to match the new point
                        % order, then swap sides.
                        if ~isempty(leftWidth)
                            revLeft = lts.components.WaypointTrack.reversePreserveStart( ...
                                leftWidth, closed);
                            revRight = lts.components.WaypointTrack.reversePreserveStart( ...
                                rightWidth, closed);
                            leftWidth = revRight;
                            rightWidth = revLeft;
                        end
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

            widthArgs = {'Width', t.width_m};
            if ~isempty(leftWidth)
                widthArgs = [widthArgs, ...
                    {'LeftWidth', leftWidth, 'RightWidth', rightWidth}]; %#ok<AGROW>
            end
            obj = lts.components.WaypointTrack(points, ...
                widthArgs{:}, ...
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
            % Guard against a malformed CSV (non-numeric columns, NaNs, wrong
            % shape) before winding/geometry consume it: a bad row would
            % otherwise surface deep in windingFromPoints as an opaque cast.
            if ~isnumeric(points) || size(points, 2) ~= 2 || ...
                    any(~isfinite(points), 'all') || size(points, 1) < 2
                error('WaypointTrack:InvalidPoints', ...
                    ['Track CSV "%s" did not yield >=2 finite numeric ' ...
                    '(x, y) rows.'], fileName);
            end
            widthArgs = {};
            if ismember('width_m', T.Properties.VariableNames)
                widthArgs = [widthArgs, {'Width', T.width_m}]; %#ok<AGROW>
            end
            if all(ismember({'left_width_m', 'right_width_m'}, ...
                    T.Properties.VariableNames))
                widthArgs = [widthArgs, {'LeftWidth', T.left_width_m, ...
                    'RightWidth', T.right_width_m}]; %#ok<AGROW>
            end
            obj = lts.components.WaypointTrack(points, widthArgs{:}, varargin{:});
        end

        function value = optionalField(s, name)
            % OPTIONFIELD Return s.(name) if present, else [].
            value = [];
            if isfield(s, name)
                value = s.(name);
            end
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
