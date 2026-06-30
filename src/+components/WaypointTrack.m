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
        function obj = loadMat(fileName)
            S = load(fileName);
            if isfield(S, 'track')
                t = S.track;
            else
                error('WaypointTrack:InvalidMat', 'MAT file must contain a variable named track.');
            end

            obj = components.WaypointTrack(t.points_m, ...
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
    end
end
