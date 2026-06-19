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

        % Get total fixed track width [m]
        width = getTrackWidth(obj)
    end
    
    methods (Static)
        function points = resampleTrack(points, ds)
            % RESAMPLETRACK Resample track points to uniform spacing
            %   points  - Nx2 matrix of [x, y] waypoints
            %   ds      - desired spacing [m]
            
            % Compute cumulative arc length
            dx = diff(points(:,1));
            dy = diff(points(:,2));
            segLen = sqrt(dx.^2 + dy.^2);
            cumLen = [0; cumsum(segLen)];
            totalLen = cumLen(end);
            
            % Interpolate at uniform spacing
            s_new = 0:ds:totalLen;
            x_new = interp1(cumLen, points(:,1), s_new, 'linear');
            y_new = interp1(cumLen, points(:,2), s_new, 'linear');
            
            points = [x_new(:), y_new(:)];
        end
        
        function kappa = computeCurvature(points)
            % COMPUTECURVATURE Compute curvature from discrete waypoints
            %   Uses finite differences on heading angle
            
            dx = gradient(points(:,1));
            dy = gradient(points(:,2));
            
            ddx = gradient(dx);
            ddy = gradient(dy);
            
            % Curvature = (x'*y'' - y'*x'') / (x'^2 + y'^2)^(3/2)
            kappa = (dx.*ddy - dy.*ddx) ./ (dx.^2 + dy.^2).^1.5;
            
            % Fix NaN at endpoints
            kappa(1) = kappa(2);
            kappa(end) = kappa(end-1);
        end
        
        function theta = computeHeading(points)
            % COMPUTEHEADING Compute heading angle from waypoints
            dx = gradient(points(:,1));
            dy = gradient(points(:,2));
            theta = atan2(dy, dx);
        end
    end
end
