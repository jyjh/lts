classdef TestTrack < components.Track
    % TESTTRACK FSAE-style test track for simulation validation
    % Generates a track with straights and constant-radius turns
    
    properties
        trackPoints   = []   % Nx2 [x, y] waypoints [m]
        trackCurvature = []  % Curvature at each point [1/m]
        trackHeading   = []  % Heading at each point [rad]
        trackMu        = []  % Surface friction at each point
        trackLength    = 0   % Total length [m]
    end
    
    methods
        function obj = TestTrack(trackType)
            % TESTTRACK Create a test track
            %   trackType: 'straight', 'oval', 'skidpad', 'autocross'
            %   Default is 'oval'
            
            if nargin < 1
                trackType = 'oval';
            end
            
            switch lower(trackType)
                case 'straight'
                    obj = buildStraight(obj);
                case 'oval'
                    obj = buildOval(obj);
                case 'skidpad'
                    obj = buildSkidpad(obj);
                case 'autocross'
                    obj = buildAutocross(obj);
                otherwise
                    error('Unknown track type: %s', trackType);
            end
        end
        
        function points = getTrackPoints(obj)
            points = obj.trackPoints;
        end
        
        function curvature = getCurvature(obj)
            curvature = obj.trackCurvature;
        end
        
        function mu = getSurfaceFriction(obj)
            mu = obj.trackMu;
        end
        
        function len = getTotalLength(obj)
            len = obj.trackLength;
        end
        
        function heading = getHeading(obj)
            heading = obj.trackHeading;
        end
    end
    
    methods (Access = private)
        function obj = buildStraight(obj)
            % 200m straight for top speed validation
            ds = 1;  % 1m spacing
            x = (0:ds:200)';
            y = zeros(size(x));
            
            obj.trackPoints = [x, y];
            obj = finalizeTrack(obj, 1.2);
        end
        
        function obj = buildOval(obj)
            % Oval track: two straights + two semicircles
            % Similar to a basic FSAE endurance loop
            straightLen = 60;  % [m]
            turnRadius  = 15;  % [m]
            ds = 1;            % spacing [m]
            
            % Build bottom straight (left to right)
            nStraight = round(straightLen / ds);
            x_straight = linspace(0, straightLen, nStraight)';
            y_straight = zeros(nStraight, 1);
            
            % Right semicircle (bottom to top)
            theta_right = linspace(-pi/2, pi/2, round(pi*turnRadius/ds))';
            x_right = straightLen + turnRadius * cos(theta_right);
            y_right = turnRadius + turnRadius * sin(theta_right);
            
            % Top straight (right to left)
            x_top = linspace(straightLen, 0, nStraight)';
            y_top = 2*turnRadius * ones(nStraight, 1);
            
            % Left semicircle (top to bottom)
            theta_left = linspace(pi/2, 3*pi/2, round(pi*turnRadius/ds))';
            x_left = turnRadius * cos(theta_left);
            y_left = turnRadius + turnRadius * sin(theta_left);
            
            % Combine (remove duplicate points at junctions)
            obj.trackPoints = [
                x_straight, y_straight;
                x_right(2:end), y_right(2:end);
                x_top(2:end), y_top(2:end);
                x_left(2:end), y_left(2:end)
            ];
            
            obj = finalizeTrack(obj, 1.2);
        end
        
        function obj = buildSkidpad(obj)
            % FSAE Skidpad: two pairs of concentric circles
            % We simulate one circle (8.125m radius per FSAE rules)
            radius = 8.125;  % [m] FSAE skidpad radius
            ds = 0.5;        % spacing [m]
            
            nPts = round(2 * pi * radius / ds);
            theta = linspace(0, 2*pi, nPts)';
            
            x = radius * cos(theta);
            y = radius * sin(theta);
            
            obj.trackPoints = [x, y];
            obj = finalizeTrack(obj, 1.2);
        end
        
        function obj = buildAutocross(obj)
            % Simple autocross-style track with varied corners
            % Mix of hairpins, chicanes, and straights
            ds = 1;
            
            % Define track as a series of [x, y] control points
            controlPts = [
                0,    0;     % Start/finish
                30,   0;     % End of first straight
                45,  10;     % Right turn entry
                50,  20;     % Right turn apex
                45,  30;     % Right turn exit
                30,  35;     % Short straight
                20,  45;     % Left turn (hairpin entry)
                10,  50;     % Hairpin apex
                0,   45;     % Hairpin exit
               -10,  35;     % Back straight
               -10,  20;     % Approach to final turn
                0,   10;     % Final turn
                0,    0;     % Back to start
            ];
            
            % Interpolate smoothly using spline
            nCtrl = size(controlPts, 1);
            t_ctrl = 0:nCtrl-1;
            t_fine = linspace(0, nCtrl-1, round(sum(sqrt(diff(controlPts(:,1)).^2 + diff(controlPts(:,2)).^2))/ds));
            
            x_fine = spline(t_ctrl, controlPts(:,1), t_fine);
            y_fine = spline(t_ctrl, controlPts(:,2), t_fine);
            
            obj.trackPoints = [x_fine(:), y_fine(:)];
            obj = finalizeTrack(obj, 1.2);
        end
        
        function obj = finalizeTrack(obj, mu)
            % FINALIZETRACK Compute derived track properties
            obj.trackCurvature = components.Track.computeCurvature(obj.trackPoints);
            obj.trackHeading = components.Track.computeHeading(obj.trackPoints);
            
            % Compute total length
            dx = diff(obj.trackPoints(:,1));
            dy = diff(obj.trackPoints(:,2));
            obj.trackLength = sum(sqrt(dx.^2 + dy.^2));
            
            % Uniform surface friction
            obj.trackMu = mu * ones(size(obj.trackPoints, 1), 1);
        end
    end
end