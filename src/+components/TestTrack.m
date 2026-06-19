classdef TestTrack < components.Track
    % TESTTRACK FSAE-style test track for simulation validation
    % Generates a track with straights and constant-radius turns
    
    properties
        trackPoints   = []   % Nx2 [x, y] waypoints [m]
        trackCurvature = []  % Curvature at each point [1/m]
        trackHeading   = []  % Heading at each point [rad]
        trackMu        = []  % Surface friction at each point
        trackLength    = 0   % Total length [m]
        trackWidth     = 3   % Fixed total track width [m]
    end
    
    methods
        function obj = TestTrack(trackType)
            % TESTTRACK Create a test track
            %   trackType: 'straight10', 'straight', 'oval', 'skidpad', 'autocross', 'busstop'
            %   Default is 'oval'
            
            if nargin < 1
                trackType = 'oval';
            end
            
            switch lower(trackType)
                case 'straight10'
                    obj = buildStraight10(obj);
                case 'straight'
                    obj = buildStraight(obj);
                case 'oval'
                    obj = buildOval(obj);
                case 'skidpad'
                    obj = buildSkidpad(obj);
                case 'autocross'
                    obj = buildAutocross(obj);
                case 'busstop'
                    obj = buildBusstop(obj);
                case '90turn'
                    obj = buildNinetyTurn(obj);
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

        function width = getTrackWidth(obj)
            width = obj.trackWidth;
        end
    end
    
    methods (Access = private)
        function obj = buildStraight10(obj)
            % 10m straight for fast export/debug validation
            ds = 0.5;  % 0.5m spacing
            x = (0:ds:10)';
            y = zeros(size(x));

            obj.trackPoints = [x, y];
            obj = finalizeTrack(obj, 1.2);
        end

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
        
        function obj = buildBusstop(obj)
            % BUSSTOP chicane
            % straight before and after.
            %
            % Layout (open track, not a closed loop):
            %   1. Entry straight  — 80 m, heading East
            %   2. Left turn       — 90° CCW arc, R = 20 m
            %   3. Short straight  — 15 m, heading North
            %   4. Right turn      — 90° CW arc, R = 20 m
            %   5. Exit straight   — 80 m, heading East
            
            ds = 0.5;            % point spacing [m]
            turnRadius = 20;     % chicane turn radius [m]
            
            % ---- Segment 1: Entry straight (East along y=0) ----
            entryLen = 150;
            nEntry = round(entryLen / ds) + 1;
            x_entry = linspace(0, entryLen, nEntry)';
            y_entry = zeros(nEntry, 1);
            
            % ---- Segment 2: Left turn (90° CCW arc, R=20) ----
            % Center at (entryLen, turnRadius) = (80, 20)
            % Arc from θ = -π/2 (bottom, at (80,0)) to θ = 0 (right, at (100,20))
            arcLen = pi/2 * turnRadius;
            nArc = max(round(arcLen / ds) + 1, 10);
            theta_left = linspace(-pi/2, 0, nArc)';
            cx_left = entryLen;
            cy_left = turnRadius;
            x_leftArc = cx_left + turnRadius * cos(theta_left);
            y_leftArc = cy_left + turnRadius * sin(theta_left);
            
            % ---- Segment 3: Short straight (North) ----
            shortLen = 6;
            nShort = round(shortLen / ds) + 1;
            % Starts where left arc ends: (entryLen + turnRadius, turnRadius) = (100, 20)
            x_short = (entryLen + turnRadius) * ones(nShort, 1);
            y_short = linspace(turnRadius, turnRadius + shortLen, nShort)';
            
            % ---- Segment 4: Right turn (90° CW arc, R=20) ----
            % Center at (entryLen + 2*turnRadius, turnRadius + shortLen) = (120, 35)
            % Arc from θ = π (left, at (100,35)) to θ = π/2 (top, at (120,55))
            cx_right = entryLen + 2 * turnRadius;
            cy_right = turnRadius + shortLen;
            theta_right = linspace(pi, pi/2, nArc)';
            x_rightArc = cx_right + turnRadius * cos(theta_right);
            y_rightArc = cy_right + turnRadius * sin(theta_right);
            
            % ---- Segment 5: Exit straight (East) ----
            exitLen = 80;
            nExit = round(exitLen / ds) + 1;
            % Starts where right arc ends: (cx_right, cy_right + turnRadius) = (120, 55)
            x_exit = linspace(cx_right, cx_right + exitLen, nExit)';
            y_exit = (cy_right + turnRadius) * ones(nExit, 1);
            
            % ---- Combine segments (remove duplicate junction points) ----
            obj.trackPoints = [
                x_entry,    y_entry;
                x_leftArc(2:end),  y_leftArc(2:end);
                x_short(2:end),    y_short(2:end);
                x_rightArc(2:end), y_rightArc(2:end);
                x_exit(2:end),     y_exit(2:end)
            ];
            
            obj = finalizeTrack(obj, 1.2);
        end

        function obj = buildNinetyTurn(obj)
            % Ninety degree turn, straight before and after.
            %
            % Layout (open track, not a closed loop):
            %   1. Entry straight  — 80 m, heading East
            %   2. Left turn       — 90° CCW arc, R = 20 m
            %   5. Exit straight   — 80 m, heading East
            
            ds = 0.5;            % point spacing [m]
            turnRadius = 20;     % chicane turn radius [m]
            
            % ---- Segment 1: Entry straight (East along y=0) ----
            entryLen = 150;
            nEntry = round(entryLen / ds) + 1;
            x_entry = linspace(0, entryLen, nEntry)';
            y_entry = zeros(nEntry, 1);
            
            % ---- Segment 2: Left turn (90° CCW arc, R=20) ----
            % Center at (entryLen, turnRadius) = (80, 20)
            % Arc from θ = -π/2 (bottom, at (80,0)) to θ = 0 (right, at (100,20))
            arcLen = pi/2 * turnRadius;
            nArc = max(round(arcLen / ds) + 1, 10);
            theta_left = linspace(-pi/2, 0, nArc)';
            cx_left = entryLen;
            cy_left = turnRadius;
            x_leftArc = cx_left + turnRadius * cos(theta_left);
            y_leftArc = cy_left + turnRadius * sin(theta_left);
            
            % ---- Segment 53: Exit straight (East) ----
            exitLen = 80;
            nExit = round(exitLen / ds) + 1;
            % Starts where right arc ends: (cx_right, cy_right + turnRadius) = (120, 55)
            x_exit = (entryLen + turnRadius) * ones(nExit, 1);
            y_exit = linspace(turnRadius, turnRadius + exitLen, nExit)';
            
            % ---- Combine segments (remove duplicate junction points) ----
            obj.trackPoints = [
                x_entry,    y_entry;
                x_leftArc(2:end),  y_leftArc(2:end);
                x_exit(2:end),     y_exit(2:end)
            ];
            
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
