classdef TestTrack < lts.components.WaypointTrack
    % Procedural tracks for simulation and validation.

    properties
        warmupLaps   = 0   % Complete laps simulated before telemetry starts
        recordedLaps = 1   % Complete laps retained in returned telemetry
    end

    methods
        function obj = TestTrack(trackType)
            if nargin < 1
                trackType = 'oval';
            end

            switch lower(trackType)
                case 'straight10'
                    obj = buildStraight(obj, 10, 0.5, 'straight10');
                case 'straight'
                    obj = buildStraight(obj, 200, 1, 'straight');
                case 'straight75'
                    obj = buildStraight(obj, 75, 1, 'straight75');
                case 'oval'
                    obj = buildOval(obj);
                case 'skidpad'
                    obj = buildSkidpad(obj);
                case 'autocross'
                    obj = buildAutocross(obj);
                case 'busstop'
                    obj = buildBusstop(obj);
                case 'slalom'
                    obj = buildSlalom(obj);
                case '90turn'
                    obj = buildNinetyTurn(obj);
                otherwise
                    error('Unknown track type: %s', trackType);
            end
        end

        function closed = isClosedLoop(obj)
            closed = obj.Closed;
        end

        function laps = getWarmupLaps(obj)
            laps = obj.warmupLaps;
        end

        function laps = getRecordedLaps(obj)
            laps = obj.recordedLaps;
        end
    end

    methods (Access = private)
        function obj = buildStraight(obj, trackLength, spacing, name)
            x = (0:spacing:trackLength)';
            obj = setPoints(obj, [x, zeros(size(x))], false, name);
        end

        function obj = buildOval(obj)
            % Two straights joined by semicircles.
            straightLen = 60;  % [m]
            turnRadius  = 15;  % [m]
            ds = 1;            % spacing [m]

            nStraight = round(straightLen / ds);
            x_straight = linspace(0, straightLen, nStraight)';
            y_straight = zeros(nStraight, 1);

            theta_right = linspace(-pi/2, pi/2, round(pi*turnRadius/ds))';
            x_right = straightLen + turnRadius * cos(theta_right);
            y_right = turnRadius + turnRadius * sin(theta_right);

            x_top = linspace(straightLen, 0, nStraight)';
            y_top = 2*turnRadius * ones(nStraight, 1);

            theta_left = linspace(pi/2, 3*pi/2, round(pi*turnRadius/ds))';
            x_left = turnRadius * cos(theta_left);
            y_left = turnRadius + turnRadius * sin(theta_left);

            points = [
                x_straight, y_straight;
                x_right(2:end), y_right(2:end);
                x_top(2:end), y_top(2:end);
                x_left(2:end), y_left(2:end)
            ];
            obj = setPoints(obj, points, true, 'oval');
        end

        function obj = buildSkidpad(obj)
            % One FSAE skidpad circle.
            radius = 9.125;  % [m] FSAE skidpad radius
            ds = 0.5;        % spacing [m]

            nPts = round(2 * pi * radius / ds);
            theta = linspace(0, 2*pi, nPts)';

            x = radius * cos(theta);
            y = radius * sin(theta);

            obj = setPoints(obj, [x, y], true, 'skidpad');
            obj.warmupLaps = 1;
            obj.recordedLaps = 1;
        end

        function obj = buildAutocross(obj)
            ds = 1;

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

            nCtrl = size(controlPts, 1);
            t_ctrl = 0:nCtrl-1;
            t_fine = linspace(0, nCtrl-1, round(sum(sqrt(diff(controlPts(:,1)).^2 + diff(controlPts(:,2)).^2))/ds));

            x_fine = spline(t_ctrl, controlPts(:,1), t_fine);
            y_fine = spline(t_ctrl, controlPts(:,2), t_fine);

            obj = setPoints(obj, [x_fine(:), y_fine(:)], true, 'autocross');
        end

        function obj = buildBusstop(obj)
            % Open bus-stop chicane.

            ds = 0.5;            % point spacing [m]
            turnRadius = 20;     % chicane turn radius [m]

            entryLen = 150;
            nEntry = round(entryLen / ds) + 1;
            x_entry = linspace(0, entryLen, nEntry)';
            y_entry = zeros(nEntry, 1);

            arcLen = pi/2 * turnRadius;
            nArc = max(round(arcLen / ds) + 1, 10);
            theta_left = linspace(-pi/2, 0, nArc)';
            cx_left = entryLen;
            cy_left = turnRadius;
            x_leftArc = cx_left + turnRadius * cos(theta_left);
            y_leftArc = cy_left + turnRadius * sin(theta_left);

            shortLen = 6;
            nShort = round(shortLen / ds) + 1;
            x_short = (entryLen + turnRadius) * ones(nShort, 1);
            y_short = linspace(turnRadius, turnRadius + shortLen, nShort)';

            cx_right = entryLen + 2 * turnRadius;
            cy_right = turnRadius + shortLen;
            theta_right = linspace(pi, pi/2, nArc)';
            x_rightArc = cx_right + turnRadius * cos(theta_right);
            y_rightArc = cy_right + turnRadius * sin(theta_right);

            exitLen = 80;
            nExit = round(exitLen / ds) + 1;
            x_exit = linspace(cx_right, cx_right + exitLen, nExit)';
            y_exit = (cy_right + turnRadius) * ones(nExit, 1);

            points = [
                x_entry,    y_entry;
                x_leftArc(2:end),  y_leftArc(2:end);
                x_short(2:end),    y_short(2:end);
                x_rightArc(2:end), y_rightArc(2:end);
                x_exit(2:end),     y_exit(2:end)
            ];

            obj = setPoints(obj, points, false, 'busstop');
        end

        function obj = buildSlalom(obj)
            % Open slalom with entry and exit straights.

            ds = 0.5;             % point spacing [m]
            entryLen = 35;        % short straight to build speed [m]
            exitLen = 15;         % settle after final weave [m]
            coneSpacing = 15;     % distance between alternating offsets [m]
            numOffsets = 7;       % number of left/right slalom offsets
            lateralOffset = 2.5;  % peak centerline offset [m]
            rampLen = coneSpacing;

            slalomLen = numOffsets * coneSpacing;
            period = 2 * coneSpacing;

            nEntry = round(entryLen / ds) + 1;
            x_entry = linspace(0, entryLen, nEntry)';
            y_entry = zeros(nEntry, 1);

            % Ramp the sine envelope to preserve heading continuity.
            u = (0:ds:slalomLen)';
            rampIn = lts.util.saturate(u / rampLen);
            rampOut = lts.util.saturate((slalomLen - u) / rampLen);
            smoothIn = rampIn.^2 .* (3 - 2 * rampIn);
            smoothOut = rampOut.^2 .* (3 - 2 * rampOut);
            envelope = smoothIn .* smoothOut;

            x_slalom = entryLen + u;
            y_slalom = lateralOffset * envelope .* sin(2 * pi * u / period);

            nExit = round(exitLen / ds) + 1;
            x_exit = linspace(entryLen + slalomLen, ...
                entryLen + slalomLen + exitLen, nExit)';
            y_exit = zeros(nExit, 1);

            points = [
                x_entry, y_entry;
                x_slalom(2:end), y_slalom(2:end);
                x_exit(2:end), y_exit(2:end)
            ];

            obj = setPoints(obj, points, false, 'slalom');
        end

        function obj = buildNinetyTurn(obj)
            % Open 90-degree turn with entry and exit straights.

            ds = 0.5;            % point spacing [m]
            turnRadius = 20;     % chicane turn radius [m]

            entryLen = 150;
            nEntry = round(entryLen / ds) + 1;
            x_entry = linspace(0, entryLen, nEntry)';
            y_entry = zeros(nEntry, 1);

            arcLen = pi/2 * turnRadius;
            nArc = max(round(arcLen / ds) + 1, 10);
            theta_left = linspace(-pi/2, 0, nArc)';
            cx_left = entryLen;
            cy_left = turnRadius;
            x_leftArc = cx_left + turnRadius * cos(theta_left);
            y_leftArc = cy_left + turnRadius * sin(theta_left);

            exitLen = 80;
            nExit = round(exitLen / ds) + 1;
            x_exit = (entryLen + turnRadius) * ones(nExit, 1);
            y_exit = linspace(turnRadius, turnRadius + exitLen, nExit)';

            points = [
                x_entry,    y_entry;
                x_leftArc(2:end),  y_leftArc(2:end);
                x_exit(2:end),     y_exit(2:end)
            ];

            obj = setPoints(obj, points, false, '90turn');
        end

        function obj = setPoints(obj, points, closed, name)
            % Store cleaned points and shared metadata.
            obj.Points = lts.components.Track.cleanPoints(points, closed);
            obj.Closed = closed;
            obj.Mu = 1.0;
            obj.Name = name;
        end
    end
end
