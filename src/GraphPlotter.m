classdef GraphPlotter
    % GRAPHPLOTTER Centralized simulation result visualization
    % Static utility class that creates component-based dashboard figures.
    % Each dashboard focuses on one vehicle subsystem, plus a general overview.
    %
    % Dashboards:
    %   1. General Overview   - Speed, track map, accelerations, g-g diagram
    %   2. Driver Dashboard   - Throttle, brake, steering, and input traces
    %   3. Aero Dashboard     - Aero forces, balance, pitch, axle loads
    %   4. Suspension Dashboard - Travel, damper velocity, tire loads, load transfer
    %   5. Tire Dashboard     - Slip ratios, wheel speeds, tire forces, utilization
    %   6. Powertrain Dashboard - Torque, drive force, speed overlay, force balance
    %
    % Usage:
    %   GraphPlotter.plotAll(stateLog, lapTime, track, vehicle, aero)
    %     Creates all 6 dashboard figures (separate windows).
    %
    %   GraphPlotter.plotAll(stateLog, lapTime, track, vehicle, aero, true)
    %     Creates all dashboards in a single window (6x4 subplot grid).
    %
    %   GraphPlotter.plotGeneralOverview(stateLog, lapTime, track)
    %     Creates the general overview dashboard.
    %
    %   GraphPlotter.plotDriver(stateLog)
    %     Creates the driver input dashboard.
    %
    %   GraphPlotter.plotAero(stateLog, vehicle, aero)
    %     Creates the aerodynamics dashboard.
    %
    %   GraphPlotter.plotSuspension(stateLog, vehicle)
    %     Creates the suspension telemetry dashboard.
    %
    %   GraphPlotter.plotTire(stateLog, vehicle)
    %     Creates the tire performance dashboard.
    %
    %   GraphPlotter.plotPowertrain(stateLog, vehicle)
    %     Creates the powertrain dashboard.
    
    methods(Static)
        
        function plotAll(stateLog, lapTime, track, vehicle, aero, singleWindow)
            % PLOTALL Create all post-simulation dashboard figures
            %   GraphPlotter.plotAll(..., singleWindow)
            %
            %   singleWindow = false (default) -> 6 separate figures
            %   singleWindow = true            -> one figure with 6x4 subplot grid
            
            if nargin < 6
                singleWindow = false;
            end
            
            if singleWindow
                figure('Name', 'LTS - All Dashboards', 'Position', [10 10 2400 1600]);
                startIdx = 1;
            else
                startIdx = [];
            end
            
            GraphPlotter.plotGeneralOverview(stateLog, lapTime, track, startIdx);
            GraphPlotter.plotDriver(stateLog, singleWindow, 5);
            GraphPlotter.plotAero(stateLog, vehicle, aero, singleWindow, 9);
            GraphPlotter.plotSuspension(stateLog, vehicle, singleWindow, 13);
            GraphPlotter.plotTire(stateLog, vehicle, singleWindow, 17);
            GraphPlotter.plotPowertrain(stateLog, vehicle, singleWindow, 21);
        end
        
        function plotGeneralOverview(stateLog, lapTime, track, startIdx)
            % PLOTGENERALOVERVIEW Create the general overview dashboard
            %   GraphPlotter.plotGeneralOverview(stateLog, lapTime, track, startIdx)
            %
            %   Creates a 4-subplot figure with:
            %     1. Speed vs Distance
            %     2. Track Map colored by speed (with lap time)
            %     3. Longitudinal & Lateral Acceleration vs Time
            %     4. g-g Diagram (ax vs ay)
            %
            %   startIdx (optional): when provided, plots into subplot(6,4,startIdx+N)
            %     instead of creating a new figure.
            
            if nargin < 4
                startIdx = [];
            end
            
            speedKmh = stateLog.speedKmh;
            time = stateLog.time;
            s = stateLog.s;
            axG = stateLog.ax / 9.81;
            ayG = stateLog.ay / 9.81;
            
            if isempty(startIdx)
                figure('Name', 'LTS - General Overview', 'Position', [50 50 1400 900]);
            end
            
            % --- Speed vs Distance ---
            if isempty(startIdx), subplot(2,2,1); else, subplot(6,4,startIdx); end
            plot(s, speedKmh, 'b-', 'LineWidth', 1.5);
            xlabel('Distance [m]');
            ylabel('Speed [km/h]');
            title('Speed vs Distance');
            grid on;
            xlim([0 max(s)]);
            
            % --- Track Map colored by speed ---
            if isempty(startIdx), subplot(2,2,2); else, subplot(6,4,startIdx+1); end
            trackPts = track.getTrackPoints();
            arcLen = [0; cumsum(sqrt(diff(trackPts(:,1)).^2 + diff(trackPts(:,2)).^2))];
            xFit = interp1(arcLen, trackPts(:,1), stateLog.s, 'linear', 'extrap');
            yFit = interp1(arcLen, trackPts(:,2), stateLog.s, 'linear', 'extrap');
            scatter(xFit, yFit, 10, speedKmh, 'filled');
            colorbar;
            colormap('jet');
            xlabel('X [m]');
            ylabel('Y [m]');
            title(sprintf('Track Map (Lap: %.2f s)', lapTime));
            axis equal;
            grid on;
            
            % --- Longitudinal and Lateral Acceleration ---
            if isempty(startIdx), subplot(2,2,3); else, subplot(6,4,startIdx+2); end
            plot(time, axG, 'b-', 'LineWidth', 1); hold on;
            plot(time, ayG, 'r-', 'LineWidth', 1);
            xlabel('Time [s]');
            ylabel('Acceleration [g]');
            title('Accelerations');
            legend('a_x', 'a_y', 'Location', 'best');
            grid on;
            
            % --- g-g Diagram ---
            if isempty(startIdx), subplot(2,2,4); else, subplot(6,4,startIdx+3); end
            scatter(ayG, axG, 5, speedKmh, 'filled');
            colorbar;
            colormap('jet');
            xlabel('Lateral Accel [g]');
            ylabel('Longitudinal Accel [g]');
            title('g-g Diagram');
            axis equal;
            grid on;
            % Draw reference circles
            hold on;
            theta = linspace(0, 2*pi, 100);
            for gLevel = [0.5 1.0 1.5 2.0]
                plot(gLevel*cos(theta), gLevel*sin(theta), 'Color', [0.7 0.7 0.7], 'LineStyle', ':', 'LineWidth', 0.5);
            end
        end

        function plotDriver(stateLog, useSingleFigure, startIdx)
            % PLOTDRIVER Create the driver input dashboard
            %   GraphPlotter.plotDriver(stateLog, useSingleFigure, startIdx)
            %
            %   Creates a 4-subplot figure with:
            %     1. Throttle, brake, and steering vs time
            %     2. Throttle, brake, and steering vs distance
            %     3. Steering and curvature vs distance
            %     4. Speed vs distance colored by longitudinal input
            %
            %   useSingleFigure, startIdx (optional): when provided, plots into
            %     subplot(6,4,startIdx+N) instead of creating a new figure.

            if nargin < 2, useSingleFigure = false; end
            if nargin < 3, startIdx = 1; end

            time = stateLog.time;
            s = stateLog.s;
            speedKmh = stateLog.speedKmh;
            throttlePct = stateLog.throttle * 100;
            brakePct = stateLog.brake * 100;
            if isfield(stateLog, 'steer')
                steerDeg = stateLog.steer * 180 / pi;
            else
                steerDeg = zeros(size(time));
            end

            if ~useSingleFigure
                figure('Name', 'LTS - Driver', 'Position', [60 60 1400 900]);
            end

            % --- Driver inputs vs time ---
            if useSingleFigure, subplot(6,4,startIdx); else, subplot(2,2,1); end
            yyaxis left;
            hThrottle = plot(time, throttlePct, 'g-', 'LineWidth', 1); hold on;
            hBrake = plot(time, brakePct, 'r-', 'LineWidth', 1);
            ylabel('Throttle / Brake [%]');
            ylim([-5 105]);
            yyaxis right;
            hSteer = plot(time, steerDeg, 'b-', 'LineWidth', 1);
            ylabel('Steering [deg]');
            xlabel('Time [s]');
            title('Driver Inputs vs Time');
            legend([hThrottle hBrake hSteer], 'Throttle', 'Brake', 'Steer', 'Location', 'best');
            grid on;

            % --- Driver inputs vs distance ---
            if useSingleFigure, subplot(6,4,startIdx+1); else, subplot(2,2,2); end
            yyaxis left;
            hThrottle = plot(s, throttlePct, 'g-', 'LineWidth', 1); hold on;
            hBrake = plot(s, brakePct, 'r-', 'LineWidth', 1);
            ylabel('Throttle / Brake [%]');
            ylim([-5 105]);
            yyaxis right;
            hSteer = plot(s, steerDeg, 'b-', 'LineWidth', 1);
            ylabel('Steering [deg]');
            xlabel('Distance [m]');
            title('Driver Inputs vs Distance');
            legend([hThrottle hBrake hSteer], 'Throttle', 'Brake', 'Steer', 'Location', 'best');
            grid on;
            xlim([0 max(s)]);

            % --- Steering and curvature ---
            if useSingleFigure, subplot(6,4,startIdx+2); else, subplot(2,2,3); end
            yyaxis left;
            plot(s, steerDeg, 'b-', 'LineWidth', 1); hold on;
            yline(0, 'k-', 'LineWidth', 0.5);
            ylabel('Steering [deg]');
            yyaxis right;
            if isfield(stateLog, 'curvature')
                plot(s, stateLog.curvature, 'Color', [0.5 0 0.5], 'LineWidth', 1);
                ylabel('Curvature [1/m]');
            else
                plot(s, zeros(size(s)), 'Color', [0.5 0 0.5], 'LineWidth', 1);
                ylabel('Curvature [1/m]');
            end
            xlabel('Distance [m]');
            title('Steering & Curvature');
            grid on;
            xlim([0 max(s)]);

            % --- Speed vs distance with driver input color ---
            if useSingleFigure, subplot(6,4,startIdx+3); else, subplot(2,2,4); end
            inputColor = stateLog.throttle - stateLog.brake;
            scatter(s, speedKmh, 5, inputColor, 'filled');
            colormap(parula);
            cb = colorbar;
            cb.Label.String = 'Throttle - Brake';
            caxis([-1 1]);
            xlabel('Distance [m]');
            ylabel('Speed [km/h]');
            title('Speed Colored by Driver Input');
            grid on;
            xlim([0 max(s)]);
        end
        
        function plotAero(stateLog, vehicle, aero, useSingleFigure, startIdx)
            % PLOTAERO Create the aerodynamics dashboard
            %   GraphPlotter.plotAero(stateLog, vehicle, aero, useSingleFigure, startIdx)
            %
            %   Creates a 4-subplot figure with:
            %     1. Aero Forces vs Time (Downforce, Drag, Drive)
            %     2. Aero Axle Loads by Speed (bar chart)
            %     3. Front/Rear Aero Balance vs Time
            %     4. Pitch Angle vs Time
            %
            %   useSingleFigure, startIdx (optional): when provided, plots into
            %     subplot(6,4,startIdx+N) instead of creating a new figure.
            
            if nargin < 4, useSingleFigure = false; end
            if nargin < 5, startIdx = 1; end
            
            time = stateLog.time;
            speedKmh = stateLog.speedKmh;
            
            if ~useSingleFigure
                figure('Name', 'LTS - Aero', 'Position', [75 75 1400 900]);
            end
            
            % --- Aero Forces vs Time ---
            if useSingleFigure, subplot(6,4,startIdx); else, subplot(2,2,1); end
            plot(time, stateLog.F_downforce, 'b-', 'LineWidth', 1); hold on;
            plot(time, stateLog.F_drag, 'r-', 'LineWidth', 1);
            plot(time, stateLog.F_drive, 'g-', 'LineWidth', 1);
            xlabel('Time [s]');
            ylabel('Force [N]');
            title('Aerodynamic & Drive Forces');
            legend('Downforce', 'Drag', 'Drive', 'Location', 'best');
            grid on;
            
            % --- Aero Axle Loads by Speed (bar chart) ---
            if useSingleFigure, subplot(6,4,startIdx+1); else, subplot(2,2,2); end
            sampleSpeeds = [10, 15, 20, 25, 30, 35];
            sampleFf = zeros(1, numel(sampleSpeeds));
            sampleFr = zeros(1, numel(sampleSpeeds));
            sampleFd = zeros(1, numel(sampleSpeeds));
            for j = 1:numel(sampleSpeeds)
                tempState = VehicleState('speed', sampleSpeeds(j), 'ax', 0, 'pitchAngle', 0, 'rideHeight', 0);
                tempState.vehicleManager = vehicle;
                af = aero.computeForces(tempState);
                sampleFf(j) = af.Fz_front;
                sampleFr(j) = af.Fz_rear;
                sampleFd(j) = af.F_drag;
            end
            bar(sampleSpeeds * 3.6, [sampleFf', sampleFr', sampleFd'], 'grouped');
            xlabel('Speed [km/h]');
            ylabel('Force [N]');
            title('Aero Loads by Axle');
            legend('Front axle', 'Rear axle', 'Drag', 'Location', 'northwest');
            grid on;
            
            % --- Front/Rear Aero Balance vs Time ---
            if useSingleFigure, subplot(6,4,startIdx+2); else, subplot(2,2,3); end
            % Use logged aero axle forces (computed during simulation)
            totalAeroFz = stateLog.aeroFz_front + stateLog.aeroFz_rear;
            frontPct = zeros(numel(time), 1);
            validIdx = totalAeroFz > 10;  % avoid division by near-zero
            frontPct(validIdx) = stateLog.aeroFz_front(validIdx) ./ totalAeroFz(validIdx) * 100;
            frontPct(~validIdx) = NaN;
            
            plot(time, frontPct, 'b-', 'LineWidth', 1); hold on;
            yline(vehicle.staticFrontWeight * 100, 'k--', 'Static Weight Dist.', 'LineWidth', 1);
            xlabel('Time [s]');
            ylabel('Front Aero Load [%]');
            title('Aero Balance (Front %)');
            legend('Aero front %', 'Static weight %', 'Location', 'best');
            grid on;
            ylim([0 100]);
            
            % --- Pitch Angle vs Time ---
            if useSingleFigure, subplot(6,4,startIdx+3); else, subplot(2,2,4); end
            pitchDeg = stateLog.pitchAngle * (180/pi);
            plot(time, pitchDeg, 'm-', 'LineWidth', 1);
            xlabel('Time [s]');
            ylabel('Pitch Angle [deg]');
            title('Vehicle Pitch');
            grid on;
        end
        
        function plotSuspension(stateLog, vehicle, useSingleFigure, startIdx)
            % PLOTSUSPENSION Create the suspension telemetry dashboard
            %   GraphPlotter.plotSuspension(stateLog, vehicle, useSingleFigure, startIdx)
            %
            %   Creates a 4-subplot figure with:
            %     1. Suspension Travel vs Distance (all 4 corners, mm)
            %     2. Per-Corner Tire Loads vs Distance (N)
            %     3. Damper Velocity vs Time (all 4 corners, mm/s)
            %     4. Front/Rear Axle Load Transfer vs Time (N)
            %
            %   useSingleFigure, startIdx (optional): when provided, plots into
            %     subplot(6,4,startIdx+N) instead of creating a new figure.
            
            if nargin < 3, useSingleFigure = false; end
            if nargin < 4, startIdx = 1; end
            
            s = stateLog.s;
            time = stateLog.time;
            
            % Convert damper position from m to mm
            damperFL_mm = stateLog.damperPos_FL * 1000;
            damperFR_mm = stateLog.damperPos_FR * 1000;
            damperRL_mm = stateLog.damperPos_RL * 1000;
            damperRR_mm = stateLog.damperPos_RR * 1000;
            
            % Convert damper velocity from m/s to mm/s
            velFL_mm = stateLog.damperVel_FL * 1000;
            velFR_mm = stateLog.damperVel_FR * 1000;
            velRL_mm = stateLog.damperVel_RL * 1000;
            velRR_mm = stateLog.damperVel_RR * 1000;
            
            % Corner tire normal forces
            Fz_FL = stateLog.Fz_FL;
            Fz_FR = stateLog.Fz_FR;
            Fz_RL = stateLog.Fz_RL;
            Fz_RR = stateLog.Fz_RR;
            
            % Compute front/rear axle load transfer
            Fz_front_total = Fz_FL + Fz_FR;
            Fz_rear_total  = Fz_RL + Fz_RR;
            
            % Static axle loads for reference
            W = vehicle.totalMass * 9.81;
            Fz_static_front = W * vehicle.staticFrontWeight;
            Fz_static_rear  = W * (1 - vehicle.staticFrontWeight);
            
            % Load transfer = deviation from static
            frontLoadTransfer = Fz_front_total - Fz_static_front;
            rearLoadTransfer  = Fz_rear_total  - Fz_static_rear;
            
            % Bump stop threshold (from suspension params)
            bumpStopMM = vehicle.suspension.frontLeft.bumpStopLength * 1000;
            
            % Corner colors
            colFL = [0 0.447 0.741];       % blue
            colFR = [0.850 0.325 0.098];   % orange-red
            colRL = [0.467 0.675 0.188];   % green
            colRR = [0.929 0.694 0.125];   % gold
            
            if ~useSingleFigure
                figure('Name', 'LTS - Suspension', 'Position', [100 100 1400 900]);
            end
            
            % --- Suspension Travel vs Distance ---
            if useSingleFigure, subplot(6,4,startIdx); else, subplot(2,2,1); end
            plot(s, damperFL_mm, '-', 'Color', colFL, 'LineWidth', 1); hold on;
            plot(s, damperFR_mm, '-', 'Color', colFR, 'LineWidth', 1);
            plot(s, damperRL_mm, '-', 'Color', colRL, 'LineWidth', 1);
            plot(s, damperRR_mm, '-', 'Color', colRR, 'LineWidth', 1);
            yline(bumpStopMM, 'k--', 'LineWidth', 1);
            yline(0, 'k-', 'LineWidth', 0.5);
            xlabel('Distance [m]');
            ylabel('Damper Displacement [mm]');
            title('Suspension Travel');
            legend('FL', 'FR', 'RL', 'RR', 'Bump Stop', 'Location', 'best');
            grid on;
            xlim([0 max(s)]);
            
            % --- Per-Corner Tire Loads vs Distance ---
            if useSingleFigure, subplot(6,4,startIdx+1); else, subplot(2,2,2); end
            plot(s, Fz_FL, '-', 'Color', colFL, 'LineWidth', 1); hold on;
            plot(s, Fz_FR, '-', 'Color', colFR, 'LineWidth', 1);
            plot(s, Fz_RL, '-', 'Color', colRL, 'LineWidth', 1);
            plot(s, Fz_RR, '-', 'Color', colRR, 'LineWidth', 1);
            xlabel('Distance [m]');
            ylabel('Tire Normal Force [N]');
            title('Per-Corner Tire Loads');
            legend('FL', 'FR', 'RL', 'RR', 'Location', 'best');
            grid on;
            xlim([0 max(s)]);
            
            % --- Damper Velocity vs Time ---
            if useSingleFigure, subplot(6,4,startIdx+2); else, subplot(2,2,3); end
            plot(time, velFL_mm, '-', 'Color', colFL, 'LineWidth', 1); hold on;
            plot(time, velFR_mm, '-', 'Color', colFR, 'LineWidth', 1);
            plot(time, velRL_mm, '-', 'Color', colRL, 'LineWidth', 1);
            plot(time, velRR_mm, '-', 'Color', colRR, 'LineWidth', 1);
            yline(0, 'k-', 'LineWidth', 0.5);
            xlabel('Time [s]');
            ylabel('Damper Velocity [mm/s]');
            title('Damper Velocity');
            legend('FL', 'FR', 'RL', 'RR', 'Location', 'best');
            grid on;
            
            % --- Front/Rear Axle Load Transfer vs Time ---
            if useSingleFigure, subplot(6,4,startIdx+3); else, subplot(2,2,4); end
            plot(time, frontLoadTransfer, 'b-', 'LineWidth', 1.5); hold on;
            plot(time, rearLoadTransfer, 'r-', 'LineWidth', 1.5);
            yline(0, 'k-', 'LineWidth', 0.5);
            xlabel('Time [s]');
            ylabel('Load Transfer [N]');
            title('Axle Load Transfer (from static)');
            legend('Front axle', 'Rear axle', 'Location', 'best');
            grid on;
        end
        
        function plotTire(stateLog, vehicle, useSingleFigure, startIdx)
            % PLOTTIRE Create the tire performance dashboard
            %   GraphPlotter.plotTire(stateLog, vehicle, useSingleFigure, startIdx)
            %
            %   Creates a 4-subplot figure with:
            %     1. Per-Corner Slip Ratio vs Time
            %     2. Wheel Speed vs Vehicle Speed (4 corners)
            %     3. Per-Corner Tire Longitudinal Force Fx vs Time
            %     4. Per-Corner Tire Lateral Force Fy vs Time
            %
            %   useSingleFigure, startIdx (optional): when provided, plots into
            %     subplot(6,4,startIdx+N) instead of creating a new figure.
            
            if nargin < 3, useSingleFigure = false; end
            if nargin < 4, startIdx = 1; end
            
            time = stateLog.time;
            speed = stateLog.speed;
            speedKmh = stateLog.speedKmh;
            
            % Corner colors
            colFL = [0 0.447 0.741];       % blue
            colFR = [0.850 0.325 0.098];   % orange-red
            colRL = [0.467 0.675 0.188];   % green
            colRR = [0.929 0.694 0.125];   % gold
            
            if ~useSingleFigure
                figure('Name', 'LTS - Tire', 'Position', [125 125 1400 900]);
            end
            
            % --- Per-Corner Slip Ratio vs Time ---
            if useSingleFigure, subplot(6,4,startIdx); else, subplot(2,2,1); end
            plot(time, stateLog.slipRatio_FL, '-', 'Color', colFL, 'LineWidth', 1); hold on;
            plot(time, stateLog.slipRatio_FR, '-', 'Color', colFR, 'LineWidth', 1);
            plot(time, stateLog.slipRatio_RL, '-', 'Color', colRL, 'LineWidth', 1);
            plot(time, stateLog.slipRatio_RR, '-', 'Color', colRR, 'LineWidth', 1);
            yline(0, 'k-', 'LineWidth', 0.5);
            xlabel('Time [s]');
            ylabel('Slip Ratio [-]');
            title('Per-Corner Slip Ratio');
            legend('FL', 'FR', 'RL', 'RR', 'Location', 'best');
            grid on;
            
            % --- Wheel Speed vs Vehicle Speed ---
            if useSingleFigure, subplot(6,4,startIdx+1); else, subplot(2,2,2); end
            % Get wheel radius for converting omega to linear speed
            if isfield(stateLog, 'omega_FL') && any(stateLog.omega_FL ~= 0)
                R = vehicle.tire.FL.wheelRadius;
                wheelSpeedFL = stateLog.omega_FL * R * 3.6;  % km/h
                wheelSpeedFR = stateLog.omega_FR * R * 3.6;
                wheelSpeedRL = stateLog.omega_RL * R * 3.6;
                wheelSpeedRR = stateLog.omega_RR * R * 3.6;
                
                plot(speedKmh, wheelSpeedFL, '.', 'Color', colFL, 'MarkerSize', 1); hold on;
                plot(speedKmh, wheelSpeedFR, '.', 'Color', colFR, 'MarkerSize', 1);
                plot(speedKmh, wheelSpeedRL, '.', 'Color', colRL, 'MarkerSize', 1);
                plot(speedKmh, wheelSpeedRR, '.', 'Color', colRR, 'MarkerSize', 1);
                % Reference line: wheel speed = vehicle speed
                maxSpd = max(speedKmh) * 1.1;
                plot([0 maxSpd], [0 maxSpd], 'k--', 'LineWidth', 1);
                xlabel('Vehicle Speed [km/h]');
                ylabel('Wheel Speed [km/h]');
                title('Wheel Speed vs Vehicle Speed');
                legend('FL', 'FR', 'RL', 'RR', 'No-slip line', 'Location', 'best');
                grid on;
                axis equal;
                xlim([0 maxSpd]);
                ylim([0 maxSpd]);
            else
                text(0.5, 0.5, 'No Pacejka tire data available', ...
                    'HorizontalAlignment', 'center', 'Units', 'normalized');
            end
            
            % --- Per-Corner Tire Longitudinal Force Fx ---
            if useSingleFigure, subplot(6,4,startIdx+2); else, subplot(2,2,3); end
            if isfield(stateLog, 'tireFx_FL')
                plot(time, stateLog.tireFx_FL, '-', 'Color', colFL, 'LineWidth', 1); hold on;
                plot(time, stateLog.tireFx_FR, '-', 'Color', colFR, 'LineWidth', 1);
                plot(time, stateLog.tireFx_RL, '-', 'Color', colRL, 'LineWidth', 1);
                plot(time, stateLog.tireFx_RR, '-', 'Color', colRR, 'LineWidth', 1);
                yline(0, 'k-', 'LineWidth', 0.5);
                xlabel('Time [s]');
                ylabel('Longitudinal Force Fx [N]');
                title('Per-Corner Tire Fx');
                legend('FL', 'FR', 'RL', 'RR', 'Location', 'best');
                grid on;
            else
                text(0.5, 0.5, 'No tire force data available', ...
                    'HorizontalAlignment', 'center', 'Units', 'normalized');
            end
            
            % --- Per-Corner Tire Lateral Force Fy ---
            if useSingleFigure, subplot(6,4,startIdx+3); else, subplot(2,2,4); end
            if isfield(stateLog, 'tireFy_FL')
                plot(time, stateLog.tireFy_FL, '-', 'Color', colFL, 'LineWidth', 1); hold on;
                plot(time, stateLog.tireFy_FR, '-', 'Color', colFR, 'LineWidth', 1);
                plot(time, stateLog.tireFy_RL, '-', 'Color', colRL, 'LineWidth', 1);
                plot(time, stateLog.tireFy_RR, '-', 'Color', colRR, 'LineWidth', 1);
                yline(0, 'k-', 'LineWidth', 0.5);
                xlabel('Time [s]');
                ylabel('Lateral Force Fy [N]');
                title('Per-Corner Tire Fy');
                legend('FL', 'FR', 'RL', 'RR', 'Location', 'best');
                grid on;
            else
                text(0.5, 0.5, 'No tire force data available', ...
                    'HorizontalAlignment', 'center', 'Units', 'normalized');
            end
        end
        
        function plotPowertrain(stateLog, vehicle, useSingleFigure, startIdx)
            % PLOTPOWERTRAIN Create the powertrain dashboard
            %   GraphPlotter.plotPowertrain(stateLog, vehicle, useSingleFigure, startIdx)
            %
            %   Creates a 4-subplot figure with:
            %     1. Motor and wheel torque vs time
            %     2. Drive Force vs Time
            %     3. Speed vs Distance with powertrain color overlay
            %     4. Force Balance (Drive, Drag, Brake) vs Time
            %
            %   useSingleFigure, startIdx (optional): when provided, plots into
            %     subplot(6,4,startIdx+N) instead of creating a new figure.
            
            if nargin < 3, useSingleFigure = false; end
            if nargin < 4, startIdx = 1; end
            
            time = stateLog.time;
            s = stateLog.s;
            speedKmh = stateLog.speedKmh;
            
            if ~useSingleFigure
                figure('Name', 'LTS - Powertrain', 'Position', [150 150 1400 900]);
            end
            
            % --- Motor and wheel torque ---
            if useSingleFigure, subplot(6,4,startIdx); else, subplot(2,2,1); end
            hasMotorTorque = isfield(stateLog, 'motorTorque');
            hasWheelTorque = isfield(stateLog, 'wheelTorque');
            if hasMotorTorque || hasWheelTorque
                if hasMotorTorque
                    yyaxis left;
                    plot(time, stateLog.motorTorque, 'm-', 'LineWidth', 1); hold on;
                    ylabel('Motor Torque [Nm]');
                end
                if hasWheelTorque
                    yyaxis right;
                    plot(time, stateLog.wheelTorque, 'Color', [0.1 0.45 0.9], 'LineWidth', 1);
                    ylabel('Wheel Torque [Nm]');
                end
                xlabel('Time [s]');
                title('Motor & Wheel Torque');
                grid on;
            else
                text(0.5, 0.5, 'No torque data available', ...
                    'HorizontalAlignment', 'center', 'Units', 'normalized');
            end
            
            % --- Drive Force vs Time ---
            if useSingleFigure, subplot(6,4,startIdx+1); else, subplot(2,2,2); end
            yyaxis left;
            plot(time, stateLog.F_drive, 'g-', 'LineWidth', 1.5);
            xlabel('Time [s]');
            ylabel('Drive Force [N]');
            title('Drive Force & Motor Speed');
            grid on;
            
            if isfield(stateLog, 'motorRPM')
                yyaxis right;
                plot(time, stateLog.motorRPM, 'm-', 'LineWidth', 1);
                ylabel('Motor Speed [rpm]');
                if isprop(vehicle.powertrain, 'rpmFalloffStartRPM') && vehicle.powertrain.rpmFalloffStartRPM > 0
                    yline(vehicle.powertrain.rpmFalloffStartRPM, 'm:', 'Falloff start');
                end
                if isprop(vehicle.powertrain, 'rpmLimitRPM') && vehicle.powertrain.rpmLimitRPM > 0
                    yline(vehicle.powertrain.rpmLimitRPM, 'm--', 'RPM cap');
                end
            end
            
            % --- Speed vs Distance with powertrain color overlay ---
            if useSingleFigure, subplot(6,4,startIdx+2); else, subplot(2,2,3); end
            if isfield(stateLog, 'motorRPM')
                powertrainColor = stateLog.motorRPM;
                colorLabel = 'Motor Speed [rpm]';
            else
                powertrainColor = stateLog.F_drive;
                colorLabel = 'Drive Force [N]';
            end
            scatter(s, speedKmh, 5, powertrainColor, 'filled');
            colormap(parula);
            cb = colorbar;
            cb.Label.String = colorLabel;
            xlabel('Distance [m]');
            ylabel('Speed [km/h]');
            title('Speed vs Distance (powertrain)');
            grid on;
            xlim([0 max(s)]);
            
            % --- Force Balance vs Time ---
            if useSingleFigure, subplot(6,4,startIdx+3); else, subplot(2,2,4); end
            % Reconstruct approximate brake force from logged data
            W = vehicle.totalMass * 9.81;
            F_brake_approx = zeros(numel(time), 1);
            brakeIdx = stateLog.brake > 0.01;
            if any(brakeIdx)
                maxBrakeForce = 0.7 * W + stateLog.F_downforce(brakeIdx) * 0.7;
                F_brake_approx(brakeIdx) = -stateLog.brake(brakeIdx) .* maxBrakeForce;
            end
            
            area(time, [stateLog.F_drive, -stateLog.F_drag, F_brake_approx], ...
                'LineStyle', 'none');
            legend('Drive', '-Drag', 'Brake', 'Location', 'best');
            xlabel('Time [s]');
            ylabel('Force [N]');
            title('Force Balance');
            grid on;
        end
        
    end
end
