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
    %   6. Wheel Speed Dashboard - Four-corner wheel speeds and free-rolling error
    %   7. Powertrain Dashboard - Torque, drive force, speed overlay, force balance
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
    %   GraphPlotter.plotWheelSpeeds(stateLog, vehicle)
    %     Creates the four-corner wheel speed dashboard.
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
            if ~singleWindow
                GraphPlotter.plotWheelSpeeds(stateLog, vehicle);
            end
            GraphPlotter.plotPowertrain(stateLog, vehicle, singleWindow, 21);
        end
        
        function plotGeneralOverview(stateLog, lapTime, track, startIdx)
            % PLOTGENERALOVERVIEW Create the general overview dashboard
            %   GraphPlotter.plotGeneralOverview(stateLog, lapTime, track, startIdx)
            %
            %   Creates a 4-subplot figure with:
            %     1. Speed vs Distance
            %     2. Track Map colored by combined G-load (with lap time)
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
            combinedG = sqrt(axG.^2 + ayG.^2);
            
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
            
            % --- Track Map colored by G-load ---
            if isempty(startIdx), subplot(2,2,2); else, subplot(6,4,startIdx+1); end
            trackPts = track.getTrackPoints();
            arcLen = [0; cumsum(sqrt(diff(trackPts(:,1)).^2 + diff(trackPts(:,2)).^2))];
            xFit = interp1(arcLen, trackPts(:,1), stateLog.s, 'linear', 'extrap');
            yFit = interp1(arcLen, trackPts(:,2), stateLog.s, 'linear', 'extrap');
            scatter(xFit, yFit, 10, combinedG, 'filled');
            cb = colorbar;
            cb.Label.String = 'Combined G Load [g]';
            colormap('jet');
            xlabel('X [m]');
            ylabel('Y [m]');
            title(sprintf('Track Map G-Load (Lap: %.2f s)', lapTime));
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
            controlTime = time;
            controlS = s;
            if isfield(stateLog, 'controlTime')
                controlTime = stateLog.controlTime;
            end
            if isfield(stateLog, 'controlS')
                controlS = stateLog.controlS;
            end
            speedKmh = stateLog.speedKmh;
            throttlePct = stateLog.throttle * 100;
            brakePct = stateLog.brake * 100;
            brakeRequestedPct = brakePct;
            if isfield(stateLog, 'brakeRequested')
                brakeRequestedPct = stateLog.brakeRequested * 100;
            end
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
            hThrottle = plot(controlTime, throttlePct, 'g-', 'LineWidth', 1); hold on;
            hBrake = plot(controlTime, brakePct, 'r-', 'LineWidth', 1);
            hBrakeRequested = [];
            if any(abs(brakeRequestedPct - brakePct) > 1e-6)
                hBrakeRequested = plot(controlTime, brakeRequestedPct, '--', ...
                    'Color', [0.5 0 0], 'LineWidth', 0.75);
            end
            ylabel('Throttle / Brake [%]');
            ylim([-5 105]);
            yyaxis right;
            hSteer = plot(controlTime, steerDeg, 'b-', 'LineWidth', 1);
            ylabel('Steering [deg]');
            xlabel('Time [s]');
            title('Driver Inputs vs Time');
            if isempty(hBrakeRequested)
                legend([hThrottle hBrake hSteer], 'Throttle', 'Brake', 'Steer', 'Location', 'best');
            else
                legend([hThrottle hBrake hBrakeRequested hSteer], ...
                    'Throttle', 'Brake', 'Brake req.', 'Steer', 'Location', 'best');
            end
            grid on;

            % --- Driver inputs vs distance ---
            if useSingleFigure, subplot(6,4,startIdx+1); else, subplot(2,2,2); end
            yyaxis left;
            hThrottle = plot(controlS, throttlePct, 'g-', 'LineWidth', 1); hold on;
            hBrake = plot(controlS, brakePct, 'r-', 'LineWidth', 1);
            hBrakeRequested = [];
            if any(abs(brakeRequestedPct - brakePct) > 1e-6)
                hBrakeRequested = plot(controlS, brakeRequestedPct, '--', ...
                    'Color', [0.5 0 0], 'LineWidth', 0.75);
            end
            ylabel('Throttle / Brake [%]');
            ylim([-5 105]);
            yyaxis right;
            hSteer = plot(s, steerDeg, 'b-', 'LineWidth', 1);
            ylabel('Steering [deg]');
            xlabel('Distance [m]');
            title('Driver Inputs vs Distance');
            if isempty(hBrakeRequested)
                legend([hThrottle hBrake hSteer], 'Throttle', 'Brake', 'Steer', 'Location', 'best');
            else
                legend([hThrottle hBrake hBrakeRequested hSteer], ...
                    'Throttle', 'Brake', 'Brake req.', 'Steer', 'Location', 'best');
            end
            grid on;
            xlim([0 max(controlS)]);

            % --- Steering and curvature ---
            if useSingleFigure, subplot(6,4,startIdx+2); else, subplot(2,2,3); end
            yyaxis left;
            plot(controlS, steerDeg, 'b-', 'LineWidth', 1); hold on;
            yline(0, 'k-', 'LineWidth', 0.5);
            ylabel('Steering [deg]');
            yyaxis right;
            if isfield(stateLog, 'curvature')
                plot(controlS, stateLog.curvature, 'Color', [0.5 0 0.5], 'LineWidth', 1);
                ylabel('Curvature [1/m]');
            else
                plot(controlS, zeros(size(controlS)), 'Color', [0.5 0 0.5], 'LineWidth', 1);
                ylabel('Curvature [1/m]');
            end
            xlabel('Distance [m]');
            title('Steering & Curvature');
            grid on;
            xlim([0 max(controlS)]);

            % --- Speed vs distance with driver input color ---
            if useSingleFigure, subplot(6,4,startIdx+3); else, subplot(2,2,4); end
            inputColor = stateLog.throttle - stateLog.brake;
            scatter(controlS, speedKmh, 5, inputColor, 'filled');
            colormap(parula);
            cb = colorbar;
            cb.Label.String = 'Throttle - Brake';
            caxis([-1 1]);
            xlabel('Distance [m]');
            ylabel('Speed [km/h]');
            title('Speed Colored by Driver Input');
            grid on;
            xlim([0 max(controlS)]);
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
            %   Creates a 6-subplot figure when opened standalone:
            %     1. Damper Travel vs Distance (all 4 corners, mm)
            %     2. Damper Speed vs Distance (all 4 corners, mm/s)
            %     3. Camber vs Distance (all 4 corners, deg)
            %     4. Wheel Steer vs Distance (all 4 corners, deg)
            %     5. Damper Speed vs Travel phase plot
            %     6. Per-Corner Tire Loads and axle load transfer
            %
            %   In single-window mode, uses the existing 4 allocated tiles:
            %     travel vs distance, speed vs distance, tire loads, load transfer.
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
            travelMatrix = [damperFL_mm, damperFR_mm, damperRL_mm, damperRR_mm];
            
            % Convert damper velocity from m/s to mm/s
            velFL_mm = stateLog.damperVel_FL * 1000;
            velFR_mm = stateLog.damperVel_FR * 1000;
            velRL_mm = stateLog.damperVel_RL * 1000;
            velRR_mm = stateLog.damperVel_RR * 1000;
            speedMatrix = [velFL_mm, velFR_mm, velRL_mm, velRR_mm];

            % Geometry telemetry, if available
            hasGeometryTelemetry = isfield(stateLog, 'camber_FL') && ...
                isfield(stateLog, 'wheelSteer_FL');
            if hasGeometryTelemetry
                camberFL_deg = stateLog.camber_FL * 180 / pi;
                camberFR_deg = stateLog.camber_FR * 180 / pi;
                camberRL_deg = stateLog.camber_RL * 180 / pi;
                camberRR_deg = stateLog.camber_RR * 180 / pi;
                steerFL_deg = stateLog.wheelSteer_FL * 180 / pi;
                steerFR_deg = stateLog.wheelSteer_FR * 180 / pi;
                steerRL_deg = stateLog.wheelSteer_RL * 180 / pi;
                steerRR_deg = stateLog.wheelSteer_RR * 180 / pi;
            end
            
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
                figure('Name', 'LTS - Suspension', 'Position', [100 100 1500 1000]);
            end
            
            % --- Suspension Travel vs Distance ---
            if useSingleFigure, subplot(6,4,startIdx); else, subplot(3,2,1); end
            plot(s, damperFL_mm, '-', 'Color', colFL, 'LineWidth', 1); hold on;
            plot(s, damperFR_mm, '-', 'Color', colFR, 'LineWidth', 1);
            plot(s, damperRL_mm, '-', 'Color', colRL, 'LineWidth', 1);
            plot(s, damperRR_mm, '-', 'Color', colRR, 'LineWidth', 1);
            yline(bumpStopMM, 'k--', 'LineWidth', 1);
            yline(0, 'k-', 'LineWidth', 0.5);
            xlabel('Distance [m]');
            ylabel('Travel [mm]');
            title('Damper Travel vs Distance');
            legend('FL', 'FR', 'RL', 'RR', 'Bump Stop', 'Location', 'best');
            grid on;
            xlim([0 max(s)]);

            % --- Damper Speed vs Distance ---
            if useSingleFigure, subplot(6,4,startIdx+1); else, subplot(3,2,2); end
            plot(s, velFL_mm, '-', 'Color', colFL, 'LineWidth', 1); hold on;
            plot(s, velFR_mm, '-', 'Color', colFR, 'LineWidth', 1);
            plot(s, velRL_mm, '-', 'Color', colRL, 'LineWidth', 1);
            plot(s, velRR_mm, '-', 'Color', colRR, 'LineWidth', 1);
            yline(0, 'k-', 'LineWidth', 0.5);
            xlabel('Distance [m]');
            ylabel('Speed [mm/s]');
            title('Damper Speed vs Distance');
            legend('FL', 'FR', 'RL', 'RR', 'Location', 'best');
            grid on;
            xlim([0 max(s)]);
            
            % --- Per-Corner Tire Loads vs Distance ---
            if useSingleFigure, subplot(6,4,startIdx+2); else, subplot(3,2,6); end
            hFzFL = plot(s, Fz_FL, '-', 'Color', colFL, 'LineWidth', 1); hold on;
            hFzFR = plot(s, Fz_FR, '-', 'Color', colFR, 'LineWidth', 1);
            hFzRL = plot(s, Fz_RL, '-', 'Color', colRL, 'LineWidth', 1);
            hFzRR = plot(s, Fz_RR, '-', 'Color', colRR, 'LineWidth', 1);
            xlabel('Distance [m]');
            ylabel('Tire Normal Force [N]');
            title('Per-Corner Tire Loads');
            grid on;
            xlim([0 max(s)]);

            if ~useSingleFigure
                yyaxis right;
                hFrontTransfer = plot(s, frontLoadTransfer, 'b--', 'LineWidth', 1);
                hRearTransfer = plot(s, rearLoadTransfer, 'r--', 'LineWidth', 1);
                yline(0, 'k-', 'LineWidth', 0.5);
                ylabel('Axle Load Transfer [N]');
                title('Tire Loads & Axle Transfer');
                legend([hFzFL hFzFR hFzRL hFzRR hFrontTransfer hRearTransfer], ...
                    'FL F_z', 'FR F_z', 'RL F_z', 'RR F_z', ...
                    'Front transfer', 'Rear transfer', 'Location', 'best');
                yyaxis left;
            else
                legend('FL', 'FR', 'RL', 'RR', 'Location', 'best');
            end
            
            if ~useSingleFigure
                % --- Camber vs Distance ---
                subplot(3,2,3);
                if hasGeometryTelemetry
                    plot(s, camberFL_deg, '-', 'Color', colFL, 'LineWidth', 1); hold on;
                    plot(s, camberFR_deg, '-', 'Color', colFR, 'LineWidth', 1);
                    plot(s, camberRL_deg, '-', 'Color', colRL, 'LineWidth', 1);
                    plot(s, camberRR_deg, '-', 'Color', colRR, 'LineWidth', 1);
                end
                yline(0, 'k-', 'LineWidth', 0.5);
                xlabel('Distance [m]');
                ylabel('Camber [deg]');
                title('Camber vs Distance');
                legend('FL', 'FR', 'RL', 'RR', 'Location', 'best');
                grid on;
                xlim([0 max(s)]);

                % --- Wheel Steer vs Distance ---
                subplot(3,2,4);
                if hasGeometryTelemetry
                    plot(s, steerFL_deg, '-', 'Color', colFL, 'LineWidth', 1); hold on;
                    plot(s, steerFR_deg, '-', 'Color', colFR, 'LineWidth', 1);
                    plot(s, steerRL_deg, '-', 'Color', colRL, 'LineWidth', 1);
                    plot(s, steerRR_deg, '-', 'Color', colRR, 'LineWidth', 1);
                end
                yline(0, 'k-', 'LineWidth', 0.5);
                xlabel('Distance [m]');
                ylabel('Wheel Steer [deg]');
                title('Wheel Steer vs Distance');
                legend('FL', 'FR', 'RL', 'RR', 'Location', 'best');
                grid on;
                xlim([0 max(s)]);
            end
            
            if ~useSingleFigure
                % --- Damper Speed vs Travel ---
                subplot(3,2,5);
                plot(damperFL_mm, velFL_mm, '.', 'Color', colFL, 'MarkerSize', 2); hold on;
                plot(damperFR_mm, velFR_mm, '.', 'Color', colFR, 'MarkerSize', 2);
                plot(damperRL_mm, velRL_mm, '.', 'Color', colRL, 'MarkerSize', 2);
                plot(damperRR_mm, velRR_mm, '.', 'Color', colRR, 'MarkerSize', 2);
                xline(bumpStopMM, 'k--', 'LineWidth', 1);
                xline(0, 'k-', 'LineWidth', 0.5);
                yline(0, 'k-', 'LineWidth', 0.5);
                xlabel('Travel [mm]');
                ylabel('Speed [mm/s]');
                title('Damper Speed vs Travel');
                legend('FL', 'FR', 'RL', 'RR', 'Bump Stop', 'Location', 'best');
                grid on;

                % Keep y-range readable if startup transients create one or two spikes.
                allSpeeds = speedMatrix(:);
                finiteSpeeds = allSpeeds(isfinite(allSpeeds));
                if numel(finiteSpeeds) > 10
                    sortedSpeeds = sort(abs(finiteSpeeds));
                    speedIdx = max(1, ceil(0.99 * numel(sortedSpeeds)));
                    speedLimit = sortedSpeeds(speedIdx);
                    if speedLimit > 0
                        ylim([-speedLimit speedLimit]);
                    end
                end

                % Add a compact travel/speed summary in the corner-load plot.
                subplot(3,2,6);
                maxTravel = max(travelMatrix, [], 1);
                minTravel = min(travelMatrix, [], 1);
                peakSpeed = max(abs(speedMatrix), [], 1);
                text(0.99, 0.95, sprintf(['Max travel [mm]: FL %.1f | FR %.1f | RL %.1f | RR %.1f\n' ...
                    'Min travel [mm]: FL %.1f | FR %.1f | RL %.1f | RR %.1f\n' ...
                    'Peak speed [mm/s]: FL %.0f | FR %.0f | RL %.0f | RR %.0f'], ...
                    maxTravel(1), maxTravel(2), maxTravel(3), maxTravel(4), ...
                    minTravel(1), minTravel(2), minTravel(3), minTravel(4), ...
                    peakSpeed(1), peakSpeed(2), peakSpeed(3), peakSpeed(4)), ...
                    'Units', 'normalized', 'HorizontalAlignment', 'right', ...
                    'VerticalAlignment', 'top', 'FontSize', 8, ...
                    'BackgroundColor', 'w', 'EdgeColor', [0.8 0.8 0.8]);
            else
                % --- Front/Rear Axle Load Transfer vs Time ---
                subplot(6,4,startIdx+3);
                plot(time, frontLoadTransfer, 'b-', 'LineWidth', 1.5); hold on;
                plot(time, rearLoadTransfer, 'r-', 'LineWidth', 1.5);
                yline(0, 'k-', 'LineWidth', 0.5);
                xlabel('Time [s]');
                ylabel('Load Transfer [N]');
                title('Axle Load Transfer (from static)');
                legend('Front axle', 'Rear axle', 'Location', 'best');
                grid on;
            end
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

        function plotWheelSpeeds(stateLog, vehicle, useSingleFigure, startIdx)
            % PLOTWHEELSPEEDS Create the four-corner wheel-speed dashboard
            %   GraphPlotter.plotWheelSpeeds(stateLog, vehicle, useSingleFigure, startIdx)
            %
            %   Creates a 4-subplot figure with:
            %     1. Four-corner linear wheel speed vs time with vehicle speed
            %     2. Four-corner wheel-speed error vs time
            %     3. Four-corner angular wheel speed vs time
            %     4. Four-corner slip ratio vs time

            if nargin < 3, useSingleFigure = false; end
            if nargin < 4, startIdx = 1; end

            time = stateLog.time;
            speedKmh = stateLog.speedKmh;

            colFL = [0 0.447 0.741];
            colFR = [0.850 0.325 0.098];
            colRL = [0.467 0.675 0.188];
            colRR = [0.929 0.694 0.125];

            hasWheelOmega = isfield(stateLog, 'omega_FL') && ...
                isfield(stateLog, 'omega_FR') && ...
                isfield(stateLog, 'omega_RL') && ...
                isfield(stateLog, 'omega_RR');

            if ~useSingleFigure
                figure('Name', 'LTS - Wheel Speeds', 'Position', [175 175 1500 950]);
            end

            if ~hasWheelOmega
                if useSingleFigure, subplot(6,4,startIdx); else, subplot(2,2,1); end
                text(0.5, 0.5, 'No wheel-speed telemetry available', ...
                    'HorizontalAlignment', 'center', 'Units', 'normalized');
                return;
            end

            R = vehicle.tire.FL.wheelRadius;
            wheelSpeedFL = stateLog.omega_FL * R * 3.6;
            wheelSpeedFR = stateLog.omega_FR * R * 3.6;
            wheelSpeedRL = stateLog.omega_RL * R * 3.6;
            wheelSpeedRR = stateLog.omega_RR * R * 3.6;

            wheelRpmFL = stateLog.omega_FL * 60 / (2 * pi);
            wheelRpmFR = stateLog.omega_FR * 60 / (2 * pi);
            wheelRpmRL = stateLog.omega_RL * 60 / (2 * pi);
            wheelRpmRR = stateLog.omega_RR * 60 / (2 * pi);

            % --- Linear wheel speed vs time ---
            if useSingleFigure, subplot(6,4,startIdx); else, subplot(2,2,1); end
            plot(time, speedKmh, 'k--', 'LineWidth', 1); hold on;
            plot(time, wheelSpeedFL, '-', 'Color', colFL, 'LineWidth', 1);
            plot(time, wheelSpeedFR, '-', 'Color', colFR, 'LineWidth', 1);
            plot(time, wheelSpeedRL, '-', 'Color', colRL, 'LineWidth', 1);
            plot(time, wheelSpeedRR, '-', 'Color', colRR, 'LineWidth', 1);
            xlabel('Time [s]');
            ylabel('Speed [km/h]');
            title('Wheel Linear Speed vs Time');
            legend('Vehicle', 'FL', 'FR', 'RL', 'RR', 'Location', 'best');
            grid on;

            % --- Wheel speed error vs time ---
            if useSingleFigure, subplot(6,4,startIdx+1); else, subplot(2,2,2); end
            plot(time, wheelSpeedFL - speedKmh, '-', 'Color', colFL, 'LineWidth', 1); hold on;
            plot(time, wheelSpeedFR - speedKmh, '-', 'Color', colFR, 'LineWidth', 1);
            plot(time, wheelSpeedRL - speedKmh, '-', 'Color', colRL, 'LineWidth', 1);
            plot(time, wheelSpeedRR - speedKmh, '-', 'Color', colRR, 'LineWidth', 1);
            yline(0, 'k-', 'LineWidth', 0.5);
            xlabel('Time [s]');
            ylabel('Wheel - Vehicle [km/h]');
            title('Wheel Speed Error');
            legend('FL', 'FR', 'RL', 'RR', 'Location', 'best');
            grid on;

            % --- Angular wheel speed vs time ---
            if useSingleFigure, subplot(6,4,startIdx+2); else, subplot(2,2,3); end
            plot(time, wheelRpmFL, '-', 'Color', colFL, 'LineWidth', 1); hold on;
            plot(time, wheelRpmFR, '-', 'Color', colFR, 'LineWidth', 1);
            plot(time, wheelRpmRL, '-', 'Color', colRL, 'LineWidth', 1);
            plot(time, wheelRpmRR, '-', 'Color', colRR, 'LineWidth', 1);
            xlabel('Time [s]');
            ylabel('Wheel Speed [rpm]');
            title('Wheel Angular Speed');
            legend('FL', 'FR', 'RL', 'RR', 'Location', 'best');
            grid on;

            % --- Slip ratio vs time ---
            if useSingleFigure, subplot(6,4,startIdx+3); else, subplot(2,2,4); end
            if isfield(stateLog, 'slipRatio_FL')
                plot(time, stateLog.slipRatio_FL, '-', 'Color', colFL, 'LineWidth', 1); hold on;
                plot(time, stateLog.slipRatio_FR, '-', 'Color', colFR, 'LineWidth', 1);
                plot(time, stateLog.slipRatio_RL, '-', 'Color', colRL, 'LineWidth', 1);
                plot(time, stateLog.slipRatio_RR, '-', 'Color', colRR, 'LineWidth', 1);
                yline(0, 'k-', 'LineWidth', 0.5);
                xlabel('Time [s]');
                ylabel('Slip Ratio [-]');
                title('Slip Ratio From Wheel Speed');
                legend('FL', 'FR', 'RL', 'RR', 'Location', 'best');
                grid on;
            else
                text(0.5, 0.5, 'No slip-ratio telemetry available', ...
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
            if isfield(stateLog, 'F_brake')
                F_brake_plot = stateLog.F_brake;
            else
                W = vehicle.totalMass * 9.81;
                F_brake_plot = zeros(numel(time), 1);
                brakeIdx = stateLog.brake > 0.01;
                if any(brakeIdx)
                    maxBrakeForce = vehicle.brakeForceCoefficient * ...
                        (W + stateLog.F_downforce(brakeIdx));
                    F_brake_plot(brakeIdx) = -stateLog.brake(brakeIdx) .* maxBrakeForce;
                end
            end
            
            area(time, [stateLog.F_drive, -stateLog.F_drag, F_brake_plot], ...
                'LineStyle', 'none');
            legend('Drive', '-Drag', 'Brake', 'Location', 'best');
            xlabel('Time [s]');
            ylabel('Force [N]');
            title('Force Balance');
            grid on;
        end
        
    end
end
