classdef GraphPlotter
    % GRAPHPLOTTER Centralized simulation result visualization
    % Static utility class that creates component-based dashboard figures.
    % Each dashboard focuses on one vehicle subsystem, plus a general overview.
    %
    % Dashboards:
    %   1. General Overview   - Speed, track map, accelerations, g-g diagram
    %   2. Aero Dashboard     - Aero forces, balance, pitch, axle loads
    %   3. Suspension Dashboard - Travel, damper velocity, tire loads, load transfer
    %   4. Tire Dashboard     - Slip ratios, wheel speeds, tire forces, utilization
    %   5. Powertrain Dashboard - Driver inputs, drive force, speed overlay, force balance
    %
    % Usage:
    %   GraphPlotter.plotAll(stateLog, lapTime, track, vehicle, aero)
    %     Creates all 5 dashboard figures.
    %
    %   GraphPlotter.plotGeneralOverview(stateLog, lapTime, track)
    %     Creates the general overview dashboard.
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
        
        function plotAll(stateLog, lapTime, track, vehicle, aero)
            % PLOTALL Create all post-simulation dashboard figures
            %   GraphPlotter.plotAll(stateLog, lapTime, track, vehicle, aero)
            GraphPlotter.plotGeneralOverview(stateLog, lapTime, track);
            GraphPlotter.plotAero(stateLog, vehicle, aero);
            GraphPlotter.plotSuspension(stateLog, vehicle);
            GraphPlotter.plotTire(stateLog, vehicle);
            GraphPlotter.plotPowertrain(stateLog, vehicle);
        end
        
        function plotGeneralOverview(stateLog, lapTime, track)
            % PLOTGENERALOVERVIEW Create the general overview dashboard
            %   GraphPlotter.plotGeneralOverview(stateLog, lapTime, track)
            %
            %   Creates a 4-subplot figure with:
            %     1. Speed vs Distance
            %     2. Track Map colored by speed (with lap time)
            %     3. Longitudinal & Lateral Acceleration vs Time
            %     4. g-g Diagram (ax vs ay)
            
            speedKmh = stateLog.speedKmh;
            time = stateLog.time;
            s = stateLog.s;
            axG = stateLog.ax / 9.81;
            ayG = stateLog.ay / 9.81;
            
            figure('Name', 'LTS - General Overview', 'Position', [50 50 1400 900]);
            
            % --- Speed vs Distance ---
            subplot(2,2,1);
            plot(s, speedKmh, 'b-', 'LineWidth', 1.5);
            xlabel('Distance [m]');
            ylabel('Speed [km/h]');
            title('Speed vs Distance');
            grid on;
            xlim([0 max(s)]);
            
            % --- Track Map colored by speed ---
            subplot(2,2,2);
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
            subplot(2,2,3);
            plot(time, axG, 'b-', 'LineWidth', 1); hold on;
            plot(time, ayG, 'r-', 'LineWidth', 1);
            xlabel('Time [s]');
            ylabel('Acceleration [g]');
            title('Accelerations');
            legend('a_x', 'a_y', 'Location', 'best');
            grid on;
            
            % --- g-g Diagram ---
            subplot(2,2,4);
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
        
        function plotAero(stateLog, vehicle, aero)
            % PLOTAERO Create the aerodynamics dashboard
            %   GraphPlotter.plotAero(stateLog, vehicle, aero)
            %
            %   Creates a 4-subplot figure with:
            %     1. Aero Forces vs Time (Downforce, Drag, Drive)
            %     2. Aero Axle Loads by Speed (bar chart)
            %     3. Front/Rear Aero Balance vs Time
            %     4. Pitch Angle vs Time
            
            time = stateLog.time;
            speedKmh = stateLog.speedKmh;
            
            figure('Name', 'LTS - Aero', 'Position', [75 75 1400 900]);
            
            % --- Aero Forces vs Time ---
            subplot(2,2,1);
            plot(time, stateLog.F_downforce, 'b-', 'LineWidth', 1); hold on;
            plot(time, stateLog.F_drag, 'r-', 'LineWidth', 1);
            plot(time, stateLog.F_drive, 'g-', 'LineWidth', 1);
            xlabel('Time [s]');
            ylabel('Force [N]');
            title('Aerodynamic & Drive Forces');
            legend('Downforce', 'Drag', 'Drive', 'Location', 'best');
            grid on;
            
            % --- Aero Axle Loads by Speed (bar chart) ---
            subplot(2,2,2);
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
            subplot(2,2,3);
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
            subplot(2,2,4);
            pitchDeg = stateLog.pitchAngle * (180/pi);
            plot(time, pitchDeg, 'm-', 'LineWidth', 1);
            xlabel('Time [s]');
            ylabel('Pitch Angle [deg]');
            title('Vehicle Pitch');
            grid on;
        end
        
        function plotSuspension(stateLog, vehicle)
            % PLOTSUSPENSION Create the suspension telemetry dashboard
            %   GraphPlotter.plotSuspension(stateLog, vehicle)
            %
            %   Creates a 4-subplot figure with:
            %     1. Suspension Travel vs Distance (all 4 corners, mm)
            %     2. Per-Corner Tire Loads vs Distance (N)
            %     3. Damper Velocity vs Time (all 4 corners, mm/s)
            %     4. Front/Rear Axle Load Transfer vs Time (N)
            
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
            
            figure('Name', 'LTS - Suspension', 'Position', [100 100 1400 900]);
            
            % --- Suspension Travel vs Distance ---
            subplot(2,2,1);
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
            subplot(2,2,2);
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
            subplot(2,2,3);
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
            subplot(2,2,4);
            plot(time, frontLoadTransfer, 'b-', 'LineWidth', 1.5); hold on;
            plot(time, rearLoadTransfer, 'r-', 'LineWidth', 1.5);
            yline(0, 'k-', 'LineWidth', 0.5);
            xlabel('Time [s]');
            ylabel('Load Transfer [N]');
            title('Axle Load Transfer (from static)');
            legend('Front axle', 'Rear axle', 'Location', 'best');
            grid on;
        end
        
        function plotTire(stateLog, vehicle)
            % PLOTTIRE Create the tire performance dashboard
            %   GraphPlotter.plotTire(stateLog, vehicle)
            %
            %   Creates a 4-subplot figure with:
            %     1. Per-Corner Slip Ratio vs Time
            %     2. Wheel Speed vs Vehicle Speed (4 corners)
            %     3. Per-Corner Tire Longitudinal Force Fx vs Time
            %     4. Per-Corner Tire Lateral Force Fy vs Time
            
            time = stateLog.time;
            speed = stateLog.speed;
            speedKmh = stateLog.speedKmh;
            
            % Corner colors
            colFL = [0 0.447 0.741];       % blue
            colFR = [0.850 0.325 0.098];   % orange-red
            colRL = [0.467 0.675 0.188];   % green
            colRR = [0.929 0.694 0.125];   % gold
            
            figure('Name', 'LTS - Tire', 'Position', [125 125 1400 900]);
            
            % --- Per-Corner Slip Ratio vs Time ---
            subplot(2,2,1);
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
            subplot(2,2,2);
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
            subplot(2,2,3);
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
            subplot(2,2,4);
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
        
        function plotPowertrain(stateLog, vehicle)
            % PLOTPOWERTRAIN Create the powertrain dashboard
            %   GraphPlotter.plotPowertrain(stateLog, vehicle)
            %
            %   Creates a 4-subplot figure with:
            %     1. Throttle & Brake Inputs vs Time
            %     2. Drive Force vs Time
            %     3. Speed vs Distance with throttle/brake color overlay
            %     4. Force Balance (Drive, Drag, Brake) vs Time
            
            time = stateLog.time;
            s = stateLog.s;
            speedKmh = stateLog.speedKmh;
            
            figure('Name', 'LTS - Powertrain', 'Position', [150 150 1400 900]);
            
            % --- Throttle & Brake Inputs ---
            subplot(2,2,1);
            plot(time, stateLog.throttle * 100, 'g-', 'LineWidth', 1); hold on;
            plot(time, stateLog.brake * 100, 'r-', 'LineWidth', 1);
            xlabel('Time [s]');
            ylabel('Input [%]');
            title('Driver Inputs');
            legend('Throttle', 'Brake', 'Location', 'best');
            grid on;
            ylim([-5 105]);
            
            % --- Drive Force vs Time ---
            subplot(2,2,2);
            plot(time, stateLog.F_drive, 'g-', 'LineWidth', 1.5);
            xlabel('Time [s]');
            ylabel('Drive Force [N]');
            title('Drive Force');
            grid on;
            
            % --- Speed vs Distance with throttle/brake color overlay ---
            subplot(2,2,3);
            % Color by driver input: green=throttle, red=brake, gray=coast
            inputColor = stateLog.throttle - stateLog.brake;  % +1=full throttle, -1=full brake
            scatter(s, speedKmh, 5, inputColor, 'filled');
            colormap(parula);
            cb = colorbar;
            cb.Label.String = 'Throttle - Brake';
            caxis([-1 1]);
            xlabel('Distance [m]');
            ylabel('Speed [km/h]');
            title('Speed vs Distance (colored by input)');
            grid on;
            xlim([0 max(s)]);
            
            % --- Force Balance vs Time ---
            subplot(2,2,4);
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