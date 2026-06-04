classdef GraphPlotter
    % GRAPHPLOTTER Centralized simulation result visualization
    % Static utility class that creates all post-simulation figures.
    % Separates plotting concerns from simulation setup and execution.
    %
    % Usage:
    %   GraphPlotter.plotAll(stateLog, lapTime, track, vehicle, aero)
    %     Creates both the vehicle dynamics figure and suspension figure.
    %
    %   GraphPlotter.plotVehicleDynamics(stateLog, lapTime, track, vehicle, aero)
    %     Creates the main 8-subplot vehicle dynamics figure.
    %
    %   GraphPlotter.plotSuspension(stateLog, vehicle)
    %     Creates the 4-subplot suspension telemetry figure.
    
    methods(Static)
        
        function plotAll(stateLog, lapTime, track, vehicle, aero)
            % PLOTALL Create all post-simulation figures
            %   GraphPlotter.plotAll(stateLog, lapTime, track, vehicle, aero)
            GraphPlotter.plotVehicleDynamics(stateLog, lapTime, track, vehicle, aero);
            GraphPlotter.plotSuspension(stateLog, vehicle);
        end
        
        function plotVehicleDynamics(stateLog, lapTime, track, vehicle, aero)
            % PLOTVEICLEDYNAMICS Create the main vehicle dynamics figure
            %   GraphPlotter.plotVehicleDynamics(stateLog, lapTime, track, vehicle, aero)
            %
            %   Creates an 8-subplot figure with:
            %     1. Speed vs Distance
            %     2. Speed vs Time
            %     3. Longitudinal & Lateral Acceleration
            %     4. Throttle & Brake Inputs
            %     5. Aerodynamic & Drive Forces
            %     6. Vehicle Pitch Angle
            %     7. Track Map colored by speed
            %     8. Aero Axle Loads by Speed (bar chart)
            
            % Convert to km/h for display
            speedKmh = stateLog.speedKmh;
            time = stateLog.time;
            s = stateLog.s;
            
            figure('Name', 'LTS Results', 'Position', [50 50 1400 900]);
            
            % --- Speed vs Distance ---
            subplot(2,4,1);
            plot(s, speedKmh, 'b-', 'LineWidth', 1.5);
            xlabel('Distance [m]');
            ylabel('Speed [km/h]');
            title('Speed vs Distance');
            grid on;
            xlim([0 max(s)]);
            
            % --- Speed vs Time ---
            subplot(2,4,2);
            plot(time, speedKmh, 'r-', 'LineWidth', 1.5);
            xlabel('Time [s]');
            ylabel('Speed [km/h]');
            title('Speed vs Time');
            grid on;
            
            % --- Longitudinal and Lateral Acceleration ---
            subplot(2,4,3);
            axG = stateLog.ax / 9.81;
            ayG = stateLog.ay / 9.81;
            plot(time, axG, 'b-', 'LineWidth', 1); hold on;
            plot(time, ayG, 'r-', 'LineWidth', 1);
            xlabel('Time [s]');
            ylabel('Acceleration [g]');
            title('Accelerations');
            legend('a_x', 'a_y', 'Location', 'best');
            grid on;
            
            % --- Throttle and Brake ---
            subplot(2,4,4);
            plot(time, stateLog.throttle * 100, 'g-', 'LineWidth', 1); hold on;
            plot(time, stateLog.brake * 100, 'r-', 'LineWidth', 1);
            xlabel('Time [s]');
            ylabel('Input [%]');
            title('Driver Inputs');
            legend('Throttle', 'Brake', 'Location', 'best');
            grid on;
            ylim([-5 105]);
            
            % --- Forces ---
            subplot(2,4,5);
            plot(time, stateLog.F_downforce, 'b-', 'LineWidth', 1); hold on;
            plot(time, stateLog.F_drag, 'r-', 'LineWidth', 1);
            plot(time, stateLog.F_drive, 'g-', 'LineWidth', 1);
            xlabel('Time [s]');
            ylabel('Force [N]');
            title('Forces');
            legend('Downforce', 'Drag', 'Drive', 'Location', 'best');
            grid on;
            
            % --- Pitch Angle ---
            subplot(2,4,6);
            pitchDeg = stateLog.pitchAngle * (180/pi);
            plot(time, pitchDeg, 'm-', 'LineWidth', 1);
            xlabel('Time [s]');
            ylabel('Pitch Angle [deg]');
            title('Vehicle Pitch');
            grid on;
            
            % --- Track Map (colored by speed) ---
            subplot(2,4,7);
            trackPts = track.getTrackPoints();
            
            % Interpolate track position from logged s
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
            
            % --- Aero axle loads vs speed ---
            subplot(2,4,8);
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
        end
        
        function plotSuspension(stateLog, vehicle)
            % PLOTSUSPENSION Create the suspension telemetry figure
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
            colFL = [0 0.447 0.741];  % blue
            colFR = [0.850 0.325 0.098];  % orange-red
            colRL = [0.467 0.675 0.188];  % green
            colRR = [0.929 0.694 0.125];  % gold
            
            figure('Name', 'LTS Suspension', 'Position', [100 100 1400 900]);
            
            % --- Suspension Travel vs Distance ---
            subplot(2,2,1);
            plot(s, damperFL_mm, '-', 'Color', colFL, 'LineWidth', 1); hold on;
            plot(s, damperFR_mm, '-', 'Color', colFR, 'LineWidth', 1);
            plot(s, damperRL_mm, '-', 'Color', colRL, 'LineWidth', 1);
            plot(s, damperRR_mm, '-', 'Color', colRR, 'LineWidth', 1);
            yline(bumpStopMM, 'k--', 'LineWidth', 1, 'Alpha', 0.5);
            yline(0, 'k-', 'LineWidth', 0.5, 'Alpha', 0.3);
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
            yline(0, 'k-', 'LineWidth', 0.5, 'Alpha', 0.3);
            xlabel('Time [s]');
            ylabel('Damper Velocity [mm/s]');
            title('Damper Velocity');
            legend('FL', 'FR', 'RL', 'RR', 'Location', 'best');
            grid on;
            
            % --- Front/Rear Axle Load Transfer vs Time ---
            subplot(2,2,4);
            plot(time, frontLoadTransfer, 'b-', 'LineWidth', 1.5); hold on;
            plot(time, rearLoadTransfer, 'r-', 'LineWidth', 1.5);
            yline(0, 'k-', 'LineWidth', 0.5, 'Alpha', 0.3);
            xlabel('Time [s]');
            ylabel('Load Transfer [N]');
            title('Axle Load Transfer (from static)');
            legend('Front axle', 'Rear axle', 'Location', 'best');
            grid on;
        end
        
    end
end