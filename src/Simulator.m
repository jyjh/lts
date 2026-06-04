classdef Simulator
    % SIMULATOR Physics engine and simulation loop for vehicle dynamics
    %
    % Core concept: given a VehicleState and driver inputs, progress the
    % state from one timestep to the next (copy-in → copy-out).
    %
    % Two modes of use:
    %   1. Single step:  [newState, forces] = sim.step(state, throttle, brake, curKappa, curMu, curHeading)
    %   2. Full lap:     [stateLog, lapTime] = sim.simulate(initialState, track)
    %
    % The Simulator composes a VehicleManager (physics components) and a
    % DriverModel (throttle/brake decisions).
    
    properties
        % Reference to VehicleManager (components + vehicle parameters)
        vehicleManager
        
        % Reference to DriverModel (computes throttle/brake inputs)
        driverModel
    end
    
    methods
        function obj = Simulator(vehicleManager, driverModel)
            % SIMULATOR Construct with a VehicleManager and DriverModel
            %   Simulator(vehicleManager, driverModel)
            obj.vehicleManager = vehicleManager;
            obj.driverModel = driverModel;
        end
        
        function [newState, forces] = step(obj, state, throttle, brake, curKappa, curMu, curHeading)
            % STEP Progress vehicle state by one timestep
            %   [newState, forces] = step(state, throttle, brake, curKappa, curMu, curHeading)
            %
            %   Given a VehicleState snapshot and driver inputs, compute the
            %   next VehicleState. The input state is NOT mutated.
            %
            %   Inputs:
            %     state       - current VehicleState (not modified)
            %     throttle    - throttle position [0-1]
            %     brake       - brake pressure [0-1]
            %     curKappa    - track curvature at current position [1/m]
            %     curMu       - surface friction at current position
            %     curHeading  - track heading at current position [rad]
            %
            %   Outputs:
            %     newState    - VehicleState at next timestep
            %     forces      - struct with F_downforce, F_drag, F_drive
            
            vm = obj.vehicleManager;
            v = state.speed;
            
            % Copy state (will be mutated by updateFromDynamics)
            newState = state;
            
            % --- AERODYNAMIC FORCES ---
            aeroForces = vm.aero.computeForces(state);
            F_downforce = aeroForces.Fz_front + aeroForces.Fz_rear;
            F_drag = aeroForces.F_drag;
            
            % --- WEIGHT AND PER-CORNER LOADS ---
            W = vm.totalMass * 9.81;
            
            cornerLoads = vm.suspension.computeCornerLoads( ...
                state, aeroForces.Fz_front, aeroForces.Fz_rear, vm.totalMass, vm.dt);
            
            Fz_front = cornerLoads.FL + cornerLoads.FR;
            Fz_rear  = cornerLoads.RL + cornerLoads.RR;
            Fz_front = max(0, Fz_front);
            Fz_rear  = max(0, Fz_rear);
            
            % --- DRIVE FORCE ---
            F_drive = vm.powertrain.computeDriveForce(v, throttle);
            
            % --- BRAKE FORCE ---
            maxBrakeForce = 0.7 * W + F_downforce * 0.7;
            F_brake = -brake * maxBrakeForce;
            
            % --- MAX CORNERING SPEED ---
            totalNormalLoad = W + F_downforce;
            peakMu = vm.tire.getPeakFriction(totalNormalLoad / 4);
            maxLateralAccel = peakMu * 9.81;
            maxAx = peakMu * 9.81;
            
            if abs(curKappa) > 1e-6
                vMaxCorner = sqrt(maxLateralAccel / abs(curKappa));
            else
                vMaxCorner = vm.maxSpeed;
            end
            
            % --- LONGITUDINAL FORCE BALANCE ---
            F_net_long = F_drive + F_brake - F_drag;
            F_rollResist = 0.015 * (W + F_downforce);
            F_net_long = F_net_long - sign(v) * F_rollResist;
            
            ax = F_net_long / vm.totalMass;
            ax = max(-maxAx, min(ax, maxAx));
            
            % --- SPEED LIMITERS ---
            if v > vMaxCorner && abs(curKappa) > 1e-6
                excessSpeed = v - vMaxCorner;
                brakingAccel = -min(excessSpeed / vm.dt * 0.5, maxAx);
                ax = min(ax, brakingAccel);
            end
            
            if v >= vm.maxSpeed
                ax = min(ax, 0);
            end
            
            % --- LATERAL DYNAMICS ---
            if abs(curKappa) > 1e-6 && v > 0.5
                ay = v^2 * curKappa;
            else
                ay = 0;
            end
            ay = max(-maxLateralAccel, min(ay, maxLateralAccel));
            
            % --- INTEGRATE STATE ---
            ds = max(0, v * vm.dt + 0.5 * ax * vm.dt^2);
            
            newState.throttle = throttle;
            newState.brake = brake;
            newState = newState.updateFromDynamics(ax, ay, ds, vm.dt, curKappa, curHeading, curMu);
            
            % --- RETURN FORCES ---
            forces.F_downforce = F_downforce;
            forces.F_drag = F_drag;
            forces.F_drive = F_drive;
        end
        
        function [stateLog, lapTime] = simulate(obj, initialState, track)
            % SIMULATE Run the full lap simulation
            %   [stateLog, lapTime] = simulate(initialState, track)
            %
            %   initialState - VehicleState at simulation start
            %   track        - Track object with geometry and surface data
            
            vm = obj.vehicleManager;
            
            % Set vehicleManager reference on state so components can access constants
            initialState.vehicleManager = vm;
            
            % Get track data
            trackPts   = track.getTrackPoints();
            curvature  = track.getCurvature();
            mu         = track.getSurfaceFriction();
            heading    = track.getHeading();
            trackLen   = track.getTotalLength();
            nPts       = size(trackPts, 1);
            
            % Compute arc-length parameterization
            dx = diff(trackPts(:,1));
            dy = diff(trackPts(:,2));
            segLen = sqrt(dx.^2 + dy.^2);
            arcLen = [0; cumsum(segLen)];
            
            % Pre-allocate telemetry log
            maxSteps = round(trackLen / (initialState.speed * vm.dt) * 5);
            maxSteps = max(maxSteps, 100000);
            stateLog = struct( ...
                'time',        zeros(maxSteps, 1), ...
                's',           zeros(maxSteps, 1), ...
                'speed',       zeros(maxSteps, 1), ...
                'speedKmh',    zeros(maxSteps, 1), ...
                'ax',          zeros(maxSteps, 1), ...
                'ay',          zeros(maxSteps, 1), ...
                'throttle',    zeros(maxSteps, 1), ...
                'brake',       zeros(maxSteps, 1), ...
                'curvature',   zeros(maxSteps, 1), ...
                'heading',     zeros(maxSteps, 1), ...
                'F_downforce', zeros(maxSteps, 1), ...
                'F_drag',      zeros(maxSteps, 1), ...
                'F_drive',     zeros(maxSteps, 1), ...
                'pitchAngle',  zeros(maxSteps, 1), ...
                'Fz_FL',       zeros(maxSteps, 1), ...
                'Fz_FR',       zeros(maxSteps, 1), ...
                'Fz_RL',       zeros(maxSteps, 1), ...
                'Fz_RR',       zeros(maxSteps, 1), ...
                'damperPos_FL', zeros(maxSteps, 1), ...
                'damperPos_FR', zeros(maxSteps, 1), ...
                'damperPos_RL', zeros(maxSteps, 1), ...
                'damperPos_RR', zeros(maxSteps, 1), ...
                'damperVel_FL', zeros(maxSteps, 1), ...
                'damperVel_FR', zeros(maxSteps, 1), ...
                'damperVel_RL', zeros(maxSteps, 1), ...
                'damperVel_RR', zeros(maxSteps, 1) ...
            );
            
            % Working state (will be updated each step)
            currentState = initialState;
            
            step = 0;
            fprintf('Starting simulation...\n');
            fprintf('Track length: %.1f m\n', trackLen);
            
            while currentState.s < trackLen && currentState.onTrack
                step = step + 1;
                
                % Find current track index by nearest arc-length
                idx = find(arcLen <= currentState.s, 1, 'last');
                idx = max(1, min(idx, nPts));
                
                % Current track properties
                curKappa   = curvature(idx);
                curMu      = mu(idx);
                curHeading = heading(idx);
                
                % Set current track properties on state (for DriverModel)
                currentState.curvature = curKappa;
                currentState.mu        = curMu;
                
                % --- DRIVER MODEL: Compute throttle and brake ---
                [throttle, brake] = obj.driverModel.computeInputs(currentState);
                
                % --- PHYSICS STEP ---
                [newState, forces] = obj.step( ...
                    currentState, throttle, brake, curKappa, curMu, curHeading);
                
                % --- LOG TELEMETRY ---
                if step <= maxSteps
                    stateLog.time(step)        = newState.time;
                    stateLog.s(step)           = newState.s;
                    stateLog.speed(step)       = newState.speed;
                    stateLog.speedKmh(step)    = newState.speed * 3.6;
                    stateLog.ax(step)          = newState.ax;
                    stateLog.ay(step)          = newState.ay;
                    stateLog.throttle(step)    = throttle;
                    stateLog.brake(step)       = brake;
                    stateLog.curvature(step)   = curKappa;
                    stateLog.heading(step)     = curHeading;
                    stateLog.F_downforce(step) = forces.F_downforce;
                    stateLog.F_drag(step)      = forces.F_drag;
                    stateLog.F_drive(step)     = forces.F_drive;
                    stateLog.pitchAngle(step)  = newState.pitchAngle;
                    
                    % Per-corner suspension telemetry
                    susp = vm.suspension;
                    stateLog.Fz_FL(step)       = susp.frontLeft.state.tireNormalForce;
                    stateLog.Fz_FR(step)       = susp.frontRight.state.tireNormalForce;
                    stateLog.Fz_RL(step)       = susp.rearLeft.state.tireNormalForce;
                    stateLog.Fz_RR(step)       = susp.rearRight.state.tireNormalForce;
                    stateLog.damperPos_FL(step) = susp.frontLeft.state.damperPosition;
                    stateLog.damperPos_FR(step) = susp.frontRight.state.damperPosition;
                    stateLog.damperPos_RL(step) = susp.rearLeft.state.damperPosition;
                    stateLog.damperPos_RR(step) = susp.rearRight.state.damperPosition;
                    stateLog.damperVel_FL(step) = susp.frontLeft.state.damperVelocity;
                    stateLog.damperVel_FR(step) = susp.frontRight.state.damperVelocity;
                    stateLog.damperVel_RL(step) = susp.rearLeft.state.damperVelocity;
                    stateLog.damperVel_RR(step) = susp.rearRight.state.damperVelocity;
                end
                
                % Advance state
                currentState = newState;
                
                % Progress display
                if mod(step, 5000) == 0
                    progress = currentState.s / trackLen * 100;
                    fprintf('  Progress: %5.1f%% | Speed: %5.1f km/h | s: %6.1f m\n', ...
                        progress, currentState.speed * 3.6, currentState.s);
                end
                
                % Safety: prevent infinite loops
                if step >= maxSteps
                    warning('Simulation reached maximum steps (%d). Stopping.', maxSteps);
                    break;
                end
            end
            
            % Trim logs
            fields = fieldnames(stateLog);
            for i = 1:numel(fields)
                stateLog.(fields{i}) = stateLog.(fields{i})(1:step);
            end
            
            lapTime = currentState.time;
            
            fprintf('\n=== Simulation Complete ===\n');
            fprintf('Lap Time:   %.3f s\n', lapTime);
            fprintf('Track Length: %.1f m\n', currentState.s);
            fprintf('Max Speed:  %.1f km/h\n', max(stateLog.speedKmh));
            fprintf('Steps:      %d\n', step);
        end
    end
end