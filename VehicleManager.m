classdef VehicleManager
    % VEHICLEMANAGER Composes all vehicle components and runs the simulation
    % This is the main simulation orchestrator using the strategy pattern
    
    properties
        % Swappable component objects
        aero        % components.AeroManager (or components.AeroComponent for legacy)
        suspension  % components.SuspensionComponent
        powertrain  % components.PowertrainComponent
        tire        % components.TireModel
        track       % components.Track
        
        % Vehicle parameters
        totalMass     = 280      % Total mass with driver [kg]
        wheelbase     = 1.55     % Wheelbase [m]
        trackWidth    = 1.2      % Track width [m]
        cgHeight      = 0.28     % CG height [m]
        
        % Simulation parameters
        dt            = 0.001    % Timestep [s]
        maxSpeed      = 40       % Speed limiter [m/s] (~144 km/h)
        brakingLookahead = 1.0   % Lookahead factor for braking distance
        
        % Vehicle state
        state = VehicleState()
    end
    
    methods
        function obj = VehicleManager(aero, suspension, powertrain, tire, track, varargin)
            % VEHICLEMANAGER Construct with all component objects
            %   VehicleManager(aero, suspension, powertrain, tire, track)
            %   Optional: name-value pairs for vehicle parameters
            
            obj.aero = aero;
            obj.suspension = suspension;
            obj.powertrain = powertrain;
            obj.tire = tire;
            obj.track = track;
            
            % Parse optional parameters
            for i = 1:2:nargin-5
                if isprop(obj, varargin{i})
                    obj.(varargin{i}) = varargin{i+1};
                end
            end
        end
        
        function [stateLog, lapTime] = simulate(obj)
            % SIMULATE Run the full lap simulation
            % Returns logged telemetry and total lap time
            
            % Initialize state
            obj.state = VehicleState('s', 0, 'speed', 0.1);
            
            % Get track data
            trackPts   = obj.track.getTrackPoints();
            curvature  = obj.track.getCurvature();
            mu         = obj.track.getSurfaceFriction();
            heading    = obj.track.getHeading();
            trackLen   = obj.track.getTotalLength();
            nPts       = size(trackPts, 1);
            
            % Compute arc-length parameterization
            dx = diff(trackPts(:,1));
            dy = diff(trackPts(:,2));
            segLen = sqrt(dx.^2 + dy.^2);
            arcLen = [0; cumsum(segLen)];
            
            % Pre-allocate telemetry log
            maxSteps = round(trackLen / (obj.state.speed * obj.dt) * 5);  % generous estimate
            maxSteps = max(maxSteps, 100000);
            stateLog = struct( ...
                'time',      zeros(maxSteps, 1), ...
                's',         zeros(maxSteps, 1), ...
                'speed',     zeros(maxSteps, 1), ...
                'speedKmh',  zeros(maxSteps, 1), ...
                'ax',        zeros(maxSteps, 1), ...
                'ay',        zeros(maxSteps, 1), ...
                'throttle',  zeros(maxSteps, 1), ...
                'brake',     zeros(maxSteps, 1), ...
                'curvature', zeros(maxSteps, 1), ...
                'heading',   zeros(maxSteps, 1), ...
                'F_downforce', zeros(maxSteps, 1), ...
                'F_drag',      zeros(maxSteps, 1), ...
                'F_drive',     zeros(maxSteps, 1), ...
                'pitchAngle',  zeros(maxSteps, 1) ...
            );
            
            step = 0;
            fprintf('Starting simulation...\n');
            fprintf('Track length: %.1f m\n', trackLen);
            
            while obj.state.s < trackLen && obj.state.onTrack
                step = step + 1;
                
                % Find current track index by nearest arc-length
                idx = find(arcLen <= obj.state.s, 1, 'last');
                idx = max(1, min(idx, nPts));
                
                % Current track properties
                curKappa = curvature(idx);
                curMu = mu(idx);
                curHeading = heading(idx);
                
                % --- DRIVER MODEL: Compute throttle and brake ---
                [throttle, brake] = obj.driverModel( ...
                    obj.state.speed, curKappa, curMu, ...
                    obj.state.s, arcLen, curvature, mu);
                
                % --- FORCE COMPUTATION ---
                v = obj.state.speed;
                
                % Aerodynamic forces (pass full vehicle state for pitch/height awareness)
                F_downforce = obj.aero.computeDownforce(obj.state);
                F_drag = obj.aero.computeDrag(obj.state);
                aeroBalance = obj.aero.computeAeroBalance(obj.state);
                
                % Total weight and aero-adjusted loads
                W = obj.totalMass * 9.81;
                
                % Static weight distribution
                frontWeightFrac = obj.suspension.getStaticWeightDistribution();
                
                % Longitudinal load transfer from current ax (estimate)
                longTransfer = obj.suspension.computeLongLoadTransfer( ...
                    obj.state.ax, obj.totalMass);
                
                % Front and rear total downforce
                F_downforceFront = F_downforce * aeroBalance;
                F_downforceRear  = F_downforce * (1 - aeroBalance);
                
                % Normal loads (front axle and rear axle totals)
                Fz_front = W * frontWeightFrac + F_downforceFront + longTransfer.front;
                Fz_rear  = W * (1 - frontWeightFrac) + F_downforceRear + longTransfer.rear;
                Fz_front = max(0, Fz_front);
                Fz_rear  = max(0, Fz_rear);
                
                % Lateral load transfer estimate for grip
                % (used to compute max cornering capability)
                
                % Drive force
                F_drive = obj.powertrain.computeDriveForce(v, throttle);
                
                % Brake force (simplified: proportional to brake input)
                maxBrakeForce = 0.7 * W + F_downforce * 0.7;  % max brake ~0.7g with aero
                F_brake = -brake * maxBrakeForce;
                
                % --- MAX CORNERING SPEED ---
                % For the current curvature, compute max lateral acceleration
                % using tire grip with current normal load
                totalNormalLoad = W + F_downforce;
                peakMu = obj.tire.getPeakFriction(totalNormalLoad / 4);  % avg tire load
                maxLateralAccel = peakMu * 9.81;
                
                % Required lateral acceleration for curvature at speed v
                % ay = v^2 * kappa
                % Max speed: v_max = sqrt(ay_max / |kappa|)
                if abs(curKappa) > 1e-6
                    vMaxCorner = sqrt(maxLateralAccel / abs(curKappa));
                else
                    vMaxCorner = obj.maxSpeed;
                end
                
                % --- LONGITUDINAL FORCE BALANCE ---
                F_net_long = F_drive + F_brake - F_drag;
                
                % Rolling resistance (small)
                F_rollResist = 0.015 * (W + F_downforce);
                F_net_long = F_net_long - sign(v) * F_rollResist;
                
                % Longitudinal acceleration
                ax = F_net_long / obj.totalMass;
                
                % Limit ax to physical limits (tire saturation)
                maxAx = peakMu * 9.81;
                ax = max(-maxAx, min(ax, maxAx));
                
                % --- SPEED LIMITER ---
                % If speed exceeds cornering capability, clamp
                if v > vMaxCorner && abs(curKappa) > 1e-6
                    % Need to slow down - apply additional braking
                    excessSpeed = v - vMaxCorner;
                    brakingAccel = -min(excessSpeed / obj.dt * 0.5, maxAx);
                    ax = min(ax, brakingAccel);
                end
                
                % Hard speed limit
                if v >= obj.maxSpeed
                    ax = min(ax, 0);
                end
                
                % --- LATERAL DYNAMICS ---
                if abs(curKappa) > 1e-6 && v > 0.5
                    ay = v^2 * curKappa;
                else
                    ay = 0;
                end
                
                % Limit lateral acceleration
                ay = max(-maxLateralAccel, min(ay, maxLateralAccel));
                
                % --- INTEGRATE STATE ---
                ds = max(0, v * obj.dt + 0.5 * ax * obj.dt^2);
                
                obj.state.throttle = throttle;
                obj.state.brake = brake;
                obj.state = obj.state.updateFromDynamics( ...
                    ax, ay, ds, obj.dt, curKappa, curHeading, curMu);
                
                % --- LOG TELEMETRY ---
                if step <= maxSteps
                    stateLog.time(step)       = obj.state.time;
                    stateLog.s(step)          = obj.state.s;
                    stateLog.speed(step)      = obj.state.speed;
                    stateLog.speedKmh(step)   = obj.state.speed * 3.6;
                    stateLog.ax(step)         = ax;
                    stateLog.ay(step)         = ay;
                    stateLog.throttle(step)   = throttle;
                    stateLog.brake(step)      = brake;
                    stateLog.curvature(step)  = curKappa;
                    stateLog.heading(step)    = curHeading;
                    stateLog.F_downforce(step) = F_downforce;
                    stateLog.F_drag(step)     = F_drag;
                    stateLog.F_drive(step)    = F_drive;
                    stateLog.pitchAngle(step) = obj.state.pitchAngle;
                end
                
                % Progress display
                if mod(step, 5000) == 0
                    progress = obj.state.s / trackLen * 100;
                    fprintf('  Progress: %5.1f%% | Speed: %5.1f km/h | s: %6.1f m\n', ...
                        progress, obj.state.speed * 3.6, obj.state.s);
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
            
            lapTime = obj.state.time;
            
            fprintf('\n=== Simulation Complete ===\n');
            fprintf('Lap Time:   %.3f s\n', lapTime);
            fprintf('Track Length: %.1f m\n', obj.state.s);
            fprintf('Max Speed:  %.1f km/h\n', max(stateLog.speedKmh));
            fprintf('Steps:      %d\n', step);
        end
    end
    
    methods (Access = private)
        function [throttle, brake] = driverModel(obj, speed, curKappa, curMu, s, arcLen, curvature, mu)
            % DRIVERMODEL Simple driver model for throttle/brake decisions
            % Uses look-ahead to determine if braking is needed
            
            throttle = 0;
            brake = 0;
            
            % Compute max lateral accel from tire grip
            % Use a temporary state for aero estimation (pitch from previous step)
            tempState = VehicleState('speed', speed, 'ax', 0, 'pitchAngle', 0, 'rideHeight', 0);
            W = obj.totalMass * 9.81;
            F_downforce = obj.aero.computeDownforce(tempState);
            totalNormalLoad = W + F_downforce;
            peakMu = obj.tire.getPeakFriction(totalNormalLoad / 4);
            maxLateralAccel = peakMu * 9.81;
            
            % Current max cornering speed
            if abs(curKappa) > 1e-6
                vMaxCurrent = sqrt(maxLateralAccel / abs(curKappa));
            else
                vMaxCurrent = obj.maxSpeed;
            end
            
            % Look ahead for upcoming curvature
            lookAheadDist = speed * 2.0 * obj.brakingLookahead;  % 2 seconds ahead
            lookAheadDist = max(lookAheadDist, 10);  % at least 10m lookahead
            
            % Find look-ahead region
            idx = find(arcLen <= s, 1, 'last');
            idxEnd = find(arcLen <= s + lookAheadDist, 1, 'last');
            if isempty(idxEnd)
                idxEnd = numel(curvature);
            end
            
            % Compute max speed in look-ahead window
            upcomingKappa = curvature(idx:min(idxEnd, numel(curvature)));
            if ~isempty(upcomingKappa)
                maxUpcomingKappa = max(abs(upcomingKappa));
                if maxUpcomingKappa > 1e-6
                    vMaxAhead = sqrt(maxLateralAccel / maxUpcomingKappa);
                else
                    vMaxAhead = obj.maxSpeed;
                end
            else
                vMaxAhead = obj.maxSpeed;
            end
            
            % Decision logic
            vTarget = min(vMaxCurrent, min(vMaxAhead, obj.maxSpeed));
            
            % Compute required deceleration to reach vTarget
            if speed > vTarget * 1.02  % 2% hysteresis
                % Need to brake
                % Required decel = (v^2 - vTarget^2) / (2 * lookAheadDist)
                if lookAheadDist > 0
                    reqDecel = (speed^2 - vTarget^2) / (2 * lookAheadDist);
                else
                    reqDecel = maxLateralAccel;
                end
                reqDecel = max(0, reqDecel);
                
                % Brake intensity [0-1]
                brake = reqDecel / (maxLateralAccel);
                brake = max(0, min(1, brake));
                throttle = 0;
            elseif speed < vTarget * 0.98
                % Can accelerate
                throttle = 1.0;
                brake = 0;
            else
                % Maintain speed - partial throttle to overcome drag
                throttle = 0.5;
                brake = 0;
            end
        end
    end
end