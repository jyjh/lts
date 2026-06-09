classdef Simulator
    % SIMULATOR Physics engine and simulation loop for vehicle dynamics
    %
    % Core concept: given a VehicleState and driver inputs, progress the
    % state from one timestep to the next (copy-in → copy-out).
    %
    % Two modes of use:
    %   1. Single step:  [newState, forces] = sim.step(state, throttle, brake, curKappa, curMu, curHeading, steer)
    %   2. Full lap:     [stateLog, lapTime] = sim.simulate(initialState, track)
    %
    % The Simulator composes a VehicleManager (physics components) and a
    % DriverModel (throttle/brake decisions).
    
    properties
        % Reference to VehicleManager (components + vehicle parameters)
        vehicleManager
        
        % Reference to DriverModel (computes throttle/brake inputs)
        driverModel
        
        % Simulation timestep [s]
        dt = 0.001

        % Maximum braking slip ratio before brake input is reduced.
        % Prevents wheel-speed integration from numerically locking a tire.
        maxBrakeSlipRatio = 0.15
        
        % Internal: track whether maxSpeed warning was issued (warn once)
        warnedMaxSpeed = false
    end
    
    methods
        function obj = Simulator(vehicleManager, driverModel, dt)
            % SIMULATOR Construct with a VehicleManager and DriverModel
            %   Simulator(vehicleManager, driverModel, dt)
            %   Simulator(vehicleManager, driverModel)  % uses default dt = 0.001
            
            obj.vehicleManager = vehicleManager;
            obj.driverModel = driverModel;
            if nargin >= 3
                obj.dt = dt;
            end
        end
        
        function [newState, forces] = step(obj, state, throttle, brake, curKappa, curMu, curHeading, steer)
            % STEP Progress vehicle state by one timestep
            %   [newState, forces] = step(state, throttle, brake, curKappa, curMu, curHeading, steer)
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
            %     steer       - steering input [rad]
            %
            %   Outputs:
            %     newState    - VehicleState at next timestep
            %     forces      - struct with F_downforce, F_drag, F_drive
            
            vm = obj.vehicleManager;
            v = state.speed;
            if nargin < 8
                steer = state.steer;
            end
            
            % Copy state (will be mutated by updateFromDynamics)
            newState = state;
            
            % --- AERODYNAMIC FORCES ---
            aeroForces = vm.aero.computeForces(state);
            F_downforce = aeroForces.Fz_front + aeroForces.Fz_rear;
            F_drag = aeroForces.F_drag;
            
            % --- WEIGHT AND PER-CORNER LOADS ---
            W = vm.totalMass * 9.81;
            
            cornerLoads = vm.suspension.computeCornerLoads( ...
                state, aeroForces.Fz_front, aeroForces.Fz_rear, vm.totalMass, obj.dt);
            
            Fz_front = cornerLoads.FL + cornerLoads.FR;
            Fz_rear  = cornerLoads.RL + cornerLoads.RR;
            Fz_front = max(0, Fz_front);
            Fz_rear  = max(0, Fz_rear);
            
            % --- POWERTRAIN STATE & DRIVE FORCE ---
            vm.powertrain.updateStateFromDrivenWheels( ...
                [vm.tire.RL.angularVelocity, vm.tire.RR.angularVelocity]);
            F_drive = vm.powertrain.computeDriveForce(v, throttle);

            % --- COMBINED TIRE GRIP LIMIT ---
            totalNormalLoad = W + F_downforce;
            peakMu = vm.tire.getPeakFriction(totalNormalLoad / 4);
            effectiveMu = min(max(peakMu, 0), max(curMu, 0));
            maxTireAccel = max(0.1, effectiveMu * totalNormalLoad / vm.totalMass);

            % --- LATERAL DYNAMICS ---
            if abs(curKappa) > 1e-6 && v > 0.5
                ayDemand = v^2 * curKappa;
            else
                ayDemand = 0;
            end

            ay = max(-maxTireAccel, min(ayDemand, maxTireAccel));
            if abs(ayDemand) > maxTireAccel && abs(curKappa) > 1e-6
                ay = sign(ayDemand) * maxTireAccel * 0.995;
            end

            lateralUsage = min(abs(ay) / maxTireAccel, 1);
            availableLongAccel = maxTireAccel * sqrt(max(0, 1 - lateralUsage^2));

            % --- WHEEL TORQUE SETUP ---
            % RWD assumption: drive torque only on rear wheels.
            % Brake distribution: fixed front/rear bias from VehicleManager.
            % Drive force is split equally between the two driven wheels.
            R = vm.tire.RL.wheelRadius;  % all corners share same radius
            T_drive_front = 0;
            T_drive_rear  = F_drive * R / 2;

            % --- BRAKE FORCE ---
            brakeCommand = max(0, min(1, brake));
            brakeBiasFront = max(0, min(1, vm.brakeBiasFront));
            brakeBiasRear = 1 - brakeBiasFront;
            brakeForceCapacity = max(0, vm.brakeForceCoefficient) * totalNormalLoad;

            longGripScale = sqrt(max(0, 1 - lateralUsage^2));
            mu_FL = min(max(vm.tire.getPeakFriction(cornerLoads.FL), 0), max(curMu, 0));
            mu_FR = min(max(vm.tire.getPeakFriction(cornerLoads.FR), 0), max(curMu, 0));
            mu_RL = min(max(vm.tire.getPeakFriction(cornerLoads.RL), 0), max(curMu, 0));
            mu_RR = min(max(vm.tire.getPeakFriction(cornerLoads.RR), 0), max(curMu, 0));
            longGrip_FL = mu_FL * max(cornerLoads.FL, 0) * longGripScale;
            longGrip_FR = mu_FR * max(cornerLoads.FR, 0) * longGripScale;
            longGrip_RL = mu_RL * max(cornerLoads.RL, 0) * longGripScale;
            longGrip_RR = mu_RR * max(cornerLoads.RR, 0) * longGripScale;

            brakeGripLimit = inf;
            if brakeBiasFront > eps
                brakeGripLimit = min(brakeGripLimit, 2 * longGrip_FL / brakeBiasFront);
                brakeGripLimit = min(brakeGripLimit, 2 * longGrip_FR / brakeBiasFront);
            end
            if brakeBiasRear > eps
                brakeGripLimit = min(brakeGripLimit, 2 * longGrip_RL / brakeBiasRear);
                brakeGripLimit = min(brakeGripLimit, 2 * longGrip_RR / brakeBiasRear);
            end

            wheelLockLimit = obj.computeWheelLockBrakeLimit( ...
                vm, v, R, T_drive_front, T_drive_rear, ...
                brakeBiasFront, brakeBiasRear);

            maxBrakeForce = min([brakeForceCapacity, brakeGripLimit, wheelLockLimit]);
            brakeForceMag = min(brakeCommand * brakeForceCapacity, maxBrakeForce);
            F_brake_front = brakeForceMag * brakeBiasFront;
            F_brake_rear = brakeForceMag * brakeBiasRear;
            F_brake = -brakeForceMag;
            effectiveBrakeCommand = 0;
            if brakeForceCapacity > eps
                effectiveBrakeCommand = brakeForceMag / brakeForceCapacity;
            end
            
            % --- WHEEL DYNAMICS & SLIP RATIO ---
            % Compute per-corner torques and update wheel angular velocities,
            % then evaluate tire forces with computed slip ratios.
            % Per-corner brake torque by axle bias
            T_brake_front = F_brake_front * R / 2;
            T_brake_rear = F_brake_rear * R / 2;

            % Update wheel rotational state (uses previous-timestep Fx)
            vm.tire.updateWheelDynamics(vm.tire.FL, T_drive_front, T_brake_front, obj.dt);
            vm.tire.updateWheelDynamics(vm.tire.FR, T_drive_front, T_brake_front, obj.dt);
            vm.tire.updateWheelDynamics(vm.tire.RL, T_drive_rear,  T_brake_rear, obj.dt);
            vm.tire.updateWheelDynamics(vm.tire.RR, T_drive_rear,  T_brake_rear, obj.dt);

            limitedRearOmega = vm.powertrain.limitDrivenWheelAngularVelocity( ...
                [vm.tire.RL.angularVelocity, vm.tire.RR.angularVelocity]);
            vm.tire.RL.angularVelocity = limitedRearOmega(1);
            vm.tire.RR.angularVelocity = limitedRearOmega(2);

            vm.powertrain.updateStateFromDrivenWheels( ...
                [vm.tire.RL.angularVelocity, vm.tire.RR.angularVelocity]);

            % Evaluate tire forces with computed per-corner slip ratios
            tireInputState = state;
            tireInputState.steer = steer;
            tireInputState.curvature = curKappa;
            tireInputState.mu = curMu;
            vm.tire.updateAllFromState(tireInputState, vm, cornerLoads, curMu);
            % --- LONGITUDINAL FORCE BALANCE ---
            F_net_long = F_drive + F_brake - F_drag;
            F_rollResist = 0.015 * (W + F_downforce);
            F_net_long = F_net_long - sign(v) * F_rollResist;
            
            axDemand = F_net_long / vm.totalMass;
            ax = max(-availableLongAccel, min(axDemand, availableLongAccel));

            if abs(ayDemand) > maxTireAccel && abs(curKappa) > 1e-6
                vMaxCorner = sqrt(maxTireAccel / abs(curKappa));
                excessSpeed = max(v - vMaxCorner, 0);
                correctiveBrakeAccel = min(excessSpeed / max(obj.dt, eps) * 0.5, availableLongAccel);
                ax = min(ax, -correctiveBrakeAccel);
            end
            
            % --- INTEGRATE STATE ---
            ds = max(0, v * obj.dt + 0.5 * ax * obj.dt^2);
            
            newState.throttle = throttle;
            newState.brake = effectiveBrakeCommand;
            newState.steer = steer;
            newState = newState.updateFromDynamics(ax, ay, ds, obj.dt, curKappa, curHeading, curMu);
            
            % Sanity check: warn once if speed exceeds maxSpeed
            if newState.speed > vm.maxSpeed && ~obj.warnedMaxSpeed
                obj.warnedMaxSpeed = true;
                warning('Simulator:SpeedExceeded', ...
                    'Speed (%.1f m/s / %.1f km/h) exceeded maxSpeed (%.1f m/s). Check simulation.', ...
                    newState.speed, newState.speed * 3.6, vm.maxSpeed);
            end
            
            % --- RETURN FORCES ---
            forces.F_downforce = F_downforce;
            forces.F_drag = F_drag;
            forces.F_drive = F_drive;
            forces.F_brake = F_brake;
            forces.F_brake_front = -F_brake_front;
            forces.F_brake_rear = -F_brake_rear;
            forces.F_brake_FL = -F_brake_front / 2;
            forces.F_brake_FR = -F_brake_front / 2;
            forces.F_brake_RL = -F_brake_rear / 2;
            forces.F_brake_RR = -F_brake_rear / 2;
            forces.brakeCommand = brakeCommand;
            forces.brake = effectiveBrakeCommand;
            forces.brakeLimit = maxBrakeForce;
            forces.brakeGripLimit = brakeGripLimit;
            forces.brakeWheelLockLimit = wheelLockLimit;
            forces.brakeForceCapacity = brakeForceCapacity;
            forces.brakeGrip_FL = longGrip_FL;
            forces.brakeGrip_FR = longGrip_FR;
            forces.brakeGrip_RL = longGrip_RL;
            forces.brakeGrip_RR = longGrip_RR;
            forces.motorRPM = 0;
            forces.motorTorque = 0;
            forces.wheelTorque = 0;
            forces.drivenWheelRPM = 0;
            forces.rpmLimitActive = false;
            if ~isempty(vm.powertrain.state)
                forces.motorRPM = vm.powertrain.state.motorRPM;
                forces.motorTorque = vm.powertrain.state.motorTorque;
                forces.wheelTorque = vm.powertrain.state.wheelTorque;
                forces.drivenWheelRPM = vm.powertrain.state.drivenWheelRPM;
                forces.rpmLimitActive = vm.powertrain.state.rpmLimitActive;
            end
            forces.aeroFz_front = aeroForces.Fz_front;
            forces.aeroFz_rear  = aeroForces.Fz_rear;
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
            maxSteps = round(trackLen / (initialState.speed * obj.dt) * 5);
            maxSteps = max(maxSteps, 100000);
            stateLog = struct( ...
                'time',        zeros(maxSteps, 1), ...
                's',           zeros(maxSteps, 1), ...
                'controlS',    zeros(maxSteps, 1), ...
                'speed',       zeros(maxSteps, 1), ...
                'speedKmh',    zeros(maxSteps, 1), ...
                'controlTime', zeros(maxSteps, 1), ...
                'ax',          zeros(maxSteps, 1), ...
                'ay',          zeros(maxSteps, 1), ...
                'throttle',    zeros(maxSteps, 1), ...
                'brake',       zeros(maxSteps, 1), ...
                'brakeRequested', zeros(maxSteps, 1), ...
                'steer',       zeros(maxSteps, 1), ...
                'curvature',   zeros(maxSteps, 1), ...
                'heading',     zeros(maxSteps, 1), ...
                'F_downforce', zeros(maxSteps, 1), ...
                'F_drag',      zeros(maxSteps, 1), ...
                'F_drive',     zeros(maxSteps, 1), ...
                'F_brake',     zeros(maxSteps, 1), ...
                'F_brake_front', zeros(maxSteps, 1), ...
                'F_brake_rear', zeros(maxSteps, 1), ...
                'F_brake_FL',  zeros(maxSteps, 1), ...
                'F_brake_FR',  zeros(maxSteps, 1), ...
                'F_brake_RL',  zeros(maxSteps, 1), ...
                'F_brake_RR',  zeros(maxSteps, 1), ...
                'brakeGrip_FL', zeros(maxSteps, 1), ...
                'brakeGrip_FR', zeros(maxSteps, 1), ...
                'brakeGrip_RL', zeros(maxSteps, 1), ...
                'brakeGrip_RR', zeros(maxSteps, 1), ...
                'motorRPM',    zeros(maxSteps, 1), ...
                'motorTorque', zeros(maxSteps, 1), ...
                'wheelTorque', zeros(maxSteps, 1), ...
                'drivenWheelRPM', zeros(maxSteps, 1), ...
                'rpmLimitActive', false(maxSteps, 1), ...
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
                'damperVel_RR', zeros(maxSteps, 1), ...
                'slipRatio_FL', zeros(maxSteps, 1), ...
                'slipRatio_FR', zeros(maxSteps, 1), ...
                'slipRatio_RL', zeros(maxSteps, 1), ...
                'slipRatio_RR', zeros(maxSteps, 1), ...
                'omega_FL',     zeros(maxSteps, 1), ...
                'omega_FR',     zeros(maxSteps, 1), ...
                'omega_RL',     zeros(maxSteps, 1), ...
                'omega_RR',     zeros(maxSteps, 1), ...
                'tireFx_FL',    zeros(maxSteps, 1), ...
                'tireFx_FR',    zeros(maxSteps, 1), ...
                'tireFx_RL',    zeros(maxSteps, 1), ...
                'tireFx_RR',    zeros(maxSteps, 1), ...
                'tireFy_FL',    zeros(maxSteps, 1), ...
                'tireFy_FR',    zeros(maxSteps, 1), ...
                'tireFy_RL',    zeros(maxSteps, 1), ...
                'tireFy_RR',    zeros(maxSteps, 1), ...
                'aeroFz_front', zeros(maxSteps, 1), ...
                'aeroFz_rear',  zeros(maxSteps, 1) ...
            );
            
            % Working state (will be updated each step)
            currentState = initialState;
            obj.initializeWheelSpeeds(currentState.speed);
            
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
                [throttle, brake, steer] = obj.driverModel.computeInputs(currentState);
                
                % --- PHYSICS STEP ---
                [newState, forces] = obj.step( ...
                    currentState, throttle, brake, curKappa, curMu, curHeading, steer);
                
                % --- LOG TELEMETRY ---
                if step <= maxSteps
                    stateLog.time(step)        = newState.time;
                    stateLog.s(step)           = newState.s;
                    stateLog.controlS(step)    = currentState.s;
                    stateLog.speed(step)       = newState.speed;
                    stateLog.speedKmh(step)    = newState.speed * 3.6;
                    stateLog.controlTime(step) = currentState.time;
                    stateLog.ax(step)          = newState.ax;
                    stateLog.ay(step)          = newState.ay;
                    stateLog.throttle(step)    = throttle;
                    stateLog.brake(step)       = forces.brake;
                    stateLog.brakeRequested(step) = forces.brakeCommand;
                    stateLog.steer(step)       = steer;
                    stateLog.curvature(step)   = curKappa;
                    stateLog.heading(step)     = curHeading;
                    stateLog.F_downforce(step) = forces.F_downforce;
                    stateLog.F_drag(step)      = forces.F_drag;
                    stateLog.F_drive(step)     = forces.F_drive;
                    stateLog.F_brake(step)     = forces.F_brake;
                    stateLog.F_brake_front(step) = forces.F_brake_front;
                    stateLog.F_brake_rear(step) = forces.F_brake_rear;
                    stateLog.F_brake_FL(step)  = forces.F_brake_FL;
                    stateLog.F_brake_FR(step)  = forces.F_brake_FR;
                    stateLog.F_brake_RL(step)  = forces.F_brake_RL;
                    stateLog.F_brake_RR(step)  = forces.F_brake_RR;
                    stateLog.brakeGrip_FL(step) = forces.brakeGrip_FL;
                    stateLog.brakeGrip_FR(step) = forces.brakeGrip_FR;
                    stateLog.brakeGrip_RL(step) = forces.brakeGrip_RL;
                    stateLog.brakeGrip_RR(step) = forces.brakeGrip_RR;
                    stateLog.motorRPM(step)    = forces.motorRPM;
                    stateLog.motorTorque(step) = forces.motorTorque;
                    stateLog.wheelTorque(step) = forces.wheelTorque;
                    stateLog.drivenWheelRPM(step) = forces.drivenWheelRPM;
                    stateLog.rpmLimitActive(step) = forces.rpmLimitActive;
                    stateLog.pitchAngle(step)  = newState.pitchAngle;
                    stateLog.aeroFz_front(step) = forces.aeroFz_front;
                    stateLog.aeroFz_rear(step)  = forces.aeroFz_rear;
                    
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
                    
                    % Per-corner tire telemetry (slip, wheel speed, forces)
                    stateLog.slipRatio_FL(step) = vm.tire.FL.slipRatio;
                    stateLog.slipRatio_FR(step) = vm.tire.FR.slipRatio;
                    stateLog.slipRatio_RL(step) = vm.tire.RL.slipRatio;
                    stateLog.slipRatio_RR(step) = vm.tire.RR.slipRatio;
                    stateLog.omega_FL(step)     = vm.tire.FL.angularVelocity;
                    stateLog.omega_FR(step)     = vm.tire.FR.angularVelocity;
                    stateLog.omega_RL(step)     = vm.tire.RL.angularVelocity;
                    stateLog.omega_RR(step)     = vm.tire.RR.angularVelocity;
                    stateLog.tireFx_FL(step)    = vm.tire.FL.Fx;
                    stateLog.tireFx_FR(step)    = vm.tire.FR.Fx;
                    stateLog.tireFx_RL(step)    = vm.tire.RL.Fx;
                    stateLog.tireFx_RR(step)    = vm.tire.RR.Fx;
                    stateLog.tireFy_FL(step)    = vm.tire.FL.Fy;
                    stateLog.tireFy_FR(step)    = vm.tire.FR.Fy;
                    stateLog.tireFy_RL(step)    = vm.tire.RL.Fy;
                    stateLog.tireFy_RR(step)    = vm.tire.RR.Fy;
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

        function brakeLimit = computeWheelLockBrakeLimit(obj, vm, vehicleSpeed, ...
                wheelRadius, driveTorqueFront, driveTorqueRear, ...
                brakeBiasFront, brakeBiasRear)
            % Compute the fixed-bias brake force limit that prevents any
            % corner from crossing the configured braking slip in one step.
            if vehicleSpeed < 0.5
                brakeLimit = inf;
                return;
            end

            cap_FL = obj.computeCornerLockBrakeForce(vm.tire.FL, vehicleSpeed, ...
                driveTorqueFront, wheelRadius);
            cap_FR = obj.computeCornerLockBrakeForce(vm.tire.FR, vehicleSpeed, ...
                driveTorqueFront, wheelRadius);
            cap_RL = obj.computeCornerLockBrakeForce(vm.tire.RL, vehicleSpeed, ...
                driveTorqueRear, wheelRadius);
            cap_RR = obj.computeCornerLockBrakeForce(vm.tire.RR, vehicleSpeed, ...
                driveTorqueRear, wheelRadius);

            brakeLimit = inf;
            if brakeBiasFront > eps
                brakeLimit = min(brakeLimit, 2 * cap_FL / brakeBiasFront);
                brakeLimit = min(brakeLimit, 2 * cap_FR / brakeBiasFront);
            end
            if brakeBiasRear > eps
                brakeLimit = min(brakeLimit, 2 * cap_RL / brakeBiasRear);
                brakeLimit = min(brakeLimit, 2 * cap_RR / brakeBiasRear);
            end
        end

        function brakeForceCap = computeCornerLockBrakeForce(obj, cornerState, ...
                vehicleSpeed, driveTorque, fallbackRadius)
            wheelRadius = cornerState.wheelRadius;
            if wheelRadius <= 0 || ~isfinite(wheelRadius)
                wheelRadius = fallbackRadius;
            end
            wheelRadius = max(wheelRadius, eps);

            tire = obj.vehicleManager.tire;
            wheelInertia = 0.5;
            if isprop(tire, 'wheelInertia')
                wheelInertia = tire.wheelInertia;
            end

            maxSlip = max(0, min(0.95, obj.maxBrakeSlipRatio));
            minOmega = (1 - maxSlip) * max(vehicleSpeed, 0) / wheelRadius;
            omega = max(cornerState.angularVelocity, 0);

            % I*domega/dt = T_drive - T_brake - Fx*R. Solve for the largest
            % brake torque that still keeps omega above the lock threshold.
            tireReactionTorque = cornerState.Fx * wheelRadius;
            allowableBrakeTorque = driveTorque - tireReactionTorque + ...
                (omega - minOmega) * wheelInertia / max(obj.dt, eps);

            brakeForceCap = max(0, allowableBrakeTorque / wheelRadius);
        end

        function initializeWheelSpeeds(obj, vehicleSpeed)
            if vehicleSpeed <= 0
                return;
            end

            tire = obj.vehicleManager.tire;
            obj.initializeCornerWheelSpeed(tire.FL, vehicleSpeed);
            obj.initializeCornerWheelSpeed(tire.FR, vehicleSpeed);
            obj.initializeCornerWheelSpeed(tire.RL, vehicleSpeed);
            obj.initializeCornerWheelSpeed(tire.RR, vehicleSpeed);
            obj.vehicleManager.powertrain.updateStateFromDrivenWheels( ...
                [tire.RL.angularVelocity, tire.RR.angularVelocity]);
        end

        function initializeCornerWheelSpeed(~, cornerState, vehicleSpeed)
            if cornerState.angularVelocity > 0
                return;
            end

            cornerState.angularVelocity = max(vehicleSpeed, 0) / ...
                max(cornerState.wheelRadius, eps);
        end
    end
end
