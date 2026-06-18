classdef Simulator
    % SIMULATOR Physics engine and simulation loop for vehicle dynamics
    %
    % Core concept: given a VehicleState and driver inputs, progress the
    % state from one timestep to the next (copy-in → copy-out).
    %
    % Two modes of use:
    %   1. Single step:  [newState, forces] = sim.step(state, input, ref)
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

        % Reserved for a future ABS-style brake controller.
        % Open-loop braking currently applies commanded brake torque directly.
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
        
        function [newState, forces] = step(obj, state, input, ref)
            % STEP Progress vehicle state by one timestep
            %   [newState, forces] = step(state, input, ref)
            %
            % Driver inputs are open-loop controls sampled from the reference
            % profile. Track curvature is reference telemetry only; planar
            % motion comes from summed tire forces and yaw moment.
            
            vm = obj.vehicleManager;
            throttle = max(0, min(1, input.throttle));
            brake = max(0, min(1, input.brake));
            steer = input.steer;
            curMu = ref.mu;

            if isnan(state.vx)
                state.vx = state.speed;
            end
            if isnan(state.yaw)
                state.yaw = ref.heading;
                state.heading = ref.heading;
            end
            if isnan(state.x)
                state.x = ref.x;
                state.y = ref.y;
            end
            
            % Copy state (will be mutated by updateFromDynamics)
            newState = state;
            
            % --- AERODYNAMIC FORCES ---
            aeroForces = vm.aero.computeForces(state);
            F_downforce = aeroForces.Fz_front + aeroForces.Fz_rear;
            F_drag = aeroForces.F_drag;
            
            % --- WEIGHT AND PER-CORNER LOADS ---
            W = vm.totalMass * 9.81;
            
            suspensionInputState = state;
            suspensionInputState.steer = steer;
            cornerLoads = vm.suspension.computeCornerLoads( ...
                suspensionInputState, aeroForces.Fz_front, aeroForces.Fz_rear, vm.totalMass, obj.dt);
            
            Fz_front = cornerLoads.FL + cornerLoads.FR;
            Fz_rear  = cornerLoads.RL + cornerLoads.RR;
            Fz_front = max(0, Fz_front);
            Fz_rear  = max(0, Fz_rear);
            
            % --- POWERTRAIN STATE & DRIVE TORQUE ---
            vm.powertrain.updateStateFromDrivenWheels( ...
                [vm.tire.RL.angularVelocity, vm.tire.RR.angularVelocity]);
            totalDriveTorque = vm.powertrain.computeDriveTorque(state.speed, throttle);

            % --- WHEEL TORQUE SETUP ---
            % RWD assumption: drive torque only on rear wheels.
            % Brake distribution: fixed front/rear bias from VehicleManager.
            R = vm.tire.RL.wheelRadius;  % all corners share same radius
            T_drive_front = 0;
            T_drive_rear  = totalDriveTorque / 2;

            % --- BRAKE TORQUE ---
            brakeCommand = max(0, min(1, brake));
            brakeBiasFront = max(0, min(1, vm.brakeBiasFront));
            brakeBiasRear = 1 - brakeBiasFront;
            % Existing brakeForceCoefficient is preserved as an equivalent
            % total brake force capacity, then converted to wheel torque.
            totalNormalLoad = W + F_downforce;
            brakeForceCapacity = max(0, vm.brakeForceCoefficient) * totalNormalLoad;
            wheelLockLimit = inf;
            maxBrakeForce = brakeForceCapacity;
            brakeForceMag = brakeCommand * brakeForceCapacity;
            F_brake_front_cmd = brakeForceMag * brakeBiasFront;
            F_brake_rear_cmd = brakeForceMag * brakeBiasRear;
            effectiveBrakeCommand = brakeCommand;
            
            % --- WHEEL DYNAMICS & SLIP RATIO ---
            % Compute per-corner torques and update wheel angular velocities,
            % then evaluate tire forces with computed slip ratios.
            % Per-corner brake torque by axle bias
            T_brake_front = F_brake_front_cmd * R / 2;
            T_brake_rear = F_brake_rear_cmd * R / 2;

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

            % First tire-force pass with previous load-transfer state.
            tireInputState = state;
            tireInputState.steer = steer;
            tireData = obj.updatePlanarTireForces(tireInputState, cornerLoads, curMu);
            dynamics = obj.computePlanarDynamics(state, tireData, F_drag, W + F_downforce);

            % One predictor/corrector pass for load transfer using current
            % force-derived body accelerations.
            correctedLoadState = state;
            correctedLoadState.steer = steer;
            correctedLoadState.ax = dynamics.ax;
            correctedLoadState.ay = dynamics.ay;
            cornerLoads = vm.suspension.computeCornerLoads( ...
                correctedLoadState, aeroForces.Fz_front, aeroForces.Fz_rear, vm.totalMass, obj.dt);
            tireData = obj.updatePlanarTireForces(tireInputState, cornerLoads, curMu);
            dynamics = obj.computePlanarDynamics(state, tireData, F_drag, W + F_downforce);

            F_tire_long = tireData.sumFxBody;
            F_drive = max(0, F_tire_long);
            F_brake = min(0, F_tire_long);
            F_rollResist = 0.015 * (W + F_downforce);

            % --- INTEGRATE STATE ---
            vx0 = state.vx;
            vy0 = state.vy;
            yaw0 = state.yaw;
            yawRate0 = state.yawRate;

            vxNew = vx0 + (dynamics.ax + yawRate0 * vy0) * obj.dt;
            vyNew = vy0 + (dynamics.ay - yawRate0 * vx0) * obj.dt;
            yawRateNew = yawRate0 + dynamics.yawAccel * obj.dt;
            yawNew = yaw0 + yawRateNew * obj.dt;

            vxWorld = vxNew * cos(yawNew) - vyNew * sin(yawNew);
            vyWorld = vxNew * sin(yawNew) + vyNew * cos(yawNew);
            xNew = state.x + vxWorld * obj.dt;
            yNew = state.y + vyWorld * obj.dt;

            nextRef = obj.projectToReference(xNew, yNew, ref.trackData, ref.idx);
            
            newState.throttle = throttle;
            newState.brake = effectiveBrakeCommand;
            newState.steer = steer;
            newState = newState.updateFromPlanarDynamics( ...
                dynamics.ax, dynamics.ay, dynamics.yawAccel, ...
                vxNew, vyNew, yawRateNew, yawNew, xNew, yNew, ...
                nextRef.s, nextRef.heading, nextRef.curvature, ...
                nextRef.lateralError, obj.dt, nextRef.mu);
            
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
            forces.F_tire_long = F_tire_long;
            forces.F_brake_front = min(0, vm.tire.FL.Fx + vm.tire.FR.Fx);
            forces.F_brake_rear = min(0, vm.tire.RL.Fx + vm.tire.RR.Fx);
            forces.F_brake_FL = min(0, vm.tire.FL.Fx);
            forces.F_brake_FR = min(0, vm.tire.FR.Fx);
            forces.F_brake_RL = min(0, vm.tire.RL.Fx);
            forces.F_brake_RR = min(0, vm.tire.RR.Fx);
            forces.brakeCommand = brakeCommand;
            forces.brake = effectiveBrakeCommand;
            forces.brakeLimit = maxBrakeForce;
            forces.brakeGripLimit = inf;
            forces.brakeWheelLockLimit = wheelLockLimit;
            forces.brakeForceCapacity = brakeForceCapacity;
            forces.brakeGrip_FL = max(vm.tire.FL.peakMu, 0) * max(cornerLoads.FL, 0);
            forces.brakeGrip_FR = max(vm.tire.FR.peakMu, 0) * max(cornerLoads.FR, 0);
            forces.brakeGrip_RL = max(vm.tire.RL.peakMu, 0) * max(cornerLoads.RL, 0);
            forces.brakeGrip_RR = max(vm.tire.RR.peakMu, 0) * max(cornerLoads.RR, 0);
            forces.driveTorqueTotal = totalDriveTorque;
            forces.driveTorque_RL = T_drive_rear;
            forces.driveTorque_RR = T_drive_rear;
            forces.brakeTorque_FL = T_brake_front;
            forces.brakeTorque_FR = T_brake_front;
            forces.brakeTorque_RL = T_brake_rear;
            forces.brakeTorque_RR = T_brake_rear;
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
            forces.F_tire_lat = tireData.sumFyBody;
            forces.yawMoment = tireData.yawMoment;
            forces.yawAccel = dynamics.yawAccel;
            forces.rollResistance = F_rollResist;
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
            trackData = struct( ...
                'points', trackPts, ...
                'arcLen', arcLen, ...
                'curvature', curvature(:), ...
                'mu', mu(:), ...
                'heading', heading(:), ...
                'length', trackLen, ...
                'nPts', nPts);
            initialState = obj.initializePlanarState(initialState, trackData);
            inputProfile = obj.buildOpenLoopInputProfile(initialState, trackData);
            
            % Pre-allocate telemetry log
            maxSteps = round(trackLen / (max(initialState.speed, 5) * obj.dt) * 5);
            maxSteps = max(maxSteps, 100000);
            stateLog = struct( ...
                'time',        zeros(maxSteps, 1), ...
                's',           zeros(maxSteps, 1), ...
                'controlS',    zeros(maxSteps, 1), ...
                'x',           zeros(maxSteps, 1), ...
                'y',           zeros(maxSteps, 1), ...
                'yaw',         zeros(maxSteps, 1), ...
                'vx',          zeros(maxSteps, 1), ...
                'vy',          zeros(maxSteps, 1), ...
                'speed',       zeros(maxSteps, 1), ...
                'speedKmh',    zeros(maxSteps, 1), ...
                'controlTime', zeros(maxSteps, 1), ...
                'ax',          zeros(maxSteps, 1), ...
                'ay',          zeros(maxSteps, 1), ...
                'yawRate',     zeros(maxSteps, 1), ...
                'yawAccel',    zeros(maxSteps, 1), ...
                'refS',        zeros(maxSteps, 1), ...
                'refHeading',  zeros(maxSteps, 1), ...
                'refCurvature', zeros(maxSteps, 1), ...
                'lateralError', zeros(maxSteps, 1), ...
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
                'F_tire_long', zeros(maxSteps, 1), ...
                'F_tire_lat',  zeros(maxSteps, 1), ...
                'yawMoment',   zeros(maxSteps, 1), ...
                'rollResistance', zeros(maxSteps, 1), ...
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
                'driveTorqueTotal', zeros(maxSteps, 1), ...
                'driveTorque_RL', zeros(maxSteps, 1), ...
                'driveTorque_RR', zeros(maxSteps, 1), ...
                'brakeTorque_FL', zeros(maxSteps, 1), ...
                'brakeTorque_FR', zeros(maxSteps, 1), ...
                'brakeTorque_RL', zeros(maxSteps, 1), ...
                'brakeTorque_RR', zeros(maxSteps, 1), ...
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
                'wheelTravel_FL', zeros(maxSteps, 1), ...
                'wheelTravel_FR', zeros(maxSteps, 1), ...
                'wheelTravel_RL', zeros(maxSteps, 1), ...
                'wheelTravel_RR', zeros(maxSteps, 1), ...
                'camber_FL',    zeros(maxSteps, 1), ...
                'camber_FR',    zeros(maxSteps, 1), ...
                'camber_RL',    zeros(maxSteps, 1), ...
                'camber_RR',    zeros(maxSteps, 1), ...
                'toe_FL',       zeros(maxSteps, 1), ...
                'toe_FR',       zeros(maxSteps, 1), ...
                'toe_RL',       zeros(maxSteps, 1), ...
                'toe_RR',       zeros(maxSteps, 1), ...
                'wheelSteer_FL', zeros(maxSteps, 1), ...
                'wheelSteer_FR', zeros(maxSteps, 1), ...
                'wheelSteer_RL', zeros(maxSteps, 1), ...
                'wheelSteer_RR', zeros(maxSteps, 1), ...
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
            currentRef = obj.projectToReference(currentState.x, currentState.y, trackData, 1);
            
            step = 0;
            fprintf('Starting simulation...\n');
            fprintf('Track length: %.1f m\n', trackLen);
            
            finishTolerance = 1e-6;
            while currentState.s < trackLen - finishTolerance && currentState.onTrack
                step = step + 1;
                
                % Project the free vehicle pose onto the reference centerline.
                currentRef = obj.projectToReference( ...
                    currentState.x, currentState.y, trackData, currentRef.idx);
                currentState.s = currentRef.s;
                currentState.refS = currentRef.s;
                currentState.refHeading = currentRef.heading;
                currentState.refCurvature = currentRef.curvature;
                currentState.curvature = currentRef.curvature;
                currentState.lateralError = currentRef.lateralError;
                currentState.mu = currentRef.mu;

                % --- OPEN-LOOP DRIVER PROFILE ---
                input = obj.sampleInputProfile(inputProfile, currentRef.idx);
                ref = currentRef;
                ref.trackData = trackData;
                
                % --- PHYSICS STEP ---
                [newState, forces] = obj.step(currentState, input, ref);
                
                % --- LOG TELEMETRY ---
                if step <= maxSteps
                    stateLog.time(step)        = newState.time;
                    stateLog.s(step)           = newState.s;
                    stateLog.controlS(step)    = currentState.s;
                    stateLog.x(step)           = newState.x;
                    stateLog.y(step)           = newState.y;
                    stateLog.yaw(step)         = newState.yaw;
                    stateLog.vx(step)          = newState.vx;
                    stateLog.vy(step)          = newState.vy;
                    stateLog.speed(step)       = newState.speed;
                    stateLog.speedKmh(step)    = newState.speed * 3.6;
                    stateLog.controlTime(step) = currentState.time;
                    stateLog.ax(step)          = newState.ax;
                    stateLog.ay(step)          = newState.ay;
                    stateLog.yawRate(step)     = newState.yawRate;
                    stateLog.yawAccel(step)    = newState.yawAccel;
                    stateLog.refS(step)        = newState.refS;
                    stateLog.refHeading(step)  = newState.refHeading;
                    stateLog.refCurvature(step) = newState.refCurvature;
                    stateLog.lateralError(step) = newState.lateralError;
                    stateLog.throttle(step)    = input.throttle;
                    stateLog.brake(step)       = forces.brake;
                    stateLog.brakeRequested(step) = forces.brakeCommand;
                    stateLog.steer(step)       = input.steer;
                    stateLog.curvature(step)   = newState.refCurvature;
                    stateLog.heading(step)     = newState.heading;
                    stateLog.F_downforce(step) = forces.F_downforce;
                    stateLog.F_drag(step)      = forces.F_drag;
                    stateLog.F_drive(step)     = forces.F_drive;
                    stateLog.F_brake(step)     = forces.F_brake;
                    stateLog.F_tire_long(step) = forces.F_tire_long;
                    stateLog.F_tire_lat(step)  = forces.F_tire_lat;
                    stateLog.yawMoment(step)   = forces.yawMoment;
                    stateLog.rollResistance(step) = forces.rollResistance;
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
                    stateLog.driveTorqueTotal(step) = forces.driveTorqueTotal;
                    stateLog.driveTorque_RL(step) = forces.driveTorque_RL;
                    stateLog.driveTorque_RR(step) = forces.driveTorque_RR;
                    stateLog.brakeTorque_FL(step) = forces.brakeTorque_FL;
                    stateLog.brakeTorque_FR(step) = forces.brakeTorque_FR;
                    stateLog.brakeTorque_RL(step) = forces.brakeTorque_RL;
                    stateLog.brakeTorque_RR(step) = forces.brakeTorque_RR;
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
                    stateLog.wheelTravel_FL(step) = susp.frontLeft.state.wheelTravel;
                    stateLog.wheelTravel_FR(step) = susp.frontRight.state.wheelTravel;
                    stateLog.wheelTravel_RL(step) = susp.rearLeft.state.wheelTravel;
                    stateLog.wheelTravel_RR(step) = susp.rearRight.state.wheelTravel;
                    stateLog.camber_FL(step)    = susp.frontLeft.state.camberAngle;
                    stateLog.camber_FR(step)    = susp.frontRight.state.camberAngle;
                    stateLog.camber_RL(step)    = susp.rearLeft.state.camberAngle;
                    stateLog.camber_RR(step)    = susp.rearRight.state.camberAngle;
                    stateLog.toe_FL(step)       = susp.frontLeft.state.toeAngle;
                    stateLog.toe_FR(step)       = susp.frontRight.state.toeAngle;
                    stateLog.toe_RL(step)       = susp.rearLeft.state.toeAngle;
                    stateLog.toe_RR(step)       = susp.rearRight.state.toeAngle;
                    stateLog.wheelSteer_FL(step) = susp.frontLeft.state.steerAngle;
                    stateLog.wheelSteer_FR(step) = susp.frontRight.state.steerAngle;
                    stateLog.wheelSteer_RL(step) = susp.rearLeft.state.steerAngle;
                    stateLog.wheelSteer_RR(step) = susp.rearRight.state.steerAngle;
                    
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

        function state = initializePlanarState(~, state, trackData)
            firstPoint = trackData.points(1, :);
            if isnan(state.x)
                state.x = firstPoint(1);
            end
            if isnan(state.y)
                state.y = firstPoint(2);
            end
            if isnan(state.yaw)
                state.yaw = trackData.heading(1);
                state.heading = state.yaw;
            end
            if isnan(state.vx)
                state.vx = max(state.speed, 0);
            end
            state.speed = hypot(state.vx, state.vy);
            state.refS = state.s;
            state.refHeading = trackData.heading(1);
            state.refCurvature = trackData.curvature(1);
            state.curvature = state.refCurvature;
            state.mu = trackData.mu(1);
        end

        function profile = buildOpenLoopInputProfile(obj, initialState, trackData)
            vm = obj.vehicleManager;
            n = trackData.nPts;
            curvature = trackData.curvature(:);
            mu = trackData.mu(:);
            vTarget = vm.maxSpeed * ones(n, 1);

            % Iterating lets the speed-dependent aero estimate influence the
            % GGV cornering envelope without solving an optimization problem.
            for iter = 1:3
                for i = 1:n
                    if abs(curvature(i)) > 1e-6
                        limits = obj.estimateGGVLimits(vTarget(i), mu(i), initialState);
                        vTarget(i) = min(vm.maxSpeed, ...
                            sqrt(max(limits.maxLatAccel, 0.1) / abs(curvature(i))));
                    else
                        vTarget(i) = vm.maxSpeed;
                    end
                end
            end

            maxBrakeAccel = zeros(n, 1);
            for i = 1:n
                limits = obj.estimateGGVLimits(vTarget(i), mu(i), initialState);
                maxBrakeAccel(i) = limits.maxBrakeAccel;
            end

            for i = n-1:-1:1
                ds = max(trackData.arcLen(i+1) - trackData.arcLen(i), 0.001);
                reachableSpeed = sqrt(vTarget(i+1)^2 + 2 * maxBrakeAccel(i+1) * ds);
                vTarget(i) = min(vTarget(i), reachableSpeed);
            end

            speedPlan = vTarget;
            speedPlan(1) = min(max(initialState.speed, 0), vTarget(1));
            maxDriveAccel = 5.0;
            for i = 1:n-1
                ds = max(trackData.arcLen(i+1) - trackData.arcLen(i), 0.001);
                reachableSpeed = sqrt(speedPlan(i)^2 + 2 * maxDriveAccel * ds);
                speedPlan(i+1) = min(vTarget(i+1), reachableSpeed);
            end

            axRef = zeros(n, 1);
            for i = 1:n-1
                ds = max(trackData.arcLen(i+1) - trackData.arcLen(i), 0.001);
                axRef(i) = (speedPlan(i+1)^2 - speedPlan(i)^2) / (2 * ds);
            end
            axRef(n) = axRef(max(n-1, 1));

            maxSteer = 0.6;
            if ~isempty(obj.driverModel) && isprop(obj.driverModel, 'maxSteeringAngle')
                maxSteer = obj.driverModel.maxSteeringAngle;
            end
            steerRef = atan(vm.wheelbase * curvature);
            steerRef = max(-maxSteer, min(maxSteer, steerRef));

            brakeRef = zeros(n, 1);
            throttleRef = ones(n, 1);
            for i = 1:n
                if axRef(i) < 0
                    brakeRef(i) = min(1, -axRef(i) / max(maxBrakeAccel(i), eps));
                    throttleRef(i) = 0;
                end
            end

            profile = struct( ...
                's', trackData.arcLen, ...
                'vTarget', speedPlan, ...
                'vLimit', vTarget, ...
                'axRef', axRef, ...
                'throttle', throttleRef, ...
                'brake', brakeRef, ...
                'steer', steerRef);
        end

        function limits = estimateGGVLimits(obj, speed, mu, templateState)
            vm = obj.vehicleManager;
            tempState = templateState;
            tempState.vehicleManager = vm;
            tempState.speed = max(speed, 0);
            tempState.vx = tempState.speed;
            tempState.vy = 0;

            aeroForces = vm.aero.computeForces(tempState);
            totalNormalLoad = vm.totalMass * 9.81 + aeroForces.Fz_front + aeroForces.Fz_rear;
            peakMu = vm.tire.getPeakFriction(totalNormalLoad / 4);
            effectiveMu = min(max(peakMu, 0), max(mu, 0));
            tireAccel = effectiveMu * totalNormalLoad / vm.totalMass;

            brakeForce = max(0, vm.brakeForceCoefficient) * totalNormalLoad;
            rollingResistance = 0.015 * totalNormalLoad;
            brakeAccel = (brakeForce + aeroForces.F_drag + rollingResistance) / vm.totalMass;

            limits.maxLatAccel = max(0.1, 0.98 * tireAccel);
            limits.maxBrakeAccel = max(0.1, 0.98 * min(tireAccel, brakeAccel));
        end

        function input = sampleInputProfile(~, profile, idx)
            idx = max(1, min(idx, numel(profile.throttle)));
            input = struct( ...
                'throttle', profile.throttle(idx), ...
                'brake', profile.brake(idx), ...
                'steer', profile.steer(idx), ...
                'targetSpeed', profile.vTarget(idx), ...
                'axRef', profile.axRef(idx));
        end

        function ref = projectToReference(~, x, y, trackData, previousIdx)
            if nargin < 5 || isempty(previousIdx) || previousIdx < 1
                previousIdx = 1;
            end

            searchStart = max(1, previousIdx);
            searchEnd = min(trackData.nPts, previousIdx + 300);
            if searchStart > searchEnd
                searchStart = trackData.nPts;
                searchEnd = trackData.nPts;
            end

            pts = trackData.points(searchStart:searchEnd, :);
            dist2 = (pts(:,1) - x).^2 + (pts(:,2) - y).^2;
            [~, localIdx] = min(dist2);
            idx = searchStart + localIdx - 1;

            refPoint = trackData.points(idx, :);
            refHeading = trackData.heading(idx);
            refS = trackData.arcLen(idx);
            if idx >= trackData.nPts
                refS = trackData.length;
            end
            dx = x - refPoint(1);
            dy = y - refPoint(2);
            lateralError = dx * (-sin(refHeading)) + dy * cos(refHeading);

            ref = struct( ...
                'idx', idx, ...
                's', refS, ...
                'x', refPoint(1), ...
                'y', refPoint(2), ...
                'heading', refHeading, ...
                'curvature', trackData.curvature(idx), ...
                'mu', trackData.mu(idx), ...
                'lateralError', lateralError);
        end

        function tireData = updatePlanarTireForces(obj, state, cornerLoads, mu)
            vm = obj.vehicleManager;
            kin = obj.getCornerKinematics(state.steer);
            corners = {'FL', 'FR', 'RL', 'RR'};

            tireData.sumFxBody = 0;
            tireData.sumFyBody = 0;
            tireData.yawMoment = 0;
            slipAngles = struct();
            slipRatios = struct();
            wheelHeadings = struct();

            for i = 1:numel(corners)
                corner = corners{i};
                tireState = vm.tire.(corner);
                cornerKin = kin.(corner);

                vxCorner = state.vx - state.yawRate * cornerKin.yPosition;
                vyCorner = state.vy + state.yawRate * cornerKin.xPosition;
                wheelHeading = cornerKin.steerAngle + cornerKin.toeAngle;

                alpha = wheelHeading - atan2(vyCorner, max(vxCorner, eps));
                longSpeed = vxCorner * cos(wheelHeading) + vyCorner * sin(wheelHeading);
                kappa = obj.computeLocalSlipRatio(tireState, longSpeed);

                slipAngles.(corner) = alpha;
                slipRatios.(corner) = kappa;
                wheelHeadings.(corner) = wheelHeading;
            end

            if ismethod(vm.tire, 'updateAllCorners')
                vm.tire.updateAllCorners( ...
                    cornerLoads.FL, cornerLoads.FR, cornerLoads.RL, cornerLoads.RR, ...
                    slipAngles.FL, slipAngles.FR, slipAngles.RL, slipAngles.RR, ...
                    slipRatios.FL, slipRatios.FR, slipRatios.RL, slipRatios.RR, mu, ...
                    kin.FL.camberAngle, kin.FR.camberAngle, ...
                    kin.RL.camberAngle, kin.RR.camberAngle);
            else
                for i = 1:numel(corners)
                    corner = corners{i};
                    tireState = vm.tire.(corner);
                    cornerKin = kin.(corner);
                    vm.tire.updateCorner(tireState, cornerLoads.(corner), ...
                        slipAngles.(corner), slipRatios.(corner), ...
                        cornerKin.camberAngle, mu);
                end
            end

            for i = 1:numel(corners)
                corner = corners{i};
                tireState = vm.tire.(corner);
                cornerKin = kin.(corner);
                wheelHeading = wheelHeadings.(corner);

                FxBody = tireState.Fx * cos(wheelHeading) - tireState.Fy * sin(wheelHeading);
                FyBody = tireState.Fx * sin(wheelHeading) + tireState.Fy * cos(wheelHeading);

                tireData.(sprintf('FxBody_%s', corner)) = FxBody;
                tireData.(sprintf('FyBody_%s', corner)) = FyBody;
                tireData.sumFxBody = tireData.sumFxBody + FxBody;
                tireData.sumFyBody = tireData.sumFyBody + FyBody;
                tireData.yawMoment = tireData.yawMoment + ...
                    cornerKin.xPosition * FyBody - cornerKin.yPosition * FxBody;
            end
        end

        function dynamics = computePlanarDynamics(obj, state, tireData, F_drag, totalNormalLoad)
            vm = obj.vehicleManager;
            rollingResistance = 0.015 * totalNormalLoad;
            forwardSign = sign(state.vx);
            if forwardSign == 0
                forwardSign = 1;
            end

            netFx = tireData.sumFxBody ...
                - forwardSign * F_drag ...
                - forwardSign * rollingResistance;
            netFy = tireData.sumFyBody;

            dynamics.ax = netFx / vm.totalMass;
            dynamics.ay = netFy / vm.totalMass;
            dynamics.yawAccel = tireData.yawMoment / max(vm.yawInertia, eps);
        end

        function kin = getCornerKinematics(obj, steer)
            vm = obj.vehicleManager;
            if ~isempty(vm.suspension) && ismethod(vm.suspension, 'getCornerKinematics')
                kin = vm.suspension.getCornerKinematics();
            else
                kin = struct();
                kin.FL = struct('camberAngle', 0, 'toeAngle', 0, 'steerAngle', steer);
                kin.FR = struct('camberAngle', 0, 'toeAngle', 0, 'steerAngle', steer);
                kin.RL = struct('camberAngle', 0, 'toeAngle', 0, 'steerAngle', 0);
                kin.RR = struct('camberAngle', 0, 'toeAngle', 0, 'steerAngle', 0);
            end

            [kin.FL.xPosition, kin.FL.yPosition] = obj.getWheelPosition('FL');
            [kin.FR.xPosition, kin.FR.yPosition] = obj.getWheelPosition('FR');
            [kin.RL.xPosition, kin.RL.yPosition] = obj.getWheelPosition('RL');
            [kin.RR.xPosition, kin.RR.yPosition] = obj.getWheelPosition('RR');
        end

        function [x, y] = getWheelPosition(obj, corner)
            vm = obj.vehicleManager;
            frontArm = vm.wheelbase * (1 - vm.staticFrontWeight);
            rearArm = vm.wheelbase * vm.staticFrontWeight;
            halfTrack = vm.trackWidth / 2;

            switch upper(corner)
                case 'FL'
                    x = frontArm;
                    y = halfTrack;
                case 'FR'
                    x = frontArm;
                    y = -halfTrack;
                case 'RL'
                    x = -rearArm;
                    y = halfTrack;
                otherwise
                    x = -rearArm;
                    y = -halfTrack;
            end
        end

        function kappa = computeLocalSlipRatio(~, cornerState, longitudinalSpeed)
            wheelSpeed = cornerState.angularVelocity * cornerState.wheelRadius;
            denom = max(abs(wheelSpeed), abs(longitudinalSpeed));
            if denom < 0.1
                kappa = 0;
            else
                kappa = (wheelSpeed - longitudinalSpeed) / denom;
            end
            kappa = max(-1, min(1, kappa));
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
