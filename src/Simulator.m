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
    % The Simulator composes a VehicleManager and asks a driver/controller
    % for inputs during full-lap simulation. Vehicle dynamics remain in
    % step(); driver policy remains outside the physics transition.
    
    properties
        % Reference to VehicleManager (components + vehicle parameters)
        vehicleManager
        
        % Reference to DriverModel/controller (computes driver inputs)
        driverModel
        
        % Simulation timestep [s]
        dt = 0.001

        % Driver input actuator limit for externally supplied steering.
        steeringRampTime = 0.10

        % Wheel-contact solve iterations per step. Each iteration re-derives
        % wheel omega from the latest tire Fx and re-evaluates the tire force
        % at the new slip, removing the one-step lag of the old single-pass
        % update. 1 reproduces the legacy explicit-Euler behavior; 2-3 is
        % the recommended semi-implicit default.
        wheelSolveIterations = 2

        % Internal: track whether maxSpeed warning was issued (warn once)
        warnedMaxSpeed = false
    end

    properties (Transient = true) %#ok<MCNPC>
        % Lazily-cached run invariants. The VehicleManager and its components
        % do not change during a simulation, so capability/value lookups that
        % the hot loop used to repeat every step (isa/ismethod/isprop) are
        % resolved once on first use and memoized here. NaN/empty = uncached.
        cachedWheelInertia = struct([])
        cachedHasChassis
        cachedDiffLocksWheels
        cachedTireHasBatchUpdate
        cachedSuspensionHasKinematics
        cachedDriverInputMethod   % 'computeInput' | 'computeInputs' | '' (cached)
        cachedFrontArm = NaN       % CG-to-front-axle moment arm [m] (run invariant)
        cachedRearArm = NaN        % CG-to-rear-axle moment arm [m] (run invariant)
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
            % Driver inputs are supplied by a driver/controller. Track
            % curvature is reference telemetry only; planar motion comes
            % from summed tire forces and yaw moment.
            
            vm = obj.vehicleManager;
            input = obj.normalizeDriverInput(input, state);
            throttle = input.throttle;
            brake = input.brake;
            steer = input.steer;

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
            W = vm.totalMass * vm.g;
            
            suspensionInputState = state;
            suspensionInputState.steer = steer;
            if obj.hasChassis()
                cornerLoads = obj.getCurrentCornerLoads(steer);
            else
                cornerLoads = vm.suspension.estimateCornerLoads( ...
                    suspensionInputState, aeroForces.Fz_front, aeroForces.Fz_rear, vm.totalMass);
            end
            
            % --- POWERTRAIN STATE & DRIVE TORQUE ---
            % Motor speed samples the differential carrier (mean of the
            % driven wheels for an open/LSD diff, the common locked speed
            % for a spool). Falls back to the raw mean if no differential.
            carrierOmega0 = 0.5 * (vm.tire.RL.angularVelocity + vm.tire.RR.angularVelocity);
            vm.powertrain.updateStateFromDrivenWheels(carrierOmega0);
            totalDriveTorque = vm.powertrain.computeDriveTorque(state.speed, throttle);

            % Off-throttle motoring/regen drag on the driven axle (opt-in via
            % motoringDragTorque / regenEnabled on the powertrain; 0 when off,
            % so baseline behavior is unchanged). Applied equally to both rear
            % wheels as a coastdown braking torque opposing rotation.
            T_coast_rear = 0;
            if ismethod(vm.powertrain, 'computeCoastdownTorque')
                T_coast_rear = 0.5 * vm.powertrain.computeCoastdownTorque( ...
                    state.speed, throttle);
            end

            % --- WHEEL TORQUE SETUP ---
            % RWD assumption: drive torque only on rear wheels.
            % Brake distribution: fixed front/rear bias from VehicleManager.
            R = vm.tire.RL.wheelRadius;  % all corners share same radius
            T_drive_front = 0;
            % Rear drive torque is split by the differential model. If no
            % differential is configured, fall back to the legacy 50/50 open
            % behavior (mean-speed carrier).
            inertia = obj.getWheelInertia();  % struct FL/FR/RL/RR
            diffOut = obj.solveDifferential( ...
                totalDriveTorque, vm.tire.RL.angularVelocity, vm.tire.RR.angularVelocity, ...
                inertia.RL, obj.dt);
            T_drive_RL = diffOut.TL + T_coast_rear;
            T_drive_RR = diffOut.TR + T_coast_rear;

            % --- BRAKE TORQUE ---
            brakeCommand = max(0, min(1, brake));
            brakeBiasFront = max(0, min(1, vm.brakeBiasFront));
            brakeBiasRear = 1 - brakeBiasFront;
            % Existing brakeForceCoefficient is preserved as an equivalent
            % total brake force capacity, then converted to wheel torque.
            % The vehicles have no ABS, so commanded brake torque is applied
            % directly at a fixed front/rear bias (wheels may lock).
            totalNormalLoad = W + F_downforce;
            brakeForceCapacity = max(0, vm.brakeForceCoefficient) * totalNormalLoad;
            brakeForceMag = brakeCommand * brakeForceCapacity;
            F_brake_front_cmd = brakeForceMag * brakeBiasFront;
            F_brake_rear_cmd = brakeForceMag * brakeBiasRear;
            effectiveBrakeCommand = brakeCommand;
            
            % --- WHEEL DYNAMICS & SLIP RATIO ---
            % Per-corner brake torque by axle bias
            T_brake_front = F_brake_front_cmd * R / 2;
            T_brake_rear = F_brake_rear_cmd * R / 2;

            % Semi-implicit wheel-contact solve. Each iteration advances
            % wheel omega from the current tire Fx, then re-evaluates the
            % tire force at the new slip ratio, so omega and Fx converge
            % within the step instead of lagging by one timestep. The
            % driven-wheel speed feeds back into the powertrain so the rev
            % limiter sees the converged motor speed.
            tireInputState = state;
            tireInputState.steer = steer;
            wheelLongSpeeds = obj.computeCornerLongitudinalSpeeds(tireInputState);

            nWheelIter = max(1, round(obj.wheelSolveIterations));
            for iter = 1:nWheelIter
                vm.tire.updateWheelDynamics(vm.tire.FL, T_drive_front, T_brake_front, obj.dt, inertia.FL, wheelLongSpeeds.FL);
                vm.tire.updateWheelDynamics(vm.tire.FR, T_drive_front, T_brake_front, obj.dt, inertia.FR, wheelLongSpeeds.FR);
                vm.tire.updateWheelDynamics(vm.tire.RL, T_drive_RL, T_brake_rear, obj.dt, inertia.RL, wheelLongSpeeds.RL);
                vm.tire.updateWheelDynamics(vm.tire.RR, T_drive_RR, T_brake_rear, obj.dt, inertia.RR, wheelLongSpeeds.RR);

                % Re-solve the differential at the updated wheel speeds so a
                % locked diff enforces a common speed and an LSD re-biases
                % torque. Driven-wheel speed then feeds the powertrain.
                diffOut = obj.solveDifferential( ...
                    totalDriveTorque, vm.tire.RL.angularVelocity, vm.tire.RR.angularVelocity, ...
                    inertia.RL, obj.dt);
                T_drive_RL = diffOut.TL + T_coast_rear;
                T_drive_RR = diffOut.TR + T_coast_rear;

                % A locked differential mechanically forces both driven
                % wheels to the carrier speed.
                if obj.differentialLocksWheels()
                    vm.tire.RL.angularVelocity = diffOut.carrierOmega;
                    vm.tire.RR.angularVelocity = diffOut.carrierOmega;
                end

                % Motor speed from the differential carrier speed (mean for
                % open/LSD, common locked speed for a spool). The RPM
                % limiter acts through torque output on the next drive
                % torque evaluation, not by overwriting omega.
                vm.powertrain.updateStateFromDrivenWheels(diffOut.carrierOmega);

                % Evaluate tire forces at the converged slip so the next
                % wheel update uses a consistent Fx. Relaxation (contact-
                % patch lag) advances only on the final iteration so the
                % lag time constant is not shrunk by the sub-iteration.
                if iter < nWheelIter
                    tireData = obj.updatePlanarTireForces(tireInputState, cornerLoads, 0, false);
                else
                    tireData = obj.updatePlanarTireForces(tireInputState, cornerLoads, obj.dt);
                end
            end
            dynamics = obj.computePlanarDynamics(state, tireData, F_drag);

            % One predictor/corrector pass for load transfer using current
            % force-derived body accelerations.
            if ~obj.hasChassis()
                correctedLoadState = state;
                correctedLoadState.steer = steer;
                correctedLoadState.ax = dynamics.ax;
                correctedLoadState.ay = dynamics.ay;
                cornerLoads = vm.suspension.computeCornerLoads( ...
                    correctedLoadState, aeroForces.Fz_front, aeroForces.Fz_rear, vm.totalMass, obj.dt);
                % Re-evaluate tire forces at the corrected loads with dt = 0: the
                % contact-patch relaxation already advanced once on the final
                % wheel-solve iteration above, so it must not advance again here.
                tireData = obj.updatePlanarTireForces(tireInputState, cornerLoads, 0);
                dynamics = obj.computePlanarDynamics(state, tireData, F_drag);
            end

            % Integrate the sprung-mass attitude (heave/pitch/roll) from the
            % converged body accelerations and aero forces. The chassis owns
            % pitch, roll, and ride-height, which feed next step's aero
            % (ground effect, pitch sensitivity) and are read back by
            % VehicleState.computePitch/Roll/RideHeight below.
            if ~isempty(vm.chassis) && ...
                    isa(vm.chassis, 'components.Chassis.ChassisComponent')
                vm.chassis.updateFromAccelerations( ...
                    dynamics.ax, dynamics.ay, aeroForces, obj.dt);
            end

            vm.powertrain.updateStateFromDrivenWheels( ...
                [vm.tire.RL.angularVelocity, vm.tire.RR.angularVelocity]);

            F_tire_long = tireData.sumFxBody;
            F_drive = max(0, F_tire_long);
            F_brake = min(0, F_tire_long);
            % Telemetry only: equivalent rolling-resistance force from the
            % per-wheel torque model (sum of Crr*Fz over the four corners).
            % This is already reflected in the tire Fx above, not applied again.
            corners = vm.tire;
            rollingResistanceCoeff = 0;
            if isprop(vm.tire, 'rollingResistanceCoeff')
                rollingResistanceCoeff = vm.tire.rollingResistanceCoeff;
            end
            F_rollResist = rollingResistanceCoeff * ...
                (corners.FL.normalForce + corners.FR.normalForce + ...
                 corners.RL.normalForce + corners.RR.normalForce);

            % --- INTEGRATE STATE ---
            vx0 = state.vx;
            vy0 = state.vy;
            yaw0 = state.yaw;

            % Midpoint yaw for a consistent body<->world rotation over the
            % step: both the body->world accel projection and the world->body
            % velocity reprojection use the same (midpoint) yaw, so the step
            % is not biased by an old-vs-new yaw asymmetry.
            yawRateNew = state.yawRate + dynamics.yawAccel * obj.dt;
            yawNew = yaw0 + yawRateNew * obj.dt;
            yawMid = yaw0 + 0.5 * yawRateNew * obj.dt;

            cy0 = cos(yaw0); sy0 = sin(yaw0);
            cyMid = cos(yawMid); syMid = sin(yawMid);

            vxWorld0 = vx0 * cy0 - vy0 * sy0;
            vyWorld0 = vx0 * sy0 + vy0 * cy0;
            axWorld = dynamics.ax * cyMid - dynamics.ay * syMid;
            ayWorld = dynamics.ax * syMid + dynamics.ay * cyMid;

            vxWorld = vxWorld0 + axWorld * obj.dt;
            vyWorld = vyWorld0 + ayWorld * obj.dt;

            vxNew = vxWorld * cyMid + vyWorld * syMid;
            vyNew = -vxWorld * syMid + vyWorld * cyMid;
            xNew = state.x + 0.5 * (vxWorld0 + vxWorld) * obj.dt;
            yNew = state.y + 0.5 * (vyWorld0 + vyWorld) * obj.dt;

            nextRef = obj.projectToReference(xNew, yNew, ref.trackData, ref.idx);
            
            newState.throttle = throttle;
            newState.brake = effectiveBrakeCommand;
            newState.steer = steer;
            newState = newState.updateFromPlanarDynamics( ...
                dynamics.ax, dynamics.ay, dynamics.yawAccel, ...
                vxNew, vyNew, yawRateNew, yawNew, xNew, yNew, ...
                nextRef.s, nextRef.heading, nextRef.curvature, ...
                nextRef.lateralError, obj.dt, nextRef.mu);
            newState.onTrack = nextRef.onTrack;
            
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
            forces.brakeGrip_FL = max(vm.tire.FL.peakMu, 0) * max(cornerLoads.FL, 0);
            forces.brakeGrip_FR = max(vm.tire.FR.peakMu, 0) * max(cornerLoads.FR, 0);
            forces.brakeGrip_RL = max(vm.tire.RL.peakMu, 0) * max(cornerLoads.RL, 0);
            forces.brakeGrip_RR = max(vm.tire.RR.peakMu, 0) * max(cornerLoads.RR, 0);
            forces.driveTorqueTotal = totalDriveTorque;
            forces.driveTorque_RL = T_drive_RL;
            forces.driveTorque_RR = T_drive_RR;
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
            % aeroDragHeight is a passthrough of aeroForces.dragHeight
            % (no recomputation needed); only the pitch moments require the
            % moment bookkeeping. With a chassis attached (the default),
            % that bookkeeping already happened in updateFromAccelerations
            % and is overwritten below, so computeAeroPitchMoments is only
            % invoked for the no-chassis fallback.
            forces.aeroDragHeight = localGetField(aeroForces, 'dragHeight', 0);
            if obj.hasChassis()
                forces.downforcePitchMoment = vm.chassis.state.downforcePitchMoment;
                forces.dragPitchMoment = vm.chassis.state.dragPitchMoment;
                forces.aeroPitchMoment = vm.chassis.state.aeroPitchMoment;
            else
                aeroMoments = obj.computeAeroPitchMoments(aeroForces);
                forces.downforcePitchMoment = aeroMoments.downforce;
                forces.dragPitchMoment = aeroMoments.drag;
                forces.aeroPitchMoment = aeroMoments.total;
            end
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
            trackPtsBase  = track.getTrackPoints();
            curvatureBase = track.getCurvature();
            muBase        = track.getSurfaceFriction();
            headingBase   = track.getHeading();
            baseTrackLen  = track.getTotalLength();
            trackWidth    = track.getTrackWidth();

            warmupLaps = obj.getTrackWarmupLaps(track);
            recordedLaps = obj.getTrackRecordedLaps(track);
            totalLaps = warmupLaps + recordedLaps;

            closedLoop = obj.isClosedLoopTrack(track, trackPtsBase);
            if totalLaps > 1
                if ~closedLoop
                    error('Simulator:WarmupRequiresClosedLoop', ...
                        'Track warmup/recorded laps require a closed-loop track.');
                end
                [trackPts, curvature, mu, heading] = obj.repeatClosedTrack( ...
                    trackPtsBase, curvatureBase, muBase, headingBase, totalLaps);
            else
                trackPts = trackPtsBase;
                curvature = curvatureBase;
                mu = muBase;
                heading = headingBase;
            end
            nPts = size(trackPts, 1);
            
            % Compute arc-length parameterization
            dx = diff(trackPts(:,1));
            dy = diff(trackPts(:,2));
            segLen = sqrt(dx.^2 + dy.^2);
            arcLen = [0; cumsum(segLen)];
            trackLen = arcLen(end);
            recordStartS = warmupLaps * baseTrackLen;
            recordEndS = min(trackLen, recordStartS + recordedLaps * baseTrackLen);
            trackData = struct( ...
                'points', trackPts, ...
                'arcLen', arcLen, ...
                'curvature', curvature(:), ...
                'mu', mu(:), ...
                'heading', heading(:), ...
                'length', trackLen, ...
                'trackWidth', trackWidth, ...
                'trackHalfWidth', trackWidth / 2, ...
                'closedLoop', closedLoop, ...
                'baseTrackLength', baseTrackLen, ...
                'totalLaps', totalLaps, ...
                'lapBreakS', (0:totalLaps)' * baseTrackLen, ...
                'nPts', nPts);
            trackData = obj.precomputeTrackSegments(trackData);
            initialState = obj.initializePlanarState(initialState, trackData);
            if ~isempty(obj.driverModel) && ...
                    ismethod(obj.driverModel, 'prepareForSimulation')
                obj.driverModel = obj.driverModel.prepareForSimulation( ...
                    initialState, trackData, obj.dt);
            end
            
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
                'bodySlipAngle', zeros(maxSteps, 1), ...
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
                'onTrack',     false(maxSteps, 1), ...
                'trackWidth',  zeros(maxSteps, 1), ...
                'trackLimitMargin', zeros(maxSteps, 1), ...
                'throttle',    zeros(maxSteps, 1), ...
                'brake',       zeros(maxSteps, 1), ...
                'brakeRequested', zeros(maxSteps, 1), ...
                'steer',       zeros(maxSteps, 1), ...
                'targetSpeed', NaN(maxSteps, 1), ...
                'axRef',       NaN(maxSteps, 1), ...
                'targetLateralError', NaN(maxSteps, 1), ...
                'lineCurvature', NaN(maxSteps, 1), ...
                'speedError',  NaN(maxSteps, 1), ...
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
                'rollAngle',   zeros(maxSteps, 1), ...
                'frontRollAngle', zeros(maxSteps, 1), ...
                'rearRollAngle',  zeros(maxSteps, 1), ...
                'twistAngle',     zeros(maxSteps, 1), ...
                'rideHeight',  zeros(maxSteps, 1), ...
                'Fz_FL',       zeros(maxSteps, 1), ...
                'Fz_FR',       zeros(maxSteps, 1), ...
                'Fz_RL',       zeros(maxSteps, 1), ...
                'Fz_RR',       zeros(maxSteps, 1), ...
                'suspensionForce_FL', zeros(maxSteps, 1), ...
                'suspensionForce_FR', zeros(maxSteps, 1), ...
                'suspensionForce_RL', zeros(maxSteps, 1), ...
                'suspensionForce_RR', zeros(maxSteps, 1), ...
                'antiRollBarForce_FL', zeros(maxSteps, 1), ...
                'antiRollBarForce_FR', zeros(maxSteps, 1), ...
                'antiRollBarForce_RL', zeros(maxSteps, 1), ...
                'antiRollBarForce_RR', zeros(maxSteps, 1), ...
                'suspensionDemand_FL', zeros(maxSteps, 1), ...
                'suspensionDemand_FR', zeros(maxSteps, 1), ...
                'suspensionDemand_RL', zeros(maxSteps, 1), ...
                'suspensionDemand_RR', zeros(maxSteps, 1), ...
                'tireDeflection_FL', zeros(maxSteps, 1), ...
                'tireDeflection_FR', zeros(maxSteps, 1), ...
                'tireDeflection_RL', zeros(maxSteps, 1), ...
                'tireDeflection_RR', zeros(maxSteps, 1), ...
                'damperPos_FL', zeros(maxSteps, 1), ...
                'damperPos_FR', zeros(maxSteps, 1), ...
                'damperPos_RL', zeros(maxSteps, 1), ...
                'damperPos_RR', zeros(maxSteps, 1), ...
                'damperVel_FL', zeros(maxSteps, 1), ...
                'damperVel_FR', zeros(maxSteps, 1), ...
                'damperVel_RL', zeros(maxSteps, 1), ...
                'damperVel_RR', zeros(maxSteps, 1), ...
                'sprungPosition_FL', zeros(maxSteps, 1), ...
                'sprungPosition_FR', zeros(maxSteps, 1), ...
                'sprungPosition_RL', zeros(maxSteps, 1), ...
                'sprungPosition_RR', zeros(maxSteps, 1), ...
                'unsprungPosition_FL', zeros(maxSteps, 1), ...
                'unsprungPosition_FR', zeros(maxSteps, 1), ...
                'unsprungPosition_RL', zeros(maxSteps, 1), ...
                'unsprungPosition_RR', zeros(maxSteps, 1), ...
                'sprungVelocity_FL', zeros(maxSteps, 1), ...
                'sprungVelocity_FR', zeros(maxSteps, 1), ...
                'sprungVelocity_RL', zeros(maxSteps, 1), ...
                'sprungVelocity_RR', zeros(maxSteps, 1), ...
                'unsprungVelocity_FL', zeros(maxSteps, 1), ...
                'unsprungVelocity_FR', zeros(maxSteps, 1), ...
                'unsprungVelocity_RL', zeros(maxSteps, 1), ...
                'unsprungVelocity_RR', zeros(maxSteps, 1), ...
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
            currentRef = obj.projectToReference(currentState.x, ...
                currentState.y, trackData, 1);
            
            step = 0;
            fprintf('Starting simulation...\n');
            fprintf('Track length: %.1f m\n', trackLen);
            if warmupLaps > 0
                fprintf('Telemetry: dropping %d warmup lap(s), recording %d lap(s)\n', ...
                    warmupLaps, recordedLaps);
            end
            
            finishTolerance = 1e-6;
            while currentState.s < trackLen - finishTolerance && currentState.onTrack
                currentRef = obj.projectToReference( ...
                    currentState.x, currentState.y, trackData, currentRef.idx);
                currentState.s = currentRef.s;
                currentState.refS = currentRef.s;
                currentState.refHeading = currentRef.heading;
                currentState.refCurvature = currentRef.curvature;
                currentState.curvature = currentRef.curvature;
                currentState.lateralError = currentRef.lateralError;
                currentState.mu = currentRef.mu;
                currentState.onTrack = currentRef.onTrack;
                if ~currentState.onTrack
                    break;
                end
                step = step + 1;

                % --- DRIVER INPUTS ---
                ref = currentRef;
                ref.trackData = trackData;
                input = obj.computeDriverInput(currentState, ref);
                
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
                    stateLog.bodySlipAngle(step) = newState.bodySlipAngle;
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
                    stateLog.onTrack(step)     = newState.onTrack;
                    stateLog.trackWidth(step)  = trackData.trackWidth;
                    stateLog.trackLimitMargin(step) = ...
                        trackData.trackHalfWidth - abs(newState.lateralError);
                    stateLog.throttle(step)    = input.throttle;
                    stateLog.brake(step)       = forces.brake;
                    stateLog.brakeRequested(step) = forces.brakeCommand;
                    stateLog.steer(step)       = input.steer;
                    targetSpeedForLog = localGetField(input, 'targetSpeed', NaN);
                    stateLog.targetSpeed(step) = targetSpeedForLog;
                    stateLog.axRef(step)       = localGetField(input, 'axRef', NaN);
                    stateLog.targetLateralError(step) = ...
                        localGetField(input, 'targetLateralError', NaN);
                    stateLog.lineCurvature(step) = ...
                        localGetField(input, 'lineCurvature', NaN);
                    if isfield(input, 'speedError') && isfinite(input.speedError)
                        stateLog.speedError(step) = input.speedError;
                    elseif isfinite(targetSpeedForLog)
                        stateLog.speedError(step) = currentState.speed - targetSpeedForLog;
                    end
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
                    stateLog.rollAngle(step)   = newState.rollAngle;
                    stateLog.frontRollAngle(step) = newState.frontRollAngle;
                    stateLog.rearRollAngle(step)  = newState.rearRollAngle;
                    stateLog.twistAngle(step)     = newState.twistAngle;
                    stateLog.rideHeight(step)  = newState.rideHeight;
                    stateLog.aeroFz_front(step) = forces.aeroFz_front;
                    stateLog.aeroFz_rear(step)  = forces.aeroFz_rear;
                    
                    % Per-corner suspension telemetry
                    susp = vm.suspension;
                    stateLog.Fz_FL(step)       = susp.frontLeft.state.tireNormalForce;
                    stateLog.Fz_FR(step)       = susp.frontRight.state.tireNormalForce;
                    stateLog.Fz_RL(step)       = susp.rearLeft.state.tireNormalForce;
                    stateLog.Fz_RR(step)       = susp.rearRight.state.tireNormalForce;
                    stateLog.suspensionForce_FL(step) = susp.frontLeft.state.suspensionForce;
                    stateLog.suspensionForce_FR(step) = susp.frontRight.state.suspensionForce;
                    stateLog.suspensionForce_RL(step) = susp.rearLeft.state.suspensionForce;
                    stateLog.suspensionForce_RR(step) = susp.rearRight.state.suspensionForce;
                    stateLog.antiRollBarForce_FL(step) = susp.frontLeft.state.antiRollBarForce;
                    stateLog.antiRollBarForce_FR(step) = susp.frontRight.state.antiRollBarForce;
                    stateLog.antiRollBarForce_RL(step) = susp.rearLeft.state.antiRollBarForce;
                    stateLog.antiRollBarForce_RR(step) = susp.rearRight.state.antiRollBarForce;
                    stateLog.suspensionDemand_FL(step) = susp.frontLeft.state.demandedLoad;
                    stateLog.suspensionDemand_FR(step) = susp.frontRight.state.demandedLoad;
                    stateLog.suspensionDemand_RL(step) = susp.rearLeft.state.demandedLoad;
                    stateLog.suspensionDemand_RR(step) = susp.rearRight.state.demandedLoad;
                    stateLog.tireDeflection_FL(step) = susp.frontLeft.state.tireDeflection;
                    stateLog.tireDeflection_FR(step) = susp.frontRight.state.tireDeflection;
                    stateLog.tireDeflection_RL(step) = susp.rearLeft.state.tireDeflection;
                    stateLog.tireDeflection_RR(step) = susp.rearRight.state.tireDeflection;
                    stateLog.damperPos_FL(step) = susp.frontLeft.state.damperPosition;
                    stateLog.damperPos_FR(step) = susp.frontRight.state.damperPosition;
                    stateLog.damperPos_RL(step) = susp.rearLeft.state.damperPosition;
                    stateLog.damperPos_RR(step) = susp.rearRight.state.damperPosition;
                    stateLog.damperVel_FL(step) = susp.frontLeft.state.damperVelocity;
                    stateLog.damperVel_FR(step) = susp.frontRight.state.damperVelocity;
                    stateLog.damperVel_RL(step) = susp.rearLeft.state.damperVelocity;
                    stateLog.damperVel_RR(step) = susp.rearRight.state.damperVelocity;
                    stateLog.sprungPosition_FL(step) = susp.frontLeft.state.sprungPosition;
                    stateLog.sprungPosition_FR(step) = susp.frontRight.state.sprungPosition;
                    stateLog.sprungPosition_RL(step) = susp.rearLeft.state.sprungPosition;
                    stateLog.sprungPosition_RR(step) = susp.rearRight.state.sprungPosition;
                    stateLog.unsprungPosition_FL(step) = susp.frontLeft.state.unsprungPosition;
                    stateLog.unsprungPosition_FR(step) = susp.frontRight.state.unsprungPosition;
                    stateLog.unsprungPosition_RL(step) = susp.rearLeft.state.unsprungPosition;
                    stateLog.unsprungPosition_RR(step) = susp.rearRight.state.unsprungPosition;
                    stateLog.sprungVelocity_FL(step) = susp.frontLeft.state.sprungVelocity;
                    stateLog.sprungVelocity_FR(step) = susp.frontRight.state.sprungVelocity;
                    stateLog.sprungVelocity_RL(step) = susp.rearLeft.state.sprungVelocity;
                    stateLog.sprungVelocity_RR(step) = susp.rearRight.state.sprungVelocity;
                    stateLog.unsprungVelocity_FL(step) = susp.frontLeft.state.unsprungVelocity;
                    stateLog.unsprungVelocity_FR(step) = susp.frontRight.state.unsprungVelocity;
                    stateLog.unsprungVelocity_RL(step) = susp.rearLeft.state.unsprungVelocity;
                    stateLog.unsprungVelocity_RR(step) = susp.rearRight.state.unsprungVelocity;
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
            
            simulationSteps = step;

            % Trim logs
            fields = fieldnames(stateLog);
            for i = 1:numel(fields)
                stateLog.(fields{i}) = stateLog.(fields{i})(1:step);
            end

            [stateLog, lapTime, recordedSteps] = obj.applyTelemetryLapWindow( ...
                stateLog, recordStartS, recordEndS);
            if recordedSteps > 0
                maxSpeedKmh = max(stateLog.speedKmh);
                recordedLength = max(stateLog.s);
            else
                maxSpeedKmh = 0;
                recordedLength = 0;
            end
            
            fprintf('\n=== Simulation Complete ===\n');
            fprintf('Lap Time:   %.3f s\n', lapTime);
            fprintf('Track Length: %.1f m\n', recordedLength);
            fprintf('Max Speed:  %.1f km/h\n', maxSpeedKmh);
            fprintf('Steps:      %d simulated, %d recorded\n', ...
                simulationSteps, recordedSteps);
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

        function laps = getTrackWarmupLaps(~, track)
            laps = 0;
            if ismethod(track, 'getWarmupLaps')
                laps = track.getWarmupLaps();
            elseif isprop(track, 'warmupLaps')
                laps = track.warmupLaps;
            end
            laps = max(0, round(laps));
        end

        function laps = getTrackRecordedLaps(~, track)
            laps = 1;
            if ismethod(track, 'getRecordedLaps')
                laps = track.getRecordedLaps();
            elseif isprop(track, 'recordedLaps')
                laps = track.recordedLaps;
            end
            laps = max(1, round(laps));
        end

        function closed = isClosedLoopTrack(~, track, points)
            closed = norm(points(1, :) - points(end, :)) <= 0.05;
            if ismethod(track, 'isClosedLoop')
                closed = track.isClosedLoop();
            elseif isprop(track, 'closedLoop')
                closed = track.closedLoop;
            elseif isprop(track, 'Closed')
                % WaypointTrack exposes Closed rather than closedLoop/isClosedLoop
                closed = track.Closed;
            end
            closed = logical(closed);
        end

        function [points, curvature, mu, heading] = repeatClosedTrack(~, ...
                points, curvature, mu, heading, lapCount)
            curvature = curvature(:);
            mu = mu(:);
            heading = heading(:);
            if lapCount <= 1
                return;
            end

            basePoints = points;
            baseCurvature = curvature;
            baseMu = mu;
            baseHeading = heading;

            % A closed loop may be stored two ways: with an explicit closure
            % point (points(1) ~= points(end), legacy convention) or without
            % (points(1) ~= points(end), WaypointTrack/cleanPoints convention).
            % hasClosurePoint selects the tiling so consecutive laps do not
            % duplicate the junction point.
            repeatStartIdx = 1;
            hasClosurePoint = norm(basePoints(1, :) - basePoints(end, :)) <= 0.05;
            if hasClosurePoint
                repeatStartIdx = 2;
            end

            for lapIdx = 2:lapCount %#ok<NASGU>
                points = [points; basePoints(repeatStartIdx:end, :)]; %#ok<AGROW>
                curvature = [curvature; baseCurvature(repeatStartIdx:end)]; %#ok<AGROW>
                mu = [mu; baseMu(repeatStartIdx:end)]; %#ok<AGROW>
                heading = [heading; baseHeading(repeatStartIdx:end)]; %#ok<AGROW>
            end

            % Without an explicit closure point the final lap is missing its
            % closing edge, so the tiled length would fall short of
            % lapCount * getTotalLength() by one segment. Append the start
            % point once to complete the circuit.
            if lapCount > 1 && ~hasClosurePoint
                points = [points; basePoints(1, :)]; %#ok<AGROW>
                curvature = [curvature; baseCurvature(1)]; %#ok<AGROW>
                mu = [mu; baseMu(1)]; %#ok<AGROW>
                heading = [heading; baseHeading(1)]; %#ok<AGROW>
            end
        end

        function [stateLog, lapTime, recordedSteps] = applyTelemetryLapWindow(obj, ...
                stateLog, recordStartS, recordEndS)
            if isempty(stateLog.time)
                lapTime = 0;
                recordedSteps = 0;
                return;
            end

            trimDiagnostics = obj.computeTelemetryTrimDiagnostics(stateLog);
            keep = stateLog.s >= recordStartS - 1e-9 & ...
                stateLog.s <= recordEndS + 1e-9;
            fields = fieldnames(stateLog);
            for i = 1:numel(fields)
                stateLog.(fields{i}) = stateLog.(fields{i})(keep);
            end

            recordedSteps = nnz(keep);
            if recordedSteps == 0
                warning('Simulator:NoRecordedTelemetry', ...
                    ['No telemetry samples fell inside the recorded lap window ' ...
                    '(%.1f m to %.1f m). Simulation ended before the timed lap ' ...
                    'started or completed. Max simulated s was %.1f m, final ' ...
                    'speed was %.1f km/h, final lateral error was %.3f m, ' ...
                    'and minimum track margin was %.3f m.'], ...
                    recordStartS, recordEndS, ...
                    trimDiagnostics.maxS, ...
                    trimDiagnostics.finalSpeedKmh, ...
                    trimDiagnostics.finalLateralError, ...
                    trimDiagnostics.minTrackMargin);
                lapTime = 0;
                return;
            end

            if recordStartS > 0
                stateLog.time = stateLog.time - stateLog.time(1);
                if isfield(stateLog, 'controlTime')
                    stateLog.controlTime = stateLog.controlTime - stateLog.controlTime(1);
                end

                distanceFields = {'s', 'controlS', 'refS'};
                for i = 1:numel(distanceFields)
                    field = distanceFields{i};
                    if isfield(stateLog, field)
                        stateLog.(field) = max(0, stateLog.(field) - recordStartS);
                    end
                end
            end

            lapTime = stateLog.time(end);
        end

        function diagnostics = computeTelemetryTrimDiagnostics(~, stateLog)
            diagnostics.maxS = NaN;
            diagnostics.finalSpeedKmh = NaN;
            diagnostics.finalLateralError = NaN;
            diagnostics.minTrackMargin = NaN;

            if isfield(stateLog, 's') && ~isempty(stateLog.s)
                diagnostics.maxS = max(stateLog.s);
            end
            if isfield(stateLog, 'speedKmh') && ~isempty(stateLog.speedKmh)
                diagnostics.finalSpeedKmh = stateLog.speedKmh(end);
            elseif isfield(stateLog, 'speed') && ~isempty(stateLog.speed)
                diagnostics.finalSpeedKmh = stateLog.speed(end) * 3.6;
            end
            if isfield(stateLog, 'lateralError') && ~isempty(stateLog.lateralError)
                diagnostics.finalLateralError = stateLog.lateralError(end);
            end
            if isfield(stateLog, 'trackLimitMargin') && ...
                    ~isempty(stateLog.trackLimitMargin)
                diagnostics.minTrackMargin = min(stateLog.trackLimitMargin);
            end
        end

        function input = computeDriverInput(obj, state, observation)
            if isempty(obj.driverModel)
                input = struct('throttle', 0, 'brake', 0, 'steer', 0);
                return;
            end

            % The driver's input-method capability is a run invariant; resolve
            % it once instead of calling ismethod every step.
            if isempty(obj.cachedDriverInputMethod)
                if ismethod(obj.driverModel, 'computeInput')
                    obj.cachedDriverInputMethod = 'computeInput';
                elseif ismethod(obj.driverModel, 'computeInputs')
                    obj.cachedDriverInputMethod = 'computeInputs';
                else
                    obj.cachedDriverInputMethod = 'none';
                end
            end

            switch obj.cachedDriverInputMethod
                case 'computeInput'
                    input = obj.driverModel.computeInput(state, observation);
                case 'computeInputs'
                    [throttle, brake, steer] = obj.driverModel.computeInputs(state);
                    input = struct( ...
                        'throttle', throttle, ...
                        'brake', brake, ...
                        'steer', steer);
                otherwise
                    error('Simulator:InvalidDriverModel', ...
                        'driverModel must implement computeInput or computeInputs.');
            end

            if ~isfield(input, 'throttle')
                input.throttle = 0;
            end
            if ~isfield(input, 'brake')
                input.brake = 0;
            end
            if ~isfield(input, 'steer')
                input.steer = 0;
            end
            input = obj.normalizeDriverInput(input, state);
        end

        function input = normalizeDriverInput(obj, input, state)
            if ~isfield(input, 'throttle') || isempty(input.throttle)
                input.throttle = 0;
            end
            if ~isfield(input, 'brake') || isempty(input.brake)
                input.brake = 0;
            end
            if ~isfield(input, 'steer') || isempty(input.steer)
                input.steer = 0;
            end

            input.throttle = max(0, min(1, input.throttle));
            input.brake = max(0, min(1, input.brake));

            if input.brake > 0
                input.throttle = 0;
            elseif input.throttle > 0
                input.brake = 0;
            end

            maxSteer = obj.getMaxSteeringAngle();
            input.steer = max(-maxSteer, min(maxSteer, input.steer));
            if nargin >= 3 && ~isempty(state)
                previousSteer = state.steer;
                if ~isfinite(previousSteer)
                    previousSteer = 0;
                end
                previousSteer = max(-maxSteer, min(maxSteer, previousSteer));
                rampTime = obj.getSteeringRampTime();
                if rampTime > 0 && isfinite(rampTime)
                    maxDelta = maxSteer * obj.dt / max(rampTime, eps);
                    delta = input.steer - previousSteer;
                    delta = max(-maxDelta, min(maxDelta, delta));
                    input.steer = previousSteer + delta;
                end
            end
        end

        function maxSteer = getMaxSteeringAngle(obj)
            maxSteer = 0.6;
            if ~isempty(obj.driverModel) && ...
                    isprop(obj.driverModel, 'maxSteeringAngle')
                maxSteer = obj.driverModel.maxSteeringAngle;
            end
            maxSteer = max(maxSteer, eps);
        end

        function rampTime = getSteeringRampTime(obj)
            rampTime = obj.steeringRampTime;
            if ~isempty(obj.driverModel) && ...
                    isprop(obj.driverModel, 'steeringRampTime')
                rampTime = obj.driverModel.steeringRampTime;
            end
        end

        function ref = referenceAtProgress(~, s, x, y, trackData)
            s = max(0, min(trackData.length, s));
            idx = find(trackData.arcLen <= s, 1, 'last');
            if isempty(idx)
                idx = 1;
            end
            idx = max(1, min(idx, trackData.nPts));

            if idx < trackData.nPts && trackData.arcLen(idx+1) > trackData.arcLen(idx)
                s0 = trackData.arcLen(idx);
                s1 = trackData.arcLen(idx+1);
                t = (s - s0) / max(s1 - s0, eps);
                refPoint = (1 - t) * trackData.points(idx, :) + ...
                    t * trackData.points(idx+1, :);
                refHeading = trackData.heading(idx);
                refCurvature = trackData.curvature(idx);
                refMu = trackData.mu(idx);
            else
                refPoint = trackData.points(idx, :);
                refHeading = trackData.heading(idx);
                refCurvature = trackData.curvature(idx);
                refMu = trackData.mu(idx);
            end

            dx = x - refPoint(1);
            dy = y - refPoint(2);
            lateralError = dx * (-sin(refHeading)) + dy * cos(refHeading);
            trackHalfWidth = trackData.trackHalfWidth;
            trackLimitMargin = trackHalfWidth - abs(lateralError);
            onTrack = trackLimitMargin >= -1e-9;

            ref = struct( ...
                'idx', idx, ...
                's', s, ...
                'x', refPoint(1), ...
                'y', refPoint(2), ...
                'heading', refHeading, ...
                'curvature', refCurvature, ...
                'mu', refMu, ...
                'lateralError', lateralError, ...
                'trackWidth', trackData.trackWidth, ...
                'trackHalfWidth', trackHalfWidth, ...
                'trackLimitMargin', trackLimitMargin, ...
                'onTrack', onTrack);
        end

        function ref = projectToReference(~, x, y, trackData, previousIdx)
            if nargin < 5 || isempty(previousIdx) || previousIdx < 1
                previousIdx = 1;
            end

            nSegments = max(trackData.nPts - 1, 1);
            hasSegmentCache = isfield(trackData, 'segmentVectors') && ...
                isfield(trackData, 'segmentLengths') && ...
                isfield(trackData, 'segmentInvLen2');
            if hasSegmentCache
                nSegments = max(size(trackData.segmentVectors, 1), 1);
            end
            previousIdx = max(1, min(previousIdx, trackData.nPts));
            backWindow = 10;
            forwardWindow = 80;
            searchStart = max(1, min(previousIdx - backWindow, nSegments));
            searchEnd = min(nSegments, max(previousIdx + forwardWindow, searchStart));

            bestDist2 = inf;
            bestIdx = searchStart;
            bestT = 0;
            bestPoint = trackData.points(bestIdx, :);

            queryPoint = [x, y];
            for segIdx = searchStart:searchEnd
                p0 = trackData.points(segIdx, :);
                if hasSegmentCache
                    v = trackData.segmentVectors(segIdx, :);
                    invLen2 = trackData.segmentInvLen2(segIdx);
                else
                    p1 = trackData.points(segIdx + 1, :);
                    v = p1 - p0;
                    len2 = dot(v, v);
                    if len2 > eps
                        invLen2 = 1 / len2;
                    else
                        invLen2 = 0;
                    end
                end
                if invLen2 <= 0
                    t = 0;
                    projectedPoint = p0;
                else
                    t = dot(queryPoint - p0, v) * invLen2;
                    t = max(0, min(1, t));
                    projectedPoint = p0 + t * v;
                end

                dist2 = sum((queryPoint - projectedPoint).^2);
                if dist2 < bestDist2
                    bestDist2 = dist2;
                    bestIdx = segIdx;
                    bestT = t;
                    bestPoint = projectedPoint;
                end
            end

            if hasSegmentCache
                segmentLength = trackData.segmentLengths(bestIdx);
            else
                segmentLength = trackData.arcLen(bestIdx + 1) - trackData.arcLen(bestIdx);
            end
            refS = trackData.arcLen(bestIdx) + bestT * max(segmentLength, 0);
            refS = max(0, min(trackData.length, refS));

            if segmentLength > eps
                if hasSegmentCache
                    v = trackData.segmentVectors(bestIdx, :);
                else
                    p0 = trackData.points(bestIdx, :);
                    p1 = trackData.points(bestIdx + 1, :);
                    v = p1 - p0;
                end
                refHeading = atan2(v(2), v(1));
            else
                refHeading = trackData.heading(bestIdx);
            end

            interpIdx = min(bestIdx + 1, trackData.nPts);
            refCurvature = (1 - bestT) * trackData.curvature(bestIdx) + ...
                bestT * trackData.curvature(interpIdx);
            refMu = (1 - bestT) * trackData.mu(bestIdx) + ...
                bestT * trackData.mu(interpIdx);

            dx = x - bestPoint(1);
            dy = y - bestPoint(2);
            lateralError = dx * (-sin(refHeading)) + dy * cos(refHeading);
            refIdx = min(bestIdx + double(bestT > 0.5), trackData.nPts);
            trackHalfWidth = trackData.trackHalfWidth;
            trackLimitMargin = trackHalfWidth - abs(lateralError);
            onTrack = trackLimitMargin >= -1e-9;

            ref = struct( ...
                'idx', refIdx, ...
                's', refS, ...
                'x', bestPoint(1), ...
                'y', bestPoint(2), ...
                'heading', refHeading, ...
                'curvature', refCurvature, ...
                'mu', refMu, ...
                'lateralError', lateralError, ...
                'trackWidth', trackData.trackWidth, ...
                'trackHalfWidth', trackHalfWidth, ...
                'trackLimitMargin', trackLimitMargin, ...
                'onTrack', onTrack);
        end

        function tireData = updatePlanarTireForces(obj, state, cornerLoads, dt, computePeakMu)
            % UPDATEPLANARTIREFORCES Evaluate tire forces and assemble body
            % forces / yaw moment from all four corners.
            %   tireData = updatePlanarTireForces(state, cornerLoads)
            %   tireData = updatePlanarTireForces(state, cornerLoads, dt)
            %
            %   dt enables the tire relaxation layer. When this is called
            %   repeatedly inside the wheel-contact solve, pass dt only on
            %   the final call so the contact-patch lag advances once per
            %   physics step (not once per solve iteration).
            if nargin < 4
                dt = obj.dt;
            end
            if nargin < 5 || isempty(computePeakMu)
                computePeakMu = true;
            end
            vm = obj.vehicleManager;
            kin = obj.getCornerKinematics(state.steer);
            corners = {'FL', 'FR', 'RL', 'RR'};
            nC = numel(corners);

            % Fixed-size column vectors (FL, FR, RL, RR) instead of
            % dynamically-grown structs with sprintf field names.
            slipAngles  = zeros(nC, 1);
            slipRatios  = zeros(nC, 1);
            longSpeeds  = zeros(nC, 1);
            wheelHeadings = zeros(nC, 1);
            cosWh = zeros(nC, 1);
            sinWh = zeros(nC, 1);

            for i = 1:nC
                corner = corners{i};
                cornerKin = kin.(corner);

                vxCorner = state.vx - state.yawRate * cornerKin.yPosition;
                vyCorner = state.vy + state.yawRate * cornerKin.xPosition;
                wh = cornerKin.steerAngle + cornerKin.toeAngle;

                cwh = cos(wh); swh = sin(wh);
                longSpeed = vxCorner * cwh + vyCorner * swh;
                latSpeed  = -vxCorner * swh + vyCorner * cwh;
                alpha = atan2(-latSpeed, max(abs(longSpeed), 0.1));
                tireState = vm.tire.(corner);
                kappa = obj.computeLocalSlipRatio(tireState, longSpeed);

                slipAngles(i)    = alpha;
                slipRatios(i)    = kappa;
                longSpeeds(i)    = longSpeed;
                wheelHeadings(i) = wh;
                cosWh(i) = cwh;
                sinWh(i) = swh;
            end

            % Per-corner contact-patch longitudinal speeds feed the tire
            % relaxation length (transient slip lag).
            longSpeedVec = longSpeeds;
            surfaceMu = obj.getStateSurfaceMu(state);

            if isempty(obj.cachedTireHasBatchUpdate)
                obj.cachedTireHasBatchUpdate = ismethod(vm.tire, 'updateAllCorners');
            end
            if obj.cachedTireHasBatchUpdate
                vm.tire.updateAllCorners( ...
                    cornerLoads.FL, cornerLoads.FR, cornerLoads.RL, cornerLoads.RR, ...
                    slipAngles(1), slipAngles(2), slipAngles(3), slipAngles(4), ...
                    slipRatios(1), slipRatios(2), slipRatios(3), slipRatios(4), ...
                    kin.FL.camberAngle, kin.FR.camberAngle, ...
                    kin.RL.camberAngle, kin.RR.camberAngle, dt, longSpeedVec, ...
                    surfaceMu, computePeakMu);
            else
                % Fallback for tire models without a batch update: evaluate
                % each corner individually. Wheel omega is integrated in the
                % main step() wheel-contact solve, not here.
                for i = 1:nC
                    corner = corners{i};
                    tireState = vm.tire.(corner);
                    cornerKin = kin.(corner);
                    vm.tire.updateCorner(tireState, cornerLoads.(corner), ...
                        slipAngles(i), slipRatios(i), ...
                        cornerKin.camberAngle, surfaceMu, dt, longSpeeds(i), ...
                        computePeakMu);
                end
            end

            sumFxBody = 0;
            sumFyBody = 0;
            yawMoment = 0;
            for i = 1:nC
                corner = corners{i};
                tireState = vm.tire.(corner);
                cornerKin = kin.(corner);

                FxBody = tireState.Fx * cosWh(i) - tireState.Fy * sinWh(i);
                FyBody = tireState.Fx * sinWh(i) + tireState.Fy * cosWh(i);

                sumFxBody = sumFxBody + FxBody;
                sumFyBody = sumFyBody + FyBody;
                yawMoment = yawMoment + ...
                    cornerKin.xPosition * FyBody - cornerKin.yPosition * FxBody;
            end
            tireData.sumFxBody = sumFxBody;
            tireData.sumFyBody = sumFyBody;
            tireData.yawMoment = yawMoment;
        end

        function tf = hasChassis(obj)
            % HASCHASSIS True when a sprung-mass attitude model is attached.
            %   The chassis owns heave/pitch/roll and imposes sprung-mass
            %   motion on the suspension corners; without it, corner loads
            %   fall back to the suspension's algebraic load-transfer path.
            %   Memoized: component wiring is a run invariant.
            if ~isempty(obj.cachedHasChassis)
                tf = obj.cachedHasChassis;
                return;
            end
            vm = obj.vehicleManager;
            tf = ~isempty(vm.chassis) && ...
                isa(vm.chassis, 'components.Chassis.ChassisComponent');
            obj.cachedHasChassis = tf;
        end

        function loads = getCurrentCornerLoads(obj, steer)
            % GETCURRENTCORNERLOADS Chassis-driven per-corner tire loads.
            %   loads = getCurrentCornerLoads(steer)
            %
            %   Thin wrapper over SuspensionManager.computeCornerLoadsFromChassis:
            %   reads the sprung-mass motion the chassis has resolved at each
            %   suspension pickup and returns the four tire normal forces.
            vm = obj.vehicleManager;
            loads = vm.suspension.computeCornerLoadsFromChassis( ...
                vm.chassis, steer, obj.dt);
        end

        function dynamics = computePlanarDynamics(obj, state, tireData, F_drag)
            % Rolling resistance is applied as a per-wheel resistance torque
            % (see PacejkaTire.updateWheelDynamics), which feeds back through
            % the tire Fx — it is NOT applied here as a body force (that would
            % double-count it).
            vm = obj.vehicleManager;
            % state.speed is maintained as hypot(vx, vy) by
            % updateFromPlanarDynamics; reuse it instead of recomputing sqrt.
            speed = state.speed;

            % Drag opposes the velocity vector (true aerodynamic behavior,
            % including a sideslip component).
            if speed > 0.1
                velocityDirX = state.vx / speed;
                velocityDirY = state.vy / speed;
            else
                velocityDirX = 1;
                velocityDirY = 0;
            end

            netFx = tireData.sumFxBody ...
                - F_drag * velocityDirX;
            netFy = tireData.sumFyBody ...
                - F_drag * velocityDirY;

            dynamics.ax = netFx / vm.totalMass;
            dynamics.ay = netFy / vm.totalMass;
            dynamics.yawAccel = tireData.yawMoment / max(vm.yawInertia, eps);
        end

        function moments = computeAeroPitchMoments(obj, aeroForces)
            % COMPUTEAEROPITCHMOMENTS Resolve aero forces into pitch moments.
            %   moments = computeAeroPitchMoments(aeroForces)
            %
            %   Returns a struct with the downforce and drag pitch moments
            %   about the CG (positive = nose-up) and the drag resultant
            %   height. Mirrors the moment bookkeeping in
            %   SimpleChassis.updateFromAccelerations so the two agree when
            %   no chassis is attached.
            if isempty(aeroForces)
                aeroForces = struct('Fz_front', 0, 'Fz_rear', 0, ...
                    'F_drag', 0, 'dragHeight', 0);
            end
            vm = obj.vehicleManager;
            FzFront = localGetField(aeroForces, 'Fz_front', 0);
            FzRear  = localGetField(aeroForces, 'Fz_rear', 0);
            Fdrag   = localGetField(aeroForces, 'F_drag', 0);
            dragHeight = localGetField(aeroForces, 'dragHeight', 0);

            if isnan(obj.cachedFrontArm)
                obj.cachedFrontArm = vm.wheelbase * (1 - vm.staticFrontWeight);
                obj.cachedRearArm  = vm.wheelbase * vm.staticFrontWeight;
            end
            frontArm = obj.cachedFrontArm;
            rearArm  = obj.cachedRearArm;
            moments.dragHeight = dragHeight;
            moments.downforce  = FzRear * rearArm - FzFront * frontArm;
            moments.drag       = Fdrag * dragHeight;
            moments.total      = moments.downforce + moments.drag;
        end

        function kin = getCornerKinematics(obj, steer)
            vm = obj.vehicleManager;
            if isempty(obj.cachedSuspensionHasKinematics)
                obj.cachedSuspensionHasKinematics = ~isempty(vm.suspension) && ...
                    ismethod(vm.suspension, 'getCornerKinematics');
            end
            if obj.cachedSuspensionHasKinematics
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

        function trackData = precomputeTrackSegments(~, trackData)
            % PRECOMPUTETRACKSEGMENTS Cache segment geometry used by projection.
            if ~isfield(trackData, 'points') || size(trackData.points, 1) < 2
                trackData.segmentVectors = zeros(0, 2);
                trackData.segmentLengths = zeros(0, 1);
                trackData.segmentInvLen2 = zeros(0, 1);
                return;
            end
            segmentVectors = diff(trackData.points, 1, 1);
            segmentLengths = hypot(segmentVectors(:,1), segmentVectors(:,2));
            len2 = segmentLengths.^2;
            segmentInvLen2 = zeros(size(len2));
            valid = len2 > eps;
            segmentInvLen2(valid) = 1 ./ len2(valid);
            trackData.segmentVectors = segmentVectors;
            trackData.segmentLengths = segmentLengths;
            trackData.segmentInvLen2 = segmentInvLen2;
        end

        function longSpeeds = computeCornerLongitudinalSpeeds(obj, state)
            kin = obj.getCornerKinematics(state.steer);
            corners = {'FL', 'FR', 'RL', 'RR'};
            for i = 1:numel(corners)
                corner = corners{i};
                cornerKin = kin.(corner);
                vxCorner = state.vx - state.yawRate * cornerKin.yPosition;
                vyCorner = state.vy + state.yawRate * cornerKin.xPosition;
                wh = cornerKin.steerAngle + cornerKin.toeAngle;
                longSpeeds.(corner) = vxCorner * cos(wh) + vyCorner * sin(wh);
            end
        end

        function surfaceMu = getStateSurfaceMu(~, state)
            surfaceMu = 1.2;
            hasMu = (isobject(state) && isprop(state, 'mu')) || ...
                (isstruct(state) && isfield(state, 'mu'));
            if hasMu && ~isempty(state.mu) && isfinite(state.mu)
                surfaceMu = state.mu;
            end
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
            slipSpeedFloor = 1.0;
            rawKappa = (wheelSpeed - longitudinalSpeed) / max(denom, slipSpeedFloor);

            if denom < slipSpeedFloor
                previousKappa = cornerState.slipRatio;
                if ~isfinite(previousKappa)
                    previousKappa = rawKappa;
                end
                blend = denom / slipSpeedFloor;
                kappa = (1 - blend) * previousKappa + blend * rawKappa;
            else
                kappa = rawKappa;
            end
            kappa = max(-1, min(1, kappa));
        end

        function inertia = getWheelInertia(obj)
            % GETWHEELINERTIA Per-wheel rotational inertia [kg*m^2].
            %   Returns a struct with FL/FR/RL/RR. The driven (rear) axle
            %   includes the motor rotor inertia reflected through ratio^2 so
            %   the motor mass is felt at the contact patch during launch and
            %   wheelspin transients. Front (undriven) wheels use the bare
            %   wheel+tire inertia. Memoized: inertias are run invariants.
            if ~isempty(obj.cachedWheelInertia)
                inertia = obj.cachedWheelInertia;
                return;
            end
            vm = obj.vehicleManager;
            baseI = 0.5;
            if ~isempty(vm.tire) && isprop(vm.tire, 'wheelInertia')
                baseI = vm.tire.wheelInertia;
            end
            % Reflect motor rotor inertia to the driven axle (RWD: rear).
            drivenI = baseI;
            if ~isempty(vm.powertrain) && ...
                    isa(vm.powertrain, 'components.Powertrain.PowertrainComponent') && ...
                    ismethod(vm.powertrain, 'getReflectedRotorInertia')
                drivenI = baseI + 0.5 * vm.powertrain.getReflectedRotorInertia();
            end
            inertia = struct('FL', baseI, 'FR', baseI, 'RL', drivenI, 'RR', drivenI);
            obj.cachedWheelInertia = inertia;
        end

        function out = solveDifferential(obj, totalWheelTorque, omegaL, omegaR, wheelInertia, dt)
            % SOLVEDIFFERENTIAL Split driven-axle torque via the configured
            % differential. Falls back to the legacy open behavior (50/50
            % torque, mean-speed carrier) when no differential is present.
            vm = obj.vehicleManager;
            if ~isempty(vm.differential) && ...
                    isa(vm.differential, 'components.Powertrain.DifferentialComponent')
                out = vm.differential.solveDrive( ...
                    totalWheelTorque, omegaL, omegaR, wheelInertia, dt);
            else
                totalWheelTorque = max(0, totalWheelTorque);
                out.TL = 0.5 * totalWheelTorque;
                out.TR = 0.5 * totalWheelTorque;
                out.carrierOmega = 0.5 * (omegaL + omegaR);
            end
        end

        function locked = differentialLocksWheels(obj)
            % DIFFERENTIALLOCKSWHEELS True if the diff forces equal wheel speed.
            %   Memoized: the differential type is a run invariant.
            if ~isempty(obj.cachedDiffLocksWheels)
                locked = obj.cachedDiffLocksWheels;
                return;
            end
            vm = obj.vehicleManager;
            locked = false;
            if ~isempty(vm.differential) && ...
                    isa(vm.differential, 'components.Powertrain.DifferentialComponent') && ...
                    ismethod(vm.differential, 'locksWheels')
                locked = vm.differential.locksWheels();
            end
            obj.cachedDiffLocksWheels = locked;
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

function value = localGetField(s, fieldName, defaultValue)
% LOCALGETFIELD Struct field access with a default, tolerant of non-structs.
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
    if isempty(value)
        value = defaultValue;
    end
else
    value = defaultValue;
end
end
