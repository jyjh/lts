classdef Simulator < handle
    % SIMULATOR Physics engine and simulation loop for vehicle dynamics
    %
    % Core concept: given a lts.simulation.VehicleState and driver inputs, progress the
    % state from one timestep to the next (copy-in → copy-out).
    %
    % Two modes of use:
    %   1. Single step:  [newState, forces] = sim.step(state, input, ref)
    %   2. Full lap:     [stateLog, lapTime] = sim.simulate(initialState, track)
    %
    % The lts.simulation.Simulator composes a lts.vehicle.VehicleManager and asks a driver/controller
    % for inputs during full-lap simulation. Vehicle dynamics remain in
    % step(); driver policy remains outside the physics transition.
    %
    % One step follows this physics chain:
    %   driver inputs -> aero loads/drag -> tire normal loads ->
    %   powertrain/diff/brake wheel torques -> wheel slip -> Pacejka tire
    %   forces -> body accelerations/yaw moment -> planar state integration.
    % Track geometry is a reference/progress signal only; the car is not
    % kinematically constrained to the centerline.
    
    properties
        % Reference to lts.vehicle.VehicleManager (components + vehicle parameters)
        vehicleManager
        
        % Reference to lts.driver.DriverModel/controller (computes driver inputs)
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

        % Telemetry logging mode. "full" preserves the existing rich log;
        % "lean" records only core lap/state/control/aggregate force channels
        % for profiling and benchmark runs.
        telemetryMode = "full"

        % Input normalization policy. Normal driver models use mutually
        % exclusive pedals and a simulator-side steering slew limiter.
        % Correlation/replay runs can disable these so measured controls are
        % applied as recorded after channel-map scaling.
        enforcePedalExclusivity = true
        applySteeringSlew = true
        brakeMode = "ratio"
        powertrainMode = "throttle"
        stopOnOffTrack = true
        stopAtTrackEnd = true
        stopTime = inf
        referenceMode = "track"
        freeSurfaceMu = 1.2

        % Internal: track whether maxSpeed warning was issued (warn once)
        warnedMaxSpeed = false
    end

    properties (Transient = true) %#ok<MCNPC>
        % Lazily-cached run invariants. The lts.vehicle.VehicleManager and its components
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
            % SIMULATOR Construct with a lts.vehicle.VehicleManager and lts.driver.DriverModel
            %   lts.simulation.Simulator(vehicleManager, driverModel, dt)
            %   lts.simulation.Simulator(vehicleManager, driverModel)  % uses default dt = 0.001
            
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
            %
            % The method is intentionally force-first:
            %   1. estimate loads and wheel torques from the old state,
            %   2. solve wheel angular velocity and tire force together,
            %   3. sum body forces/moment,
            %   4. integrate vx/vy/yaw/x/y once.
            % This avoids the old "speed follows track curvature" shortcut;
            % yaw and lateral position now emerge from the four contact patches.
            
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
                % Chassis-coupled path: use the sprung-mass attitude from the
                % previous completed step to impose corner motion on the
                % suspension. The chassis is then advanced later in this step
                % with the newly computed accelerations, giving a stable
                % one-step stagger between tire loads and platform attitude.
                cornerLoads = obj.getCurrentCornerLoads(steer);
            else
                % Algebraic fallback: without a chassis state, estimate static
                % + aero + longitudinal/lateral load transfer directly from
                % the current accelerations.
                cornerLoads = vm.suspension.estimateCornerLoads( ...
                    suspensionInputState, aeroForces.Fz_front, aeroForces.Fz_rear, vm.totalMass);
            end
            
            % --- POWERTRAIN STATE & DRIVE TORQUE ---
            % Motor speed samples the differential carrier (mean of the
            % driven wheels for an open/LSD diff, the common locked speed
            % for a spool). Falls back to the raw mean if no differential.
            carrierOmega0 = 0.5 * (vm.tire.RL.angularVelocity + vm.tire.RR.angularVelocity);
            vm.powertrain.updateStateFromDrivenWheels(carrierOmega0);
            [totalDriveTorque, totalCoastdownTorque] = ...
                obj.computePowertrainTorques(state, input, throttle);

            % --- WHEEL TORQUE SETUP ---
            % RWD assumption: drive torque only on rear wheels.
            % Brake distribution: fixed front/rear bias from lts.vehicle.VehicleManager.
            R = vm.tire.RL.wheelRadius;  % all corners share same radius
            T_drive_front = 0;
            % Rear drive torque is split by the differential model. If no
            % differential is configured, fall back to the legacy 50/50 open
            % behavior (mean-speed carrier).
            inertia = obj.getWheelInertia();  % struct FL/FR/RL/RR
            diffOut = obj.solveDifferential( ...
                totalDriveTorque, totalCoastdownTorque, ...
                vm.tire.RL.angularVelocity, vm.tire.RR.angularVelocity, ...
                inertia.RL, obj.dt);
            T_drive_RL = diffOut.TL;
            T_drive_RR = diffOut.TR;

            % --- BRAKE TORQUE ---
            % Ratio mode preserves the legacy brake_force_coefficient path.
            % Pressure mode converts logged front/rear line pressure to
            % independent axle brake forces via vehicle calibration.
            totalNormalLoad = W + F_downforce;
            brakeForces = obj.computeBrakeForces(input, totalNormalLoad);
            brakeCommand = brakeForces.requestedCommand;
            effectiveBrakeCommand = brakeForces.effectiveCommand;
            F_brake_front_cmd = brakeForces.frontForce;
            F_brake_rear_cmd = brakeForces.rearForce;
            
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
                    totalDriveTorque, totalCoastdownTorque, ...
                    vm.tire.RL.angularVelocity, vm.tire.RR.angularVelocity, ...
                    inertia.RL, obj.dt);
                T_drive_RL = diffOut.TL;
                T_drive_RR = diffOut.TR;

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
                    tireData = obj.updatePlanarTireForces( ...
                        tireInputState, cornerLoads, 0, false, 'steady');
                else
                    tireData = obj.updatePlanarTireForces( ...
                        tireInputState, cornerLoads, obj.dt, true, 'advance');
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
                correctedLoadState.frontAxleAy = dynamics.frontAxleAy;
                correctedLoadState.rearAxleAy = dynamics.rearAxleAy;
                cornerLoads = vm.suspension.computeCornerLoads( ...
                    correctedLoadState, aeroForces.Fz_front, aeroForces.Fz_rear, vm.totalMass, obj.dt);
                % Re-evaluate tire forces at the corrected loads with dt = 0: the
                % contact-patch relaxation already advanced once on the final
                % wheel-solve iteration above, so it must not advance again here.
                tireData = obj.updatePlanarTireForces( ...
                    tireInputState, cornerLoads, 0, true, 'hold');
                dynamics = obj.computePlanarDynamics(state, tireData, F_drag);
            end

            % Integrate the sprung-mass attitude (heave/pitch/roll) from the
            % converged body accelerations and aero forces. The chassis owns
            % pitch, roll, and ride-height, which feed next step's aero
            % (ground effect, pitch sensitivity) and are read back by
            % lts.simulation.VehicleState.computePitch/Roll/RideHeight below.
            if ~isempty(vm.chassis) && ...
                    isa(vm.chassis, 'lts.components.Chassis.ChassisComponent')
                vm.chassis.updateFromAccelerations( ...
                    dynamics.ax, dynamics.ay, aeroForces, obj.dt, dynamics.yawAccel);
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
            kinematics = obj.integratePlanarKinematics( ...
                state, dynamics, obj.dt);
            yawRateNew = kinematics.yawRate;
            yawNew = kinematics.yaw;
            vxNew = kinematics.vx;
            vyNew = kinematics.vy;
            xNew = kinematics.x;
            yNew = kinematics.y;

            if obj.isFreeReference(ref)
                % Free reference mode is used by correlation replay: progress
                % is just distance travelled in world space, with a fixed
                % surface mu, and no track-limit projection.
                dsFree = hypot(xNew - state.x, yNew - state.y);
                nextRef = obj.freeReferenceForState( ...
                    newState, state.s + dsFree, xNew, yNew, yawNew);
            else
                % Track reference mode projects the freely integrated x/y back
                % onto the centerline for progress, curvature/mu lookup, and
                % lateral-error telemetry. Projection does not overwrite x/y.
                nextRef = obj.projectToReference(xNew, yNew, ref.trackData, ref.idx);
            end
            
            newState.throttle = throttle;
            newState.brake = effectiveBrakeCommand;
            newState.steer = steer;
            newState = newState.updateFromPlanarDynamics( ...
                dynamics.ax, dynamics.ay, dynamics.yawAccel, ...
                vxNew, vyNew, yawRateNew, yawNew, xNew, yNew, ...
                nextRef.s, nextRef.heading, nextRef.curvature, ...
                nextRef.lateralError, obj.dt, nextRef.mu, ...
                dynamics.frontAxleAy, dynamics.rearAxleAy);
            newState.onTrack = nextRef.onTrack;
            
            % Sanity check: warn once if speed exceeds maxSpeed
            if newState.speed > vm.maxSpeed && ~obj.warnedMaxSpeed
                obj.warnedMaxSpeed = true;
                warning('lts_simulation_Simulator:SpeedExceeded', ...
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
            forces.brakePressureMode = brakeForces.pressureModeActive;
            forces.brakePressureFrontBar = brakeForces.frontPressureBar;
            forces.brakePressureRearBar = brakeForces.rearPressureBar;
            forces.brakeGrip_FL = max(vm.tire.FL.peakMu, 0) * max(cornerLoads.FL, 0);
            forces.brakeGrip_FR = max(vm.tire.FR.peakMu, 0) * max(cornerLoads.FR, 0);
            forces.brakeGrip_RL = max(vm.tire.RL.peakMu, 0) * max(cornerLoads.RL, 0);
            forces.brakeGrip_RR = max(vm.tire.RR.peakMu, 0) * max(cornerLoads.RR, 0);
            forces.driveTorqueTotal = totalDriveTorque;
            forces.coastdownTorqueTotal = totalCoastdownTorque;
            forces.driveTorque_RL = T_drive_RL;
            forces.driveTorque_RR = T_drive_RR;
            forces.brakeTorque_FL = T_brake_front;
            forces.brakeTorque_FR = T_brake_front;
            forces.brakeTorque_RL = T_brake_rear;
            forces.brakeTorque_RR = T_brake_rear;
            forces.motorRPM = 0;
            forces.motorTorque = 0;
            forces.motorTorqueRequested = 0;
            forces.motorTorquePowerLimitNm = NaN;
            forces.motorTorquePowerLimitActive = false;
            forces.wheelTorque = 0;
            forces.packVoltageV = NaN;
            forces.packCurrentA = NaN;
            forces.packPowerW = NaN;
            forces.drivenWheelRPM = 0;
            forces.rpmLimitActive = false;
            if ~isempty(vm.powertrain.state)
                forces.motorRPM = vm.powertrain.state.motorRPM;
                forces.motorTorque = vm.powertrain.state.motorTorque;
                forces.motorTorqueRequested = vm.powertrain.state.requestedMotorTorque;
                forces.motorTorquePowerLimitNm = vm.powertrain.state.motorTorquePowerLimitNm;
                forces.motorTorquePowerLimitActive = vm.powertrain.state.motorTorquePowerLimitActive;
                forces.wheelTorque = vm.powertrain.state.wheelTorque;
                forces.packVoltageV = vm.powertrain.state.packVoltageV;
                forces.packCurrentA = vm.powertrain.state.packCurrentA;
                forces.packPowerW = vm.powertrain.state.packPowerW;
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
            forces.frontAxleAy = dynamics.frontAxleAy;
            forces.rearAxleAy = dynamics.rearAxleAy;
            forces.rollResistance = F_rollResist;
        end
        
        function [stateLog, lapTime] = simulate(obj, initialState, track)
            % SIMULATE Run the full lap simulation
            %   [stateLog, lapTime] = simulate(initialState, track)
            %
            %   initialState - lts.simulation.VehicleState at simulation start
            %   track        - Track object with geometry and surface data
            %
            % This is orchestration rather than physics: it prepares the track
            % reference, initializes the vehicle/controller, repeats warmup
            % laps for closed circuits when requested, calls step(), and
            % records telemetry. The state transition itself lives in step().
            
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
                    error('lts_simulation_Simulator:WarmupRequiresClosedLoop', ...
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
            % trackData is the simulator's immutable reference bundle. The
            % mutable vehicle state stores only its current projection fields
            % (refS/refHeading/refCurvature/lateralError/mu/onTrack).
            trackData = obj.precomputeTrackSegments(trackData);
            initialState = obj.initializePlanarState(initialState, trackData, ...
                obj.referenceMode, obj.freeSurfaceMu);
            if ~isempty(obj.driverModel) && ...
                    ismethod(obj.driverModel, 'prepareForSimulation')
                obj.driverModel = obj.driverModel.prepareForSimulation( ...
                    initialState, trackData, obj.dt);
            end
            
            % Pre-allocate telemetry log
            maxSteps = round(trackLen / (max(initialState.speed, 5) * obj.dt) * 5);
            if isfinite(obj.stopTime)
                maxSteps = max(maxSteps, ceil(max(0, obj.stopTime - initialState.time) / obj.dt) + 2);
            end
            maxSteps = max(maxSteps, 100000);
            leanTelemetry = obj.isLeanTelemetry();
            if leanTelemetry
                stateLog = obj.createLeanStateLog(maxSteps);
            else
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
                'frontAxleAy', zeros(maxSteps, 1), ...
                'rearAxleAy',  zeros(maxSteps, 1), ...
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
                'brakePressureMode', false(maxSteps, 1), ...
                'brakePressureFrontBar', NaN(maxSteps, 1), ...
                'brakePressureRearBar', NaN(maxSteps, 1), ...
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
                'motorTorqueRequested', zeros(maxSteps, 1), ...
                'motorTorquePowerLimitNm', NaN(maxSteps, 1), ...
                'motorTorquePowerLimitActive', false(maxSteps, 1), ...
                'wheelTorque', zeros(maxSteps, 1), ...
                'packVoltageV', NaN(maxSteps, 1), ...
                'packCurrentA', NaN(maxSteps, 1), ...
                'packPowerW', NaN(maxSteps, 1), ...
                'drivenWheelRPM', zeros(maxSteps, 1), ...
                'rpmLimitActive', false(maxSteps, 1), ...
                'pitchAngle',  zeros(maxSteps, 1), ...
                'rollAngle',   zeros(maxSteps, 1), ...
                'rollRate',    zeros(maxSteps, 1), ...
                'frontRollAngle', zeros(maxSteps, 1), ...
                'rearRollAngle',  zeros(maxSteps, 1), ...
                'frontRollRate',  zeros(maxSteps, 1), ...
                'rearRollRate',   zeros(maxSteps, 1), ...
                'twistAngle',     zeros(maxSteps, 1), ...
                'twistRate',      zeros(maxSteps, 1), ...
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
                'slipAngle_FL', zeros(maxSteps, 1), ...
                'slipAngle_FR', zeros(maxSteps, 1), ...
                'slipAngle_RL', zeros(maxSteps, 1), ...
                'slipAngle_RR', zeros(maxSteps, 1), ...
                'slipRatio_FL', zeros(maxSteps, 1), ...
                'slipRatio_FR', zeros(maxSteps, 1), ...
                'slipRatio_RL', zeros(maxSteps, 1), ...
                'slipRatio_RR', zeros(maxSteps, 1), ...
                'peakMu_FL', zeros(maxSteps, 1), ...
                'peakMu_FR', zeros(maxSteps, 1), ...
                'peakMu_RL', zeros(maxSteps, 1), ...
                'peakMu_RR', zeros(maxSteps, 1), ...
                'tireUtilization_FL', zeros(maxSteps, 1), ...
                'tireUtilization_FR', zeros(maxSteps, 1), ...
                'tireUtilization_RL', zeros(maxSteps, 1), ...
                'tireUtilization_RR', zeros(maxSteps, 1), ...
                'omega_FL',     zeros(maxSteps, 1), ...
                'omega_FR',     zeros(maxSteps, 1), ...
                'omega_RL',     zeros(maxSteps, 1), ...
                'omega_RR',     zeros(maxSteps, 1), ...
                'tireSpeed_FL', zeros(maxSteps, 1), ...
                'tireSpeed_FR', zeros(maxSteps, 1), ...
                'tireSpeed_RL', zeros(maxSteps, 1), ...
                'tireSpeed_RR', zeros(maxSteps, 1), ...
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
            end
            
            % Working state (will be updated each step)
            currentState = initialState;
            obj.initializeWheelSpeeds(currentState.speed);
            if obj.isFreeReferenceMode()
                currentRef = obj.freeReferenceForState(currentState, ...
                    currentState.s, currentState.x, currentState.y, currentState.yaw);
            else
                currentRef = obj.projectToReference(currentState.x, ...
                    currentState.y, trackData, 1);
            end
            
            step = 0;
            fprintf('Starting simulation...\n');
            fprintf('Track length: %.1f m\n', trackLen);
            if warmupLaps > 0
                fprintf('Telemetry: dropping %d warmup lap(s), recording %d lap(s)\n', ...
                    warmupLaps, recordedLaps);
            end
            
            finishTolerance = 1e-6;
            while obj.shouldContinueSimulation(currentState, trackLen, finishTolerance)
                if obj.isFreeReferenceMode()
                    currentRef = obj.freeReferenceForState(currentState, ...
                        currentState.s, currentState.x, currentState.y, currentState.yaw);
                else
                    currentRef = obj.projectToReference( ...
                        currentState.x, currentState.y, trackData, currentRef.idx);
                end
                currentState.s = currentRef.s;
                currentState.refS = currentRef.s;
                currentState.refHeading = currentRef.heading;
                currentState.refCurvature = currentRef.curvature;
                currentState.curvature = currentRef.curvature;
                currentState.lateralError = currentRef.lateralError;
                currentState.mu = currentRef.mu;
                currentState.onTrack = currentRef.onTrack;
                if ~obj.shouldContinueSimulation(currentState, trackLen, finishTolerance)
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
                    stateLog.controlS(step)    = localGetField(input, ...
                        'sourceDistance', currentState.s);
                    stateLog.x(step)           = newState.x;
                    stateLog.y(step)           = newState.y;
                    stateLog.yaw(step)         = newState.yaw;
                    stateLog.vx(step)          = newState.vx;
                    stateLog.vy(step)          = newState.vy;
                    stateLog.bodySlipAngle(step) = newState.bodySlipAngle;
                    stateLog.speed(step)       = newState.speed;
                    stateLog.speedKmh(step)    = newState.speed * 3.6;
                    stateLog.controlTime(step) = localGetField(input, ...
                        'sourceTime', currentState.time);
                    stateLog.ax(step)          = newState.ax;
                    stateLog.ay(step)          = newState.ay;
                    stateLog.frontAxleAy(step) = newState.frontAxleAy;
                    stateLog.rearAxleAy(step)  = newState.rearAxleAy;
                    stateLog.yawRate(step)     = newState.yawRate;
                    stateLog.yawAccel(step)    = newState.yawAccel;
                    stateLog.refS(step)        = newState.refS;
                    stateLog.refHeading(step)  = newState.refHeading;
                    stateLog.refCurvature(step) = newState.refCurvature;
                    stateLog.lateralError(step) = newState.lateralError;
                    stateLog.onTrack(step)     = newState.onTrack;
                    if obj.isFreeReferenceMode()
                        stateLog.trackWidth(step) = 0;
                        stateLog.trackLimitMargin(step) = 0;
                    else
                        stateLog.trackWidth(step)  = trackData.trackWidth;
                        stateLog.trackLimitMargin(step) = ...
                            trackData.trackHalfWidth - abs(newState.lateralError);
                    end
                    stateLog.throttle(step)    = input.throttle;
                    stateLog.brake(step)       = forces.brake;
                    stateLog.brakeRequested(step) = forces.brakeCommand;
                    stateLog.brakePressureMode(step) = forces.brakePressureMode;
                    stateLog.brakePressureFrontBar(step) = forces.brakePressureFrontBar;
                    stateLog.brakePressureRearBar(step) = forces.brakePressureRearBar;
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
                    stateLog.motorTorqueRequested(step) = forces.motorTorqueRequested;
                    stateLog.motorTorquePowerLimitNm(step) = forces.motorTorquePowerLimitNm;
                    stateLog.motorTorquePowerLimitActive(step) = forces.motorTorquePowerLimitActive;
                    stateLog.wheelTorque(step) = forces.wheelTorque;
                    stateLog.packVoltageV(step) = forces.packVoltageV;
                    stateLog.packCurrentA(step) = forces.packCurrentA;
                    stateLog.packPowerW(step) = forces.packPowerW;
                    stateLog.drivenWheelRPM(step) = forces.drivenWheelRPM;
                    stateLog.rpmLimitActive(step) = forces.rpmLimitActive;
                    stateLog.pitchAngle(step)  = newState.pitchAngle;
                    stateLog.rollAngle(step)   = newState.rollAngle;
                    stateLog.rollRate(step)    = newState.rollRate;
                    stateLog.frontRollAngle(step) = newState.frontRollAngle;
                    stateLog.rearRollAngle(step)  = newState.rearRollAngle;
                    stateLog.frontRollRate(step)  = newState.frontRollRate;
                    stateLog.rearRollRate(step)   = newState.rearRollRate;
                    stateLog.twistAngle(step)     = newState.twistAngle;
                    stateLog.twistRate(step)      = newState.twistRate;
                    stateLog.rideHeight(step)  = newState.rideHeight;
                    stateLog.aeroFz_front(step) = forces.aeroFz_front;
                    stateLog.aeroFz_rear(step)  = forces.aeroFz_rear;
                    
                    if ~leanTelemetry
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
                        stateLog.slipAngle_FL(step) = vm.tire.FL.slipAngle;
                        stateLog.slipAngle_FR(step) = vm.tire.FR.slipAngle;
                        stateLog.slipAngle_RL(step) = vm.tire.RL.slipAngle;
                        stateLog.slipAngle_RR(step) = vm.tire.RR.slipAngle;
                        stateLog.slipRatio_FL(step) = vm.tire.FL.slipRatio;
                        stateLog.slipRatio_FR(step) = vm.tire.FR.slipRatio;
                        stateLog.slipRatio_RL(step) = vm.tire.RL.slipRatio;
                        stateLog.slipRatio_RR(step) = vm.tire.RR.slipRatio;
                        stateLog.peakMu_FL(step) = vm.tire.FL.peakMu;
                        stateLog.peakMu_FR(step) = vm.tire.FR.peakMu;
                        stateLog.peakMu_RL(step) = vm.tire.RL.peakMu;
                        stateLog.peakMu_RR(step) = vm.tire.RR.peakMu;
                        stateLog.tireUtilization_FL(step) = ...
                            obj.computeTireUtilization(vm.tire.FL);
                        stateLog.tireUtilization_FR(step) = ...
                            obj.computeTireUtilization(vm.tire.FR);
                        stateLog.tireUtilization_RL(step) = ...
                            obj.computeTireUtilization(vm.tire.RL);
                        stateLog.tireUtilization_RR(step) = ...
                            obj.computeTireUtilization(vm.tire.RR);
                        stateLog.omega_FL(step)     = vm.tire.FL.angularVelocity;
                        stateLog.omega_FR(step)     = vm.tire.FR.angularVelocity;
                        stateLog.omega_RL(step)     = vm.tire.RL.angularVelocity;
                        stateLog.omega_RR(step)     = vm.tire.RR.angularVelocity;
                        stateLog.tireSpeed_FL(step) = vm.tire.FL.angularVelocity * vm.tire.FL.wheelRadius;
                        stateLog.tireSpeed_FR(step) = vm.tire.FR.angularVelocity * vm.tire.FR.wheelRadius;
                        stateLog.tireSpeed_RL(step) = vm.tire.RL.angularVelocity * vm.tire.RL.wheelRadius;
                        stateLog.tireSpeed_RR(step) = vm.tire.RR.angularVelocity * vm.tire.RR.wheelRadius;
                        stateLog.tireFx_FL(step)    = vm.tire.FL.Fx;
                        stateLog.tireFx_FR(step)    = vm.tire.FR.Fx;
                        stateLog.tireFx_RL(step)    = vm.tire.RL.Fx;
                        stateLog.tireFx_RR(step)    = vm.tire.RR.Fx;
                        stateLog.tireFy_FL(step)    = vm.tire.FL.Fy;
                        stateLog.tireFy_FR(step)    = vm.tire.FR.Fy;
                        stateLog.tireFy_RL(step)    = vm.tire.RL.Fy;
                        stateLog.tireFy_RR(step)    = vm.tire.RR.Fy;
                    end
                end
                
                % Advance state
                currentState = newState;
                
                % Progress display
                if mod(step, 5000) == 0
                    progress = obj.simulationProgress(currentState, input, trackLen);
                    fprintf('  Progress: %5.1f%% | Speed: %5.1f km/h | s: %6.1f m | replay: %s\n', ...
                        progress * 100, currentState.speed * 3.6, ...
                        currentState.s, obj.replayProgressText(input));
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

            if obj.isFreeReferenceMode()
                recordedSteps = step;
                if recordedSteps > 0
                    lapTime = stateLog.time(end);
                else
                    lapTime = 0;
                end
            else
                [stateLog, lapTime, recordedSteps] = obj.applyTelemetryLapWindow( ...
                    stateLog, recordStartS, recordEndS);
            end
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

        function [stateLog, lapTime] = simulateReplay(obj, initialState, track, replayProfile, varargin)
            % SIMULATEREPLAY Run a lap with externally supplied controls.
            %   [stateLog, lapTime] = simulateReplay(initialState, track, replayProfile)
            %
            %   replayProfile may be a lts.correlation.CorrelationReplayProfile object or a
            %   normalized replay CSV path accepted by lts.correlation.CorrelationReplayProfile.
            parser = inputParser;
            parser.addParameter('ReplayDomain', 'distance', @(x) ischar(x) || isstring(x));
            parser.addParameter('AllowPedalOverlap', true, @(x) islogical(x) || isnumeric(x));
            parser.addParameter('ApplySteeringSlew', false, @(x) islogical(x) || isnumeric(x));
            parser.addParameter('BrakeMode', 'ratio', @(x) ischar(x) || isstring(x));
            parser.addParameter('PowertrainMode', 'throttle', @(x) ischar(x) || isstring(x));
            parser.addParameter('StopOnOffTrack', false, @(x) islogical(x) || isnumeric(x));
            parser.addParameter('StopAtTrackEnd', false, @(x) islogical(x) || isnumeric(x));
            parser.addParameter('StopAtReplayEnd', true, @(x) islogical(x) || isnumeric(x));
            parser.addParameter('ReferenceMode', 'track', @(x) ischar(x) || isstring(x));
            parser.addParameter('SurfaceMu', NaN, @(x) isnumeric(x) && isscalar(x));
            parser.parse(varargin{:});

            if isa(replayProfile, 'lts.correlation.CorrelationReplayProfile')
                profile = replayProfile;
            else
                profile = lts.correlation.CorrelationReplayProfile.fromCsv(replayProfile);
            end

            previousDriver = obj.driverModel;
            previousMethod = obj.cachedDriverInputMethod;
            previousPedalPolicy = obj.enforcePedalExclusivity;
            previousSteerPolicy = obj.applySteeringSlew;
            previousBrakeMode = obj.brakeMode;
            previousPowertrainMode = obj.powertrainMode;
            previousOffTrackPolicy = obj.stopOnOffTrack;
            previousTrackEndPolicy = obj.stopAtTrackEnd;
            previousStopTime = obj.stopTime;
            previousReferenceMode = obj.referenceMode;
            previousFreeSurfaceMu = obj.freeSurfaceMu;
            cleanup = onCleanup(@() obj.restoreReplayPolicies( ...
                previousDriver, previousMethod, previousPedalPolicy, ...
                previousSteerPolicy, previousBrakeMode, previousOffTrackPolicy, ...
                previousTrackEndPolicy, previousStopTime, ...
                previousReferenceMode, previousFreeSurfaceMu, previousPowertrainMode));

            obj.driverModel = lts.correlation.TelemetryReplayDriver(profile, ...
                'ReplayDomain', parser.Results.ReplayDomain);
            obj.cachedDriverInputMethod = [];
            obj.enforcePedalExclusivity = ~logical(parser.Results.AllowPedalOverlap);
            obj.applySteeringSlew = logical(parser.Results.ApplySteeringSlew);
            obj.brakeMode = obj.validateBrakeMode(parser.Results.BrakeMode);
            obj.powertrainMode = obj.validatePowertrainMode(parser.Results.PowertrainMode);
            if obj.powertrainMode == "motor_torque_command" && ...
                    ~profile.hasMotorTorqueCommand()
                error('lts_simulation_Simulator:MissingMotorTorqueCommand', ...
                    ['PowertrainMode "motor_torque_command" requires ' ...
                    'motor_torque_command_nm in the replay profile.']);
            end
            obj.stopOnOffTrack = logical(parser.Results.StopOnOffTrack);
            obj.stopAtTrackEnd = logical(parser.Results.StopAtTrackEnd);
            obj.referenceMode = lower(string(parser.Results.ReferenceMode));
            if obj.referenceMode ~= "track" && obj.referenceMode ~= "free"
                error('lts_simulation_Simulator:InvalidReferenceMode', ...
                    'ReferenceMode must be "track" or "free".');
            end
            if isfinite(parser.Results.SurfaceMu) && parser.Results.SurfaceMu > 0
                obj.freeSurfaceMu = double(parser.Results.SurfaceMu);
            end
            if logical(parser.Results.StopAtReplayEnd)
                obj.stopTime = profile.duration();
            else
                obj.stopTime = inf;
            end

            [stateLog, lapTime] = obj.simulate(initialState, track);
            stateLog = obj.addReplayReferenceChannels(stateLog, profile);
        end

        function stateLog = addReplayReferenceChannels(~, stateLog, profile)
            if isempty(stateLog.time)
                return;
            end

            if isfield(stateLog, 'controlTime')
                queryTime = stateLog.controlTime(:);
            else
                queryTime = stateLog.time(:);
            end

            stateLog.replayThrottle = localInterpProfileChannel( ...
                profile.time, profile.throttle, queryTime);
            stateLog.replayBrake = localInterpProfileChannel( ...
                profile.time, profile.brake, queryTime);
            if profile.hasBrakePressure()
                stateLog.replayBrakePressureFrontBar = localInterpProfileChannel( ...
                    profile.time, profile.brakePressureFrontBar, queryTime);
                stateLog.replayBrakePressureRearBar = localInterpProfileChannel( ...
                    profile.time, profile.brakePressureRearBar, queryTime);
            end
            if profile.hasRegenTorque()
                stateLog.replayRegenTorqueNm = localInterpProfileChannel( ...
                    profile.time, profile.regenTorqueNm, queryTime);
            end
            if profile.hasMotorTorqueCommand()
                stateLog.replayMotorTorqueCommandNm = localInterpProfileChannel( ...
                    profile.time, profile.motorTorqueCommandNm, queryTime);
            end
            if profile.hasMotorRpm()
                stateLog.replayMotorRpm = localInterpProfileChannel( ...
                    profile.time, profile.motorRpm, queryTime);
            end
            if profile.hasPackPower()
                stateLog.replayPackVoltageV = localInterpProfileChannel( ...
                    profile.time, profile.packVoltageV, queryTime);
                stateLog.replayPackCurrentA = localInterpProfileChannel( ...
                    profile.time, profile.packCurrentA, queryTime);
                stateLog.replayPackPowerW = ...
                    stateLog.replayPackVoltageV .* stateLog.replayPackCurrentA;
            end
            stateLog.replaySteer = localInterpProfileChannel( ...
                profile.time, profile.steer, queryTime);
            stateLog.replaySpeed = localInterpProfileChannel( ...
                profile.time, profile.speed, queryTime);
            if profile.hasLatAccel()
                stateLog.replayLatAccelG = localInterpProfileChannel( ...
                    profile.time, profile.latAccelG, queryTime);
                stateLog.replayFrontLatAccelG = localInterpProfileChannel( ...
                    profile.time, profile.frontLatAccelG, queryTime);
                stateLog.replayRearLatAccelG = localInterpProfileChannel( ...
                    profile.time, profile.rearLatAccelG, queryTime);
            end
            if profile.hasLongAccel()
                stateLog.replayLongAccelG = localInterpProfileChannel( ...
                    profile.time, profile.longAccelG, queryTime);
                stateLog.replayFrontLongAccelG = localInterpProfileChannel( ...
                    profile.time, profile.frontLongAccelG, queryTime);
                stateLog.replayRearLongAccelG = localInterpProfileChannel( ...
                    profile.time, profile.rearLongAccelG, queryTime);
            end
            stateLog.replayYawRate = localInterpProfileChannel( ...
                profile.time, profile.yawRate, queryTime);
        end

        function restoreReplayPolicies(obj, driverModel, inputMethod, pedalPolicy, ...
                steerPolicy, brakeMode, offTrackPolicy, trackEndPolicy, stopTime, ...
                referenceMode, freeSurfaceMu, powertrainMode)
            obj.driverModel = driverModel;
            obj.cachedDriverInputMethod = inputMethod;
            obj.enforcePedalExclusivity = pedalPolicy;
            obj.applySteeringSlew = steerPolicy;
            obj.brakeMode = brakeMode;
            obj.stopOnOffTrack = offTrackPolicy;
            obj.stopAtTrackEnd = trackEndPolicy;
            obj.stopTime = stopTime;
            if nargin >= 10
                obj.referenceMode = referenceMode;
            end
            if nargin >= 11
                obj.freeSurfaceMu = freeSurfaceMu;
            end
            if nargin >= 12
                obj.powertrainMode = powertrainMode;
            end
        end

        function tf = shouldContinueSimulation(obj, state, trackLen, finishTolerance)
            reachedTrackEnd = obj.stopAtTrackEnd && state.s >= trackLen - finishTolerance;
            reachedStopTime = isfinite(obj.stopTime) && ...
                state.time >= obj.stopTime - 0.5 * obj.dt;
            stoppedOffTrack = obj.stopOnOffTrack && ~state.onTrack;
            tf = ~(reachedTrackEnd || reachedStopTime || stoppedOffTrack);
        end

        function progress = simulationProgress(~, state, input, trackLen)
            progress = NaN;
            if isstruct(input) && isfield(input, 'replayProgress') && ...
                    isfinite(input.replayProgress)
                progress = input.replayProgress;
            elseif trackLen > 0
                progress = state.s / trackLen;
            end
            progress = max(0, min(1, progress));
        end

        function text = replayProgressText(~, input)
            if ~isstruct(input) || ~isfield(input, 'replayProgress') || ...
                    ~isfinite(input.replayProgress)
                text = 'n/a';
                return;
            end

            domain = localGetField(input, 'replayDomain', '');
            switch string(domain)
                case "time"
                    text = sprintf('%.3f s', localGetField(input, 'sourceTime', NaN));
                case "distance"
                    text = sprintf('%.1f m', localGetField(input, 'sourceDistance', NaN));
                otherwise
                    text = sprintf('%.1f%%', input.replayProgress * 100);
            end
        end

        function brakeForces = computeBrakeForces(obj, input, totalNormalLoad)
            brakeForces = lts.simulation.BrakeForcePolicy.compute( ...
                input, totalNormalLoad, obj.vehicleManager, obj.brakeMode);
        end

        function [totalDriveTorque, totalCoastdownTorque] = computePowertrainTorques(obj, state, input, throttle)
            vm = obj.vehicleManager;
            mode = obj.validatePowertrainMode(obj.powertrainMode);

            switch mode
                case "throttle"
                    totalDriveTorque = vm.powertrain.computeDriveTorque(state.speed, throttle);

                    % Off-throttle motoring/regen drag on the driven axle
                    % (opt-in via the powertrain component; 0 when off). This
                    % is signed driveline torque so ramp-plate LSDs can use
                    % their decel ramps. Hydraulic brake torque is separate.
                    totalCoastdownTorque = 0;
                    if ismethod(vm.powertrain, 'computeCoastdownTorque')
                        totalCoastdownTorque = vm.powertrain.computeCoastdownTorque( ...
                            state.speed, throttle);
                    end

                case "motor_torque_command"
                    motorTorqueCommandNm = localGetField(input, ...
                        'motorTorqueCommandNm', NaN);
                    if ~isfinite(motorTorqueCommandNm)
                        error('lts_simulation_Simulator:MissingMotorTorqueCommand', ...
                            ['PowertrainMode "motor_torque_command" requires a finite ' ...
                            'motorTorqueCommandNm input from the replay profile.']);
                    end

                    motorTorqueRequestNm = obj.selectDirectMotorTorqueRequest( ...
                        motorTorqueCommandNm, input);
                    wheelTorque = obj.applyMotorTorqueCommand( ...
                        motorTorqueRequestNm, throttle, input);
                    totalDriveTorque = max(0, wheelTorque);
                    totalCoastdownTorque = min(0, wheelTorque);

                otherwise
                    error('lts_simulation_Simulator:InvalidPowertrainMode', ...
                        'PowertrainMode must be "throttle" or "motor_torque_command".');
            end
        end

        function wheelTorque = applyMotorTorqueCommand(obj, motorTorqueCommandNm, throttle, input)
            if nargin < 4 || isempty(input)
                input = struct();
            end

            vm = obj.vehicleManager;
            ratio = vm.powertrain.getTotalGearRatio();
            efficiency = vm.powertrain.getDrivetrainEfficiency();
            regenEfficiency = obj.regenDrivetrainEfficiency();
            [appliedMotorTorqueNm, powerLimitNm, packVoltageV, ...
                packCurrentA, packPowerW, powerLimitActive] = ...
                obj.limitMotorTorqueCommandByPackPower(motorTorqueCommandNm, input);
            if appliedMotorTorqueNm >= 0
                wheelTorque = appliedMotorTorqueNm * ratio * efficiency;
                stateEfficiency = efficiency;
            else
                % Reverse the loss direction for regen: wheel braking power
                % must exceed the mechanical/electrical power reaching the
                % motor/pack, not shrink below it as motoring torque does.
                wheelTorque = appliedMotorTorqueNm * ratio / max(regenEfficiency, eps);
                stateEfficiency = regenEfficiency;
            end

            driveForce = 0;
            if ~isempty(vm.tire) && isprop(vm.tire, 'RL')
                driveForce = wheelTorque / max(vm.tire.RL.wheelRadius, eps);
            end

            if ~isempty(vm.powertrain.state)
                vm.powertrain.state.updateOutputs( ...
                    throttle, appliedMotorTorqueNm, wheelTorque, driveForce, ...
                    stateEfficiency, false);
                vm.powertrain.state.requestedMotorTorque = motorTorqueCommandNm;
                vm.powertrain.state.motorTorquePowerLimitNm = powerLimitNm;
                vm.powertrain.state.motorTorquePowerLimitActive = powerLimitActive;
                vm.powertrain.state.packVoltageV = packVoltageV;
                vm.powertrain.state.packCurrentA = packCurrentA;
                vm.powertrain.state.packPowerW = packPowerW;
            end
        end

        function efficiency = regenDrivetrainEfficiency(obj)
            vm = obj.vehicleManager;
            efficiency = vm.powertrain.getDrivetrainEfficiency();
            if ismethod(vm.powertrain, 'getRegenDrivetrainEfficiency')
                efficiency = vm.powertrain.getRegenDrivetrainEfficiency();
            elseif isprop(vm.powertrain, 'regenEfficiency') && ...
                    isfinite(vm.powertrain.regenEfficiency)
                efficiency = vm.powertrain.regenEfficiency;
            end
            efficiency = max(0, min(1, efficiency));
        end

        function motorTorqueRequestNm = selectDirectMotorTorqueRequest(~, motorTorqueCommandNm, input)
            motorTorqueRequestNm = motorTorqueCommandNm;
            if motorTorqueCommandNm < 0
                return;
            end

            regenTorqueNm = localGetField(input, 'regenTorqueNm', NaN);
            if ~isfinite(regenTorqueNm) || regenTorqueNm >= 0
                return;
            end

            packVoltageV = localGetField(input, 'packVoltageV', NaN);
            packCurrentA = localGetField(input, 'packCurrentA', NaN);
            if ~isfinite(packVoltageV) || ~isfinite(packCurrentA) || packVoltageV <= 0
                return;
            end

            % The throttle-regen channel is a candidate/request and can be
            % nonzero during motoring. Let it override Calculated Cmd only
            % when the logged pack power confirms actual charging.
            if packVoltageV * packCurrentA < -100
                motorTorqueRequestNm = regenTorqueNm;
            end
        end

        function [appliedMotorTorqueNm, powerLimitNm, packVoltageV, ...
                packCurrentA, packPowerW, powerLimitActive] = ...
                limitMotorTorqueCommandByPackPower(obj, requestedMotorTorqueNm, input)
            appliedMotorTorqueNm = requestedMotorTorqueNm;
            powerLimitNm = NaN;
            packVoltageV = localGetField(input, 'packVoltageV', NaN);
            packCurrentA = localGetField(input, 'packCurrentA', NaN);
            packPowerW = NaN;
            powerLimitActive = false;

            if ~isfinite(packVoltageV) || ~isfinite(packCurrentA) || packVoltageV <= 0
                return;
            end

            packPowerW = packVoltageV * packCurrentA;
            if ~isfinite(packPowerW) || requestedMotorTorqueNm == 0
                return;
            end

            motorOmega = obj.motorAngularVelocityForPowerLimit(input);
            if ~isfinite(motorOmega)
                return;
            end
            motorOmega = max(abs(motorOmega), 1.0);

            if requestedMotorTorqueNm > 0
                powerLimitNm = max(0, packPowerW) / motorOmega;
                appliedMotorTorqueNm = min(requestedMotorTorqueNm, powerLimitNm);
            else
                powerLimitNm = min(0, packPowerW) / motorOmega;
                appliedMotorTorqueNm = max(requestedMotorTorqueNm, powerLimitNm);
            end

            toleranceNm = max(1e-9, 1e-6 * abs(requestedMotorTorqueNm));
            powerLimitActive = abs(appliedMotorTorqueNm - requestedMotorTorqueNm) > toleranceNm;
        end

        function motorOmega = motorAngularVelocityForPowerLimit(obj, input)
            motorOmega = NaN;
            vm = obj.vehicleManager;
            loggedMotorRpm = localGetField(input, 'motorRpm', NaN);
            if isfinite(loggedMotorRpm)
                motorOmega = loggedMotorRpm * 2 * pi / 60;
                return;
            end

            replaySpeed = localGetField(input, 'targetSpeed', NaN);
            if isfinite(replaySpeed) && replaySpeed > 0 && ...
                    ~isempty(vm) && ~isempty(vm.powertrain)
                ratio = vm.powertrain.getTotalGearRatio();
                wheelRadius = NaN;
                if ~isempty(vm.tire) && isprop(vm.tire, 'RL')
                    wheelRadius = vm.tire.RL.wheelRadius;
                elseif isprop(vm.powertrain, 'wheelRadius')
                    wheelRadius = vm.powertrain.wheelRadius;
                end
                if isfinite(ratio) && ratio > 0 && isfinite(wheelRadius) && wheelRadius > 0
                    motorOmega = replaySpeed / wheelRadius * ratio;
                    return;
                end
            end

            if ~isempty(vm) && ~isempty(vm.powertrain) && ~isempty(vm.powertrain.state)
                motorOmega = vm.powertrain.state.motorAngularVelocity;
                if isfinite(motorOmega)
                    return;
                end
            end

        end

        function mode = validateBrakeMode(~, mode)
            mode = lts.simulation.BrakeForcePolicy.validateMode(mode);
        end

        function mode = validatePowertrainMode(~, mode)
            mode = lower(string(mode));
            mode = strrep(mode, "-", "_");
            if mode == "motor_torque" || mode == "motor_command" || ...
                    mode == "calculated_cmd" || mode == "calculated_command"
                mode = "motor_torque_command";
            end
            if mode ~= "throttle" && mode ~= "motor_torque_command"
                error('lts_simulation_Simulator:InvalidPowertrainMode', ...
                    'PowertrainMode must be "throttle" or "motor_torque_command".');
            end
        end

        function state = initializePlanarState(obj, state, trackData, referenceMode, surfaceMu)
            % INITIALIZEPLANARSTATE Fill missing x/y/yaw/vx reference fields.
            %
            % Track mode starts on the first reference waypoint and heading
            % unless the caller supplied logged/free-space state. Free mode is
            % deliberately decoupled from track geometry for correlation runs.
            if nargin < 4 || isempty(referenceMode)
                referenceMode = "track";
            end
            if nargin < 5 || isempty(surfaceMu) || ~isfinite(surfaceMu) || surfaceMu <= 0
                surfaceMu = obj.freeSurfaceMu;
            end
            if lower(string(referenceMode)) == "free"
                if isnan(state.x)
                    state.x = 0;
                end
                if isnan(state.y)
                    state.y = 0;
                end
                if isnan(state.yaw)
                    state.yaw = 0;
                    state.heading = 0;
                end
                if isnan(state.vx)
                    state.vx = max(state.speed, 0);
                end
                state.speed = hypot(state.vx, state.vy);
                state.refS = state.s;
                state.refHeading = state.yaw;
                state.refCurvature = 0;
                state.curvature = 0;
                state.lateralError = 0;
                state.mu = surfaceMu;
                state.onTrack = true;
                return;
            end

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

        function tf = isFreeReferenceMode(obj)
            tf = lower(string(obj.referenceMode)) == "free";
        end

        function tf = isFreeReference(~, ref)
            tf = isstruct(ref) && isfield(ref, 'referenceMode') && ...
                strcmpi(char(ref.referenceMode), 'free');
        end

        function ref = freeReferenceForState(obj, state, s, x, y, yaw)
            if nargin < 6 || ~isfinite(yaw)
                yaw = state.yaw;
            end
            if ~isfinite(yaw)
                yaw = 0;
            end
            if nargin < 4 || ~isfinite(x)
                x = state.x;
            end
            if nargin < 5 || ~isfinite(y)
                y = state.y;
            end
            if ~isfinite(s)
                s = state.s;
            end
            ref = struct( ...
                'idx', 1, ...
                's', max(0, s), ...
                'x', x, ...
                'y', y, ...
                'heading', yaw, ...
                'curvature', 0, ...
                'mu', obj.freeSurfaceMu, ...
                'lateralError', 0, ...
                'trackWidth', 0, ...
                'trackHalfWidth', 0, ...
                'trackLimitMargin', 0, ...
                'onTrack', true, ...
                'referenceMode', 'free');
        end

        function tf = isLeanTelemetry(obj)
            tf = lower(string(obj.telemetryMode)) == "lean";
        end

        function stateLog = createLeanStateLog(~, maxSteps)
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
                'frontAxleAy', zeros(maxSteps, 1), ...
                'rearAxleAy',  zeros(maxSteps, 1), ...
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
                'brakePressureMode', false(maxSteps, 1), ...
                'brakePressureFrontBar', NaN(maxSteps, 1), ...
                'brakePressureRearBar', NaN(maxSteps, 1), ...
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
                'motorTorqueRequested', zeros(maxSteps, 1), ...
                'motorTorquePowerLimitNm', NaN(maxSteps, 1), ...
                'motorTorquePowerLimitActive', false(maxSteps, 1), ...
                'wheelTorque', zeros(maxSteps, 1), ...
                'packVoltageV', NaN(maxSteps, 1), ...
                'packCurrentA', NaN(maxSteps, 1), ...
                'packPowerW', NaN(maxSteps, 1), ...
                'drivenWheelRPM', zeros(maxSteps, 1), ...
                'rpmLimitActive', false(maxSteps, 1), ...
                'pitchAngle',  zeros(maxSteps, 1), ...
                'rollAngle',   zeros(maxSteps, 1), ...
                'rollRate',    zeros(maxSteps, 1), ...
                'frontRollAngle', zeros(maxSteps, 1), ...
                'rearRollAngle',  zeros(maxSteps, 1), ...
                'frontRollRate',  zeros(maxSteps, 1), ...
                'rearRollRate',   zeros(maxSteps, 1), ...
                'twistAngle',     zeros(maxSteps, 1), ...
                'twistRate',      zeros(maxSteps, 1), ...
                'rideHeight',  zeros(maxSteps, 1), ...
                'aeroFz_front', zeros(maxSteps, 1), ...
                'aeroFz_rear',  zeros(maxSteps, 1) ...
            );
        end

        function laps = getTrackWarmupLaps(~, track)
            laps = lts.simulation.TrackReference.warmupLaps(track);
        end

        function laps = getTrackRecordedLaps(~, track)
            laps = lts.simulation.TrackReference.recordedLaps(track);
        end

        function closed = isClosedLoopTrack(~, track, points)
            closed = lts.simulation.TrackReference.isClosedLoop(track, points);
        end

        function [points, curvature, mu, heading] = repeatClosedTrack(~, ...
                points, curvature, mu, heading, lapCount)
            [points, curvature, mu, heading] = ...
                lts.simulation.TrackReference.repeatClosed(points, curvature, mu, heading, lapCount);
        end

        function [stateLog, lapTime, recordedSteps] = applyTelemetryLapWindow(obj, ...
                stateLog, recordStartS, recordEndS)
            [stateLog, lapTime, recordedSteps] = ...
                lts.simulation.TelemetryWindow.apply(stateLog, recordStartS, recordEndS);
        end

        function diagnostics = computeTelemetryTrimDiagnostics(~, stateLog)
            diagnostics = lts.simulation.TelemetryWindow.trimDiagnostics(stateLog);
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
                    error('lts_simulation_Simulator:InvalidDriverModel', ...
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

            if obj.enforcePedalExclusivity
                if input.brake > 0
                    input.throttle = 0;
                elseif input.throttle > 0
                    input.brake = 0;
                end
            end

            maxSteer = obj.getMaxSteeringAngle();
            if isfinite(maxSteer)
                input.steer = max(-maxSteer, min(maxSteer, input.steer));
            end
            if nargin >= 3 && ~isempty(state)
                previousSteer = state.steer;
                if ~isfinite(previousSteer)
                    previousSteer = 0;
                end
                if isfinite(maxSteer)
                    previousSteer = max(-maxSteer, min(maxSteer, previousSteer));
                end
                rampTime = obj.getSteeringRampTime();
                if obj.applySteeringSlew && rampTime > 0 && isfinite(rampTime) && isfinite(maxSteer)
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
            ref = lts.simulation.TrackReference.referenceAtProgress( ...
                s, x, y, trackData);
        end

        function ref = projectToReference(~, x, y, trackData, previousIdx)
            ref = lts.simulation.TrackReference.projectToReference( ...
                x, y, trackData, previousIdx);
        end

        function tireData = updatePlanarTireForces(obj, state, cornerLoads, dt, computePeakMu, relaxationMode)
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
            if nargin < 6 || isempty(relaxationMode)
                if dt > 0
                    relaxationMode = 'advance';
                else
                    relaxationMode = 'steady';
                end
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
                % Slip angle is the angle between wheel heading and local
                % contact-patch velocity. The 0.1 m/s denominator floor avoids
                % undefined atan behavior while the car is nearly stationary.
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
                    surfaceMu, computePeakMu, relaxationMode);
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
                        computePeakMu, relaxationMode);
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
                % Planar yaw moment about CG: Mz = x*Fy - y*Fx for each
                % contact patch, using suspension geometry contact positions.
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
                isa(vm.chassis, 'lts.components.Chassis.ChassisComponent');
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
            if isnan(obj.cachedFrontArm)
                obj.cachedFrontArm = vm.wheelbase * (1 - vm.staticFrontWeight);
                obj.cachedRearArm  = vm.wheelbase * vm.staticFrontWeight;
            end
            dynamics.frontAxleAy = dynamics.ay + dynamics.yawAccel * obj.cachedFrontArm;
            dynamics.rearAxleAy  = dynamics.ay - dynamics.yawAccel * obj.cachedRearArm;
        end

        function kinematics = integratePlanarKinematics(~, state, dynamics, dt)
            % INTEGRATEPLANARKINEMATICS Advance body velocity through world space.
            %   Body-frame vx/vy are first projected into world coordinates,
            %   force-derived body accelerations are applied at midpoint yaw,
            %   then the updated velocity is expressed in the new body frame.
            %   Reprojecting at the new yaw is essential: a steady circular
            %   trajectory should not create artificial sideslip or kinetic energy.
            vx0 = state.vx;
            vy0 = state.vy;
            yaw0 = state.yaw;
            yawRate0 = state.yawRate;

            yawRateNew = yawRate0 + dynamics.yawAccel * dt;
            yawNew = yaw0 + yawRate0 * dt + 0.5 * dynamics.yawAccel * dt^2;
            yawMid = yaw0 + 0.5 * yawRate0 * dt + 0.125 * dynamics.yawAccel * dt^2;

            cy0 = cos(yaw0); sy0 = sin(yaw0);
            cyMid = cos(yawMid); syMid = sin(yawMid);
            cyNew = cos(yawNew); syNew = sin(yawNew);

            vxWorld0 = vx0 * cy0 - vy0 * sy0;
            vyWorld0 = vx0 * sy0 + vy0 * cy0;
            axWorld = dynamics.ax * cyMid - dynamics.ay * syMid;
            ayWorld = dynamics.ax * syMid + dynamics.ay * cyMid;

            vxWorld = vxWorld0 + axWorld * dt;
            vyWorld = vyWorld0 + ayWorld * dt;

            kinematics.vx = vxWorld * cyNew + vyWorld * syNew;
            kinematics.vy = -vxWorld * syNew + vyWorld * cyNew;
            kinematics.yawRate = yawRateNew;
            kinematics.yaw = yawNew;
            kinematics.x = state.x + 0.5 * (vxWorld0 + vxWorld) * dt;
            kinematics.y = state.y + 0.5 * (vyWorld0 + vyWorld) * dt;
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

            corners = {'FL', 'FR', 'RL', 'RR'};
            for i = 1:numel(corners)
                corner = corners{i};
                cornerKin = kin.(corner);
                [wheelX, wheelY] = obj.getWheelPosition(corner);
                if ~isfield(cornerKin, 'xPosition')
                    cornerKin.xPosition = wheelX;
                end
                if ~isfield(cornerKin, 'yPosition')
                    cornerKin.yPosition = wheelY;
                end
                if ~isfield(cornerKin, 'wheelCenterXPosition')
                    cornerKin.wheelCenterXPosition = wheelX;
                end
                if ~isfield(cornerKin, 'wheelCenterYPosition')
                    cornerKin.wheelCenterYPosition = wheelY;
                end
                kin.(corner) = cornerKin;
            end
        end

        function trackData = precomputeTrackSegments(~, trackData)
            trackData = lts.simulation.TrackReference.precomputeSegments(trackData);
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
            % COMPUTELOCALSLIPRATIO SAE-style longitudinal slip.
            %   kappa = (omega*R - Vx) / max(|omega*R|, |Vx|, floor)
            %
            % Positive kappa means the tread is trying to drive the road
            % backwards (tractive force); negative means braking. Below the
            % 1 m/s floor the result blends toward the previous slip to avoid
            % violent sign flips when both wheel and ground speeds are tiny.
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

        function utilization = computeTireUtilization(~, cornerState)
            capacity = max(cornerState.peakMu, 0) * ...
                max(cornerState.normalForce, 0);
            if capacity <= eps
                utilization = 0;
                return;
            end

            utilization = hypot(cornerState.Fx, cornerState.Fy) / capacity;
            if ~isfinite(utilization)
                utilization = 0;
            end
        end

        function inertia = getWheelInertia(obj)
            if ~isempty(obj.cachedWheelInertia)
                inertia = obj.cachedWheelInertia;
                return;
            end
            inertia = lts.simulation.DrivelineSupport.wheelInertia(obj.vehicleManager);
            obj.cachedWheelInertia = inertia;
        end

        function out = solveDifferential(obj, totalDriveTorque, varargin)
            % SOLVEDIFFERENTIAL Split driven-axle torque via the configured
            % differential. Falls back to the legacy open behavior (50/50
            % torque, mean-speed carrier) when no differential is present.
            if numel(varargin) == 4
                totalCoastdownTorque = 0;
                omegaL = varargin{1};
                omegaR = varargin{2};
                wheelInertia = varargin{3};
                dt = varargin{4};
            elseif numel(varargin) == 5
                totalCoastdownTorque = varargin{1};
                omegaL = varargin{2};
                omegaR = varargin{3};
                wheelInertia = varargin{4};
                dt = varargin{5};
            else
                error('lts_simulation_Simulator:BadDifferentialArgs', ...
                    'Expected drive torque plus omega/inertia/dt, optionally with coastdown torque.');
            end

            out = lts.simulation.DrivelineSupport.solveDifferential( ...
                obj.vehicleManager, totalDriveTorque, totalCoastdownTorque, ...
                omegaL, omegaR, wheelInertia, dt);
        end

        function locked = differentialLocksWheels(obj)
            % DIFFERENTIALLOCKSWHEELS True if the diff forces equal wheel speed.
            %   Memoized: the differential type is a run invariant.
            if ~isempty(obj.cachedDiffLocksWheels)
                locked = obj.cachedDiffLocksWheels;
                return;
            end
            locked = lts.simulation.DrivelineSupport.locksWheels(obj.vehicleManager);
            obj.cachedDiffLocksWheels = locked;
        end

        function initializeWheelSpeeds(obj, vehicleSpeed)
            lts.simulation.DrivelineSupport.initializeWheelSpeeds( ...
                obj.vehicleManager, vehicleSpeed);
        end

        function initializeCornerWheelSpeed(~, cornerState, vehicleSpeed)
            lts.simulation.DrivelineSupport.initializeCornerWheelSpeed( ...
                cornerState, vehicleSpeed);
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

function values = localInterpProfileChannel(axis, channel, query)
axis = double(axis(:));
channel = double(channel(:));
query = double(query(:));

keep = isfinite(axis) & isfinite(channel);
axis = axis(keep);
channel = channel(keep);

if isempty(axis)
    values = NaN(size(query));
elseif numel(axis) == 1
    values = repmat(channel(1), size(query));
else
    [axis, uniqueIdx] = unique(axis, 'stable');
    channel = channel(uniqueIdx);
    query = max(axis(1), min(axis(end), query));
    values = interp1(axis, channel, query, 'linear');
end
end
