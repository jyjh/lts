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
        limitMotorTorqueByPackPower = false
        stopOnOffTrack = true
        stopAtTrackEnd = true
        stopTime = inf
        referenceMode = "track"
        % Deprecated compatibility setting. Surface-dependent grip has been
        % removed; every reference surface is neutral and reports mu = 1.
        freeSurfaceMu = 1

        % Internal: track whether maxSpeed warning was issued (warn once)
        warnedMaxSpeed = false
    end

    properties (Transient = true) %#ok<MCNPC>
        % Lazily-cached run invariants. The lts.vehicle.VehicleManager and its components
        % do not change during a simulation, so capability/value lookups that
        % the hot loop used to repeat every step (isa/ismethod/isprop) are
        % resolved once on first use and memoized here. NaN/empty = uncached.
        cachedWheelInertia = struct([])
        cachedDiffLocksWheels
        cachedTireHasBatchUpdate
        cachedSuspensionHasKinematics
        cachedDriverInputMethod   % 'computeInput' | 'computeInputs' | '' (cached)
        cachedFrontArm = NaN       % CG-to-front-axle moment arm [m] (run invariant)
        cachedRearArm = NaN        % CG-to-rear-axle moment arm [m] (run invariant)
        cachedRollingResistanceCoeff = NaN
        cachedPowertrainHasCoastdown
        cachedMaxSteeringAngle
        cachedSteeringRampTime
        cachedPowertrainMode = []  % validated powertrain mode string
        cachedPowertrainModeSource = [] % raw property value used to build cache
        cachedNextRef = struct()   % nextRef written by step(); reused as currentRef next iteration
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
            obj.requireChassis();
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
            [dragDirX, dragDirY] = obj.bodyVelocityDirection(state);
            % Positive component magnitudes along the velocity vector. The
            % actual aerodynamic body forces are their negatives. Chassis
            % load transfer uses these to separate tire-induced acceleration
            % from the force applied at the aero resultant height.
            aeroForces.F_drag_longitudinal = F_drag * dragDirX;
            aeroForces.F_drag_lateral = F_drag * dragDirY;
            
            % --- WEIGHT AND PER-CORNER LOADS ---
            W = vm.totalMass * vm.g;
            
            % Use the sprung-mass attitude from the previous completed step
            % to impose corner motion on the suspension. The chassis is then
            % advanced later in this step with the newly computed
            % accelerations, giving a stable one-step stagger between tire
            % loads and platform attitude.
            cornerLoads = obj.getCurrentCornerLoads(steer);
            
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
            inertia = obj.getWheelInertia();  % wheel inertia plus carrier coupling
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

            % Fixed-point wheel-contact solve over one physical timestep.
            % Every iteration starts from omega(t) and recomputes the same
            % omega(t+dt) candidate using the latest tire force. Iterations
            % therefore improve convergence without applying drive/brake
            % impulse more than once.
            tireInputState = state;
            tireInputState.steer = steer;
            tireContact = obj.computePlanarTireContactData( ...
                tireInputState, obj.getCornerKinematics(steer));
            wheelLongSpeeds = tireContact.longSpeeds;
            omegaStart = [vm.tire.FL.angularVelocity; vm.tire.FR.angularVelocity; ...
                vm.tire.RL.angularVelocity; vm.tire.RR.angularVelocity];

            nWheelIter = max(1, round(obj.wheelSolveIterations));
            for iter = 1:nWheelIter
                vm.tire.FL.angularVelocity = omegaStart(1);
                vm.tire.FR.angularVelocity = omegaStart(2);
                vm.tire.RL.angularVelocity = omegaStart(3);
                vm.tire.RR.angularVelocity = omegaStart(4);
                vm.tire.updateWheelDynamics(vm.tire.FL, T_drive_front, T_brake_front, obj.dt, inertia.FL, wheelLongSpeeds(1));
                vm.tire.updateWheelDynamics(vm.tire.FR, T_drive_front, T_brake_front, obj.dt, inertia.FR, wheelLongSpeeds(2));
                vm.tire.updateDrivenWheelPairDynamics( ...
                    vm.tire.RL, vm.tire.RR, T_drive_RL, T_drive_RR, ...
                    T_brake_rear, T_brake_rear, obj.dt, inertia.RL, inertia.RR, ...
                    inertia.reflectedRotorInertia, wheelLongSpeeds(3), wheelLongSpeeds(4));

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
                if iter < nWheelIter
                    % Feed the updated carrier speed back into the torque map
                    % for the next fixed-point candidate. Do not evaluate
                    % after the final candidate: that torque was not applied
                    % during this physical step and must not reach telemetry.
                    [totalDriveTorque, totalCoastdownTorque] = ...
                        obj.computePowertrainTorques(state, input, throttle);
                    diffOut = obj.solveDifferential( ...
                        totalDriveTorque, totalCoastdownTorque, ...
                        vm.tire.RL.angularVelocity, vm.tire.RR.angularVelocity, ...
                        inertia.RL, obj.dt);
                    T_drive_RL = diffOut.TL;
                    T_drive_RR = diffOut.TR;
                end

                % Preview relaxation from the committed state during
                % intermediate iterations. Only the final evaluation commits
                % it, so the lag advances exactly once per physical step.
                if iter < nWheelIter
                    tireData = obj.updatePlanarTireForces( ...
                        tireInputState, cornerLoads, obj.dt, false, 'preview', tireContact);
                else
                    tireData = obj.updatePlanarTireForces( ...
                        tireInputState, cornerLoads, obj.dt, true, 'advance', tireContact);
                end
            end
            dynamics = obj.computePlanarDynamics(state, tireData, aeroForces);

            % Integrate the sprung-mass attitude (heave/pitch/roll) from the
            % converged body accelerations and aero forces. The chassis owns
            % pitch, roll, and ride-height, which feed next step's aero
            % (ground effect, pitch sensitivity) and are read back by
            % lts.simulation.VehicleState.computePitch/Roll/RideHeight below.
            vm.chassis.updateFromAccelerations( ...
                dynamics.ax, dynamics.ay, aeroForces, obj.dt, dynamics.yawAccel);

            vm.powertrain.updateStateFromDrivenWheels( ...
                [vm.tire.RL.angularVelocity, vm.tire.RR.angularVelocity]);

            F_tire_long = tireData.sumFxBody;
            F_drive = max(0, F_tire_long);
            F_brake = min(0, F_tire_long);
            % Telemetry only: equivalent rolling-resistance force from the
            % per-wheel torque model (sum of Crr*Fz over the four corners).
            % This is already reflected in the tire Fx above, not applied again.
            % Short-circuit when Crr == 0 to avoid four handle-property reads.
            rollingResistanceCoeff = obj.getRollingResistanceCoeff();
            if rollingResistanceCoeff ~= 0
                corners = vm.tire;
                F_rollResist = rollingResistanceCoeff * ...
                    (corners.FL.normalForce + corners.FR.normalForce + ...
                     corners.RL.normalForce + corners.RR.normalForce);
            else
                F_rollResist = 0;
            end

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
            % Cache nextRef so the outer simulate() loop can skip re-projecting
            % currentState.(x,y) at the top of the next iteration — they are
            % identical to (xNew, yNew) that was just projected here.
            obj.cachedNextRef = nextRef;
            
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
            % moment bookkeeping, which happened in updateFromAccelerations.
            forces.aeroDragHeight = localGetField(aeroForces, 'dragHeight', 0);
            forces.downforcePitchMoment = vm.chassis.state.downforcePitchMoment;
            forces.dragPitchMoment = vm.chassis.state.dragPitchMoment;
            forces.aeroPitchMoment = vm.chassis.state.aeroPitchMoment;
            forces.F_tire_lat = tireData.sumFyBody;
            forces.yawMoment = dynamics.yawMoment;
            forces.yawAccel = dynamics.yawAccel;
            forces.frontAxleAy = dynamics.frontAxleAy;
            forces.rearAxleAy = dynamics.rearAxleAy;
            forces.rollResistance = F_rollResist;
        end
        
        function [stateLog, lapTime] = simulate(obj, initialState, track, varargin)
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
            
            parser = inputParser;
            parser.addParameter('PreserveInitialComponentState', false, ...
                @(x) (islogical(x) || isnumeric(x)) && isscalar(x));
            parser.parse(varargin{:});

            vm = obj.vehicleManager;
            obj.resetForSimulation( ...
                logical(parser.Results.PreserveInitialComponentState));
            
            % Set vehicleManager reference on state so components can access constants
            initialState.vehicleManager = vm;
            
            % Get track data
            trackPtsBase  = track.getTrackPoints();
            curvatureBase = track.getCurvature();
            % Surface friction is compatibility telemetry only. Tire forces
            % are defined entirely by the tire data, so every reference point
            % carries neutral mu = 1.
            muBase        = ones(size(curvatureBase));
            headingBase   = track.getHeading();
            baseTrackLen  = track.getTotalLength();
            trackWidth    = track.getTrackWidth();

            warmupLaps = obj.getTrackWarmupLaps(track);
            recordedLaps = obj.getTrackRecordedLaps(track);
            totalLaps = warmupLaps + recordedLaps;

            closedLoop = obj.isClosedLoopTrack(track, trackPtsBase);
            if totalLaps > 1 && ~closedLoop
                error('lts_simulation_Simulator:WarmupRequiresClosedLoop', ...
                    'Track warmup/recorded laps require a closed-loop track.');
            end
            if closedLoop
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
                obj.referenceMode, 1);
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
            % Pre-compute run-invariant stop conditions (P1-B).
            hasFiniteStopTime = isfinite(obj.stopTime);
            stopTimeBound     = obj.stopTime - 0.5 * obj.dt;
            trackEndBound     = trackLen - finishTolerance;
            % Inline termination check using pre-computed locals.
            stopCheck = @(st) ~( ...
                (obj.stopAtTrackEnd  && st.s    >= trackEndBound) || ...
                (hasFiniteStopTime  && st.time >= stopTimeBound)  || ...
                (obj.stopOnOffTrack && ~st.onTrack));
            while stopCheck(currentState)
                % P2-C: The outer projectToReference call is eliminated after
                % the first step. step() writes its internally-computed nextRef
                % to obj.cachedNextRef, so we reuse it here. The first step
                % uses the currentRef that was computed just before the loop.
                currentState.s = currentRef.s;
                currentState.refS = currentRef.s;
                currentState.refHeading = currentRef.heading;
                currentState.refCurvature = currentRef.curvature;
                currentState.curvature = currentRef.curvature;
                currentState.lateralError = currentRef.lateralError;
                currentState.mu = currentRef.mu;
                currentState.onTrack = currentRef.onTrack;
                if ~stopCheck(currentState)
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
                    % P2-A: Resolve all optional input fields once to locals,
                    % avoiding repeated isstruct+isfield checks inside the block.
                    inputSourceDist   = localGetField(input, 'sourceDistance',      currentState.s);
                    inputSourceTime   = localGetField(input, 'sourceTime',          currentState.time);
                    inputTargetSpeed  = localGetField(input, 'targetSpeed',         NaN);
                    inputAxRef        = localGetField(input, 'axRef',               NaN);
                    inputTargetLatErr = localGetField(input, 'targetLateralError',  NaN);
                    inputLineCurv     = localGetField(input, 'lineCurvature',       NaN);
                    inputSpeedError   = localGetField(input, 'speedError',          NaN);

                    stateLog.time(step)        = newState.time;
                    stateLog.s(step)           = newState.s;
                    stateLog.controlS(step)    = inputSourceDist;
                    stateLog.x(step)           = newState.x;
                    stateLog.y(step)           = newState.y;
                    stateLog.yaw(step)         = newState.yaw;
                    stateLog.vx(step)          = newState.vx;
                    stateLog.vy(step)          = newState.vy;
                    stateLog.bodySlipAngle(step) = newState.bodySlipAngle;
                    stateLog.speed(step)       = newState.speed;
                    stateLog.speedKmh(step)    = newState.speed * 3.6;
                    stateLog.controlTime(step) = inputSourceTime;
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
                    stateLog.targetSpeed(step) = inputTargetSpeed;
                    stateLog.axRef(step)       = inputAxRef;
                    stateLog.targetLateralError(step) = inputTargetLatErr;
                    stateLog.lineCurvature(step) = inputLineCurv;
                    if isfinite(inputSpeedError)
                        stateLog.speedError(step) = inputSpeedError;
                    elseif isfinite(inputTargetSpeed)
                        stateLog.speedError(step) = currentState.speed - inputTargetSpeed;
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
                        stateLog.tireSpeed_FL(step) = abs(vm.tire.FL.angularVelocity * vm.tire.FL.wheelRadius);
                        stateLog.tireSpeed_FR(step) = abs(vm.tire.FR.angularVelocity * vm.tire.FR.wheelRadius);
                        stateLog.tireSpeed_RL(step) = abs(vm.tire.RL.angularVelocity * vm.tire.RL.wheelRadius);
                        stateLog.tireSpeed_RR(step) = abs(vm.tire.RR.angularVelocity * vm.tire.RR.wheelRadius);
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
                % P2-C: Reuse the nextRef computed inside step() as currentRef
                % for the next iteration — avoids re-projecting currentState.(x,y).
                currentRef = obj.cachedNextRef;
                
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

            % A finite stop time denotes a replay-bounded run. Its complete
            % telemetry window is time-domain, even when track projection is
            % retained for diagnostics; do not reinterpret it as an
            % incomplete distance-domain lap.
            if obj.isFreeReferenceMode() || isfinite(obj.stopTime)
                recordedSteps = step;
                if recordedSteps > 0
                    lapTime = stateLog.time(end);
                else
                    lapTime = NaN;
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
            parser.addParameter('LimitMotorTorqueByPackPower', ...
                obj.limitMotorTorqueByPackPower, @(x) islogical(x) || isnumeric(x));
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
            previousLimitMotorTorqueByPackPower = obj.limitMotorTorqueByPackPower;
            previousOffTrackPolicy = obj.stopOnOffTrack;
            previousTrackEndPolicy = obj.stopAtTrackEnd;
            previousStopTime = obj.stopTime;
            previousReferenceMode = obj.referenceMode;
            previousFreeSurfaceMu = obj.freeSurfaceMu;
            cleanup = onCleanup(@() obj.restoreReplayPolicies( ...
                previousDriver, previousMethod, previousPedalPolicy, ...
                previousSteerPolicy, previousBrakeMode, previousOffTrackPolicy, ...
                previousTrackEndPolicy, previousStopTime, ...
                previousReferenceMode, previousFreeSurfaceMu, previousPowertrainMode, ...
                previousLimitMotorTorqueByPackPower));

            obj.driverModel = lts.correlation.TelemetryReplayDriver(profile, ...
                'ReplayDomain', parser.Results.ReplayDomain);
            obj.cachedDriverInputMethod = [];
            obj.enforcePedalExclusivity = ~logical(parser.Results.AllowPedalOverlap);
            obj.applySteeringSlew = logical(parser.Results.ApplySteeringSlew);
            obj.brakeMode = obj.validateBrakeMode(parser.Results.BrakeMode);
            obj.powertrainMode = obj.validatePowertrainMode(parser.Results.PowertrainMode);
            obj.cachedPowertrainMode = [];
            obj.cachedPowertrainModeSource = [];
            obj.limitMotorTorqueByPackPower = ...
                logical(parser.Results.LimitMotorTorqueByPackPower);
            if obj.powertrainMode == "motor_torque_command" && ...
                    ~profile.hasMotorTorqueCommand()
                error('lts_simulation_Simulator:MissingMotorTorqueCommand', ...
                    ['PowertrainMode "motor_torque_command" requires ' ...
                    'motor_torque_command_nm in the replay profile.']);
            end
            if obj.powertrainMode == "motor_torque_delivered" && ...
                    ~profile.hasMotorTorqueDelivered()
                error('lts_simulation_Simulator:MissingMotorTorqueDelivered', ...
                    ['PowertrainMode "motor_torque_delivered" requires ' ...
                    'motor_torque_delivered_nm in the replay profile.']);
            end
            obj.stopOnOffTrack = logical(parser.Results.StopOnOffTrack);
            obj.stopAtTrackEnd = logical(parser.Results.StopAtTrackEnd);
            obj.referenceMode = lower(string(parser.Results.ReferenceMode));
            if obj.referenceMode ~= "track" && obj.referenceMode ~= "free"
                error('lts_simulation_Simulator:InvalidReferenceMode', ...
                    'ReferenceMode must be "track" or "free".');
            end
            % SurfaceMu is accepted for replay/API compatibility only.
            obj.freeSurfaceMu = 1;
            if logical(parser.Results.StopAtReplayEnd)
                obj.stopTime = profile.duration();
            else
                obj.stopTime = inf;
            end

            % CorrelationStateInitializer warm-starts the chassis, suspension,
            % tire relaxation, wheel-speed, and powertrain component states.
            % A normal simulate() call resets those components for deterministic
            % standalone runs; replay must retain them to avoid rebuilding a
            % mid-corner transient from static equilibrium.
            [stateLog, lapTime] = obj.simulate(initialState, track, ...
                'PreserveInitialComponentState', true);
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
            if profile.hasMotorTorqueDelivered()
                stateLog.replayMotorTorqueDeliveredNm = localInterpProfileChannel( ...
                    profile.time, profile.motorTorqueDeliveredNm, queryTime);
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
            if profile.hasWheelSpeeds()
                stateLog.replayWheelSpeedFL = localInterpProfileChannel( ...
                    profile.time, profile.wheelSpeedFL, queryTime);
                stateLog.replayWheelSpeedFR = localInterpProfileChannel( ...
                    profile.time, profile.wheelSpeedFR, queryTime);
                stateLog.replayWheelSpeedRL = localInterpProfileChannel( ...
                    profile.time, profile.wheelSpeedRL, queryTime);
                stateLog.replayWheelSpeedRR = localInterpProfileChannel( ...
                    profile.time, profile.wheelSpeedRR, queryTime);
                simulatedFields = { ...
                    'tireSpeed_FL', 'tireSpeed_FR', ...
                    'tireSpeed_RL', 'tireSpeed_RR'};
                replayFields = { ...
                    'replayWheelSpeedFL', 'replayWheelSpeedFR', ...
                    'replayWheelSpeedRL', 'replayWheelSpeedRR'};
                errorFields = { ...
                    'wheelSpeedErrorFL', 'wheelSpeedErrorFR', ...
                    'wheelSpeedErrorRL', 'wheelSpeedErrorRR'};
                for cornerIdx = 1:numel(simulatedFields)
                    if isfield(stateLog, simulatedFields{cornerIdx})
                        stateLog.(errorFields{cornerIdx}) = ...
                            stateLog.(simulatedFields{cornerIdx}) - ...
                            stateLog.(replayFields{cornerIdx});
                    end
                end
            end
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
                referenceMode, ~, powertrainMode, limitMotorTorqueByPackPower)
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
                %#ok<NASGU> Legacy argument retained for call compatibility.
                obj.freeSurfaceMu = 1;
            end
            if nargin >= 12
                obj.powertrainMode = powertrainMode;
            end
            if nargin >= 13
                obj.limitMotorTorqueByPackPower = limitMotorTorqueByPackPower;
            end
            % Clear cached mode so it is re-resolved for the new configuration.
            obj.cachedPowertrainMode = [];
            obj.cachedPowertrainModeSource = [];
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
            progress = lts.util.saturate(progress);
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
            % P1-A: Cache the validated mode string — powertrainMode is a run
            % invariant so lower()/strrep() need only run once per simulation.
            modeSource = string(obj.powertrainMode);
            if isempty(obj.cachedPowertrainMode) || ...
                    isempty(obj.cachedPowertrainModeSource) || ...
                    ~isequal(modeSource, obj.cachedPowertrainModeSource)
                obj.cachedPowertrainMode = obj.validatePowertrainMode(modeSource);
                obj.cachedPowertrainModeSource = modeSource;
            end
            mode = obj.cachedPowertrainMode;

            switch mode
                case "throttle"
                    totalDriveTorque = vm.powertrain.computeDriveTorque(state.speed, throttle);

                    % Off-throttle motoring/regen drag on the driven axle
                    % (opt-in via the powertrain component; 0 when off). This
                    % is signed driveline torque so ramp-plate LSDs can use
                    % their decel ramps. Hydraulic brake torque is separate.
                    totalCoastdownTorque = 0;
                    if isempty(obj.cachedPowertrainHasCoastdown)
                        obj.cachedPowertrainHasCoastdown = ...
                            ismethod(vm.powertrain, 'computeCoastdownTorque');
                    end
                    if obj.cachedPowertrainHasCoastdown
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

                case "motor_torque_delivered"
                    deliveredMotorTorqueNm = localGetField(input, ...
                        'motorTorqueDeliveredNm', NaN);
                    if ~isfinite(deliveredMotorTorqueNm)
                        error('lts_simulation_Simulator:MissingMotorTorqueDelivered', ...
                            ['PowertrainMode "motor_torque_delivered" requires a finite ' ...
                            'motorTorqueDeliveredNm input from the replay profile.']);
                    end
                    wheelTorque = obj.applyDeliveredMotorTorque( ...
                        deliveredMotorTorqueNm, throttle, input);
                    totalDriveTorque = max(0, wheelTorque);
                    totalCoastdownTorque = min(0, wheelTorque);

                otherwise
                    error('lts_simulation_Simulator:InvalidPowertrainMode', ...
                        ['PowertrainMode must be "throttle", "motor_torque_command", ' ...
                        'or "motor_torque_delivered".']);
            end
        end

        function wheelTorque = applyDeliveredMotorTorque( ...
                obj, deliveredMotorTorqueNm, throttle, input)
            vm = obj.vehicleManager;
            ratio = vm.powertrain.getTotalGearRatio();
            efficiency = vm.powertrain.getDrivetrainEfficiency();
            if ismethod(vm.powertrain, 'getDeliveredTorqueDrivetrainEfficiency')
                efficiency = vm.powertrain.getDeliveredTorqueDrivetrainEfficiency();
            end
            if deliveredMotorTorqueNm >= 0
                wheelTorque = deliveredMotorTorqueNm * ratio * efficiency;
            else
                wheelTorque = deliveredMotorTorqueNm * ratio / max(efficiency, eps);
            end

            driveForce = wheelTorque / max(vm.tire.RL.wheelRadius, eps);
            requestedMotorTorqueNm = localGetField(input, ...
                'motorTorqueCommandNm', deliveredMotorTorqueNm);
            packVoltageV = localGetField(input, 'packVoltageV', NaN);
            packCurrentA = localGetField(input, 'packCurrentA', NaN);
            vm.powertrain.state.updateOutputs( ...
                throttle, deliveredMotorTorqueNm, wheelTorque, driveForce, ...
                efficiency, false);
            vm.powertrain.state.requestedMotorTorque = requestedMotorTorqueNm;
            vm.powertrain.state.motorTorquePowerLimitNm = NaN;
            vm.powertrain.state.motorTorquePowerLimitActive = false;
            vm.powertrain.state.packVoltageV = packVoltageV;
            vm.powertrain.state.packCurrentA = packCurrentA;
            vm.powertrain.state.packPowerW = packVoltageV * packCurrentA;
        end

        function wheelTorque = applyMotorTorqueCommand(obj, motorTorqueCommandNm, throttle, input)
            if nargin < 4 || isempty(input)
                input = struct();
            end

            vm = obj.vehicleManager;
            ratio = vm.powertrain.getTotalGearRatio();
            efficiency = obj.motoringDrivetrainEfficiency(input);
            regenEfficiency = obj.regenDrivetrainEfficiency();
            [appliedMotorTorqueNm, powerLimitNm, packVoltageV, ...
                packCurrentA, packPowerW, powerLimitActive] = ...
                obj.limitMotorTorqueCommandByPackPower(motorTorqueCommandNm, input);
            [appliedMotorTorqueNm, rpmLimitActive] = ...
                obj.limitDirectMotorTorqueByRpm( ...
                motorTorqueCommandNm, appliedMotorTorqueNm);
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
                    stateEfficiency, rpmLimitActive);
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
            efficiency = lts.util.saturate(efficiency);
        end

        function efficiency = motoringDrivetrainEfficiency(obj, input)
            vm = obj.vehicleManager;
            efficiency = vm.powertrain.getDrivetrainEfficiency();
            if ~ismethod(vm.powertrain, 'getMotoringEfficiencyAtRPM')
                efficiency = lts.util.saturate(efficiency);
                return;
            end
            motorRPM = localGetField(input, 'motorRpm', NaN);
            if ~isfinite(motorRPM) && ~isempty(vm.powertrain.state)
                motorRPM = vm.powertrain.state.motorRPM;
            end
            efficiency = vm.powertrain.getMotoringEfficiencyAtRPM(motorRPM);
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
            if ~obj.limitMotorTorqueByPackPower
                return;
            end
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

        function [appliedMotorTorqueNm, rpmLimitActive] = ...
                limitDirectMotorTorqueByRpm(obj, requestedMotorTorqueNm, appliedMotorTorqueNm)
            rpmLimitActive = false;
            if requestedMotorTorqueNm <= 0
                return;
            end

            vm = obj.vehicleManager;
            if isempty(vm) || isempty(vm.powertrain) || isempty(vm.powertrain.state)
                return;
            end

            motorRPM = vm.powertrain.state.motorRPM;
            if ~isfinite(motorRPM)
                return;
            end

            if ismethod(vm.powertrain, 'isRPMLimitActive')
                rpmLimitActive = vm.powertrain.isRPMLimitActive(motorRPM);
            elseif isprop(vm.powertrain, 'rpmLimitRPM')
                limitRPM = vm.powertrain.rpmLimitRPM;
                if isfinite(limitRPM) && limitRPM > 0
                    releaseRPM = limitRPM;
                    if vm.powertrain.state.rpmLimitActive && ...
                            isprop(vm.powertrain, 'rpmLimitHysteresisRPM')
                        releaseRPM = limitRPM - max(0, vm.powertrain.rpmLimitHysteresisRPM);
                    end
                    rpmLimitActive = motorRPM >= releaseRPM;
                end
            end

            if rpmLimitActive
                appliedMotorTorqueNm = min(appliedMotorTorqueNm, 0);
            end
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
            if mode == "motor_iq" || mode == "measured_torque" || ...
                    mode == "delivered_torque"
                mode = "motor_torque_delivered";
            end
            if mode ~= "throttle" && mode ~= "motor_torque_command" && ...
                    mode ~= "motor_torque_delivered"
                error('lts_simulation_Simulator:InvalidPowertrainMode', ...
                    ['PowertrainMode must be "throttle", "motor_torque_command", ' ...
                    'or "motor_torque_delivered".']);
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
            if isstruct(input) && isfield(input, 'normalized') && ...
                    logical(input.normalized)
                return;
            end
            if ~isfield(input, 'throttle') || isempty(input.throttle)
                input.throttle = 0;
            end
            if ~isfield(input, 'brake') || isempty(input.brake)
                input.brake = 0;
            end
            if ~isfield(input, 'steer') || isempty(input.steer)
                input.steer = 0;
            end

            input.throttle = lts.util.saturate(input.throttle);
            input.brake = lts.util.saturate(input.brake);

            if obj.enforcePedalExclusivity
                if input.brake > 0
                    input.throttle = 0;
                elseif input.throttle > 0
                    input.brake = 0;
                end
            end

            maxSteer = obj.getMaxSteeringAngle();
            if isfinite(maxSteer)
                input.steer = lts.util.clamp(input.steer, -maxSteer, maxSteer);
            end
            if nargin >= 3 && ~isempty(state)
                previousSteer = state.steer;
                if ~isfinite(previousSteer)
                    previousSteer = 0;
                end
                if isfinite(maxSteer)
                    previousSteer = lts.util.clamp(previousSteer, -maxSteer, maxSteer);
                end
                rampTime = obj.getSteeringRampTime();
                if obj.applySteeringSlew && rampTime > 0 && isfinite(rampTime) && isfinite(maxSteer)
                    maxDelta = maxSteer * obj.dt / max(rampTime, eps);
                    delta = input.steer - previousSteer;
                    delta = lts.util.clamp(delta, -maxDelta, maxDelta);
                    input.steer = previousSteer + delta;
                end
            end
            input.normalized = true;
        end

        function maxSteer = getMaxSteeringAngle(obj)
            if ~isempty(obj.cachedMaxSteeringAngle)
                maxSteer = obj.cachedMaxSteeringAngle;
                return;
            end
            maxSteer = 0.6;
            if ~isempty(obj.driverModel) && ...
                    isprop(obj.driverModel, 'maxSteeringAngle')
                maxSteer = obj.driverModel.maxSteeringAngle;
            end
            maxSteer = max(maxSteer, eps);
            obj.cachedMaxSteeringAngle = maxSteer;
        end

        function rampTime = getSteeringRampTime(obj)
            if ~isempty(obj.cachedSteeringRampTime)
                rampTime = obj.cachedSteeringRampTime;
                return;
            end
            rampTime = obj.steeringRampTime;
            if ~isempty(obj.driverModel) && ...
                    isprop(obj.driverModel, 'steeringRampTime')
                rampTime = obj.driverModel.steeringRampTime;
            end
            obj.cachedSteeringRampTime = rampTime;
        end

        function ref = referenceAtProgress(~, s, x, y, trackData)
            ref = lts.simulation.TrackReference.referenceAtProgress( ...
                s, x, y, trackData);
        end

        function ref = projectToReference(~, x, y, trackData, previousIdx)
            ref = lts.simulation.TrackReference.projectToReference( ...
                x, y, trackData, previousIdx);
        end

        function tireData = updatePlanarTireForces(obj, state, cornerLoads, dt, computePeakMu, relaxationMode, tireContact)
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
            if nargin < 7 || isempty(tireContact)
                tireContact = obj.computePlanarTireContactData( ...
                    state, obj.getCornerKinematics(state.steer));
            end

            % Per-corner contact-patch longitudinal speeds feed the tire
            % relaxation length (transient slip lag).
            slipAngles = tireContact.slipAngles;
            longSpeedVec = tireContact.longSpeeds;
            tireFL = vm.tire.FL;
            tireFR = vm.tire.FR;
            tireRL = vm.tire.RL;
            tireRR = vm.tire.RR;
            slipRatios = [ ...
                obj.computeLocalSlipRatio(tireFL, longSpeedVec(1)); ...
                obj.computeLocalSlipRatio(tireFR, longSpeedVec(2)); ...
                obj.computeLocalSlipRatio(tireRL, longSpeedVec(3)); ...
                obj.computeLocalSlipRatio(tireRR, longSpeedVec(4))];
            surfaceMu = obj.getStateSurfaceMu(state);

            if isempty(obj.cachedTireHasBatchUpdate)
                obj.cachedTireHasBatchUpdate = ismethod(vm.tire, 'updateAllCorners');
            end
            if obj.cachedTireHasBatchUpdate
                vm.tire.updateAllCorners( ...
                    cornerLoads.FL, cornerLoads.FR, cornerLoads.RL, cornerLoads.RR, ...
                    slipAngles(1), slipAngles(2), slipAngles(3), slipAngles(4), ...
                    slipRatios(1), slipRatios(2), slipRatios(3), slipRatios(4), ...
                    tireContact.camberAngles(1), tireContact.camberAngles(2), ...
                    tireContact.camberAngles(3), tireContact.camberAngles(4), dt, longSpeedVec, ...
                    surfaceMu, computePeakMu, relaxationMode);
            else
                % Fallback for tire models without a batch update: evaluate
                % each corner individually. Wheel omega is integrated in the
                % main step() wheel-contact solve, not here.
                vm.tire.updateCorner(tireFL, cornerLoads.FL, slipAngles(1), ...
                    slipRatios(1), tireContact.camberAngles(1), surfaceMu, dt, ...
                    longSpeedVec(1), computePeakMu, relaxationMode);
                vm.tire.updateCorner(tireFR, cornerLoads.FR, slipAngles(2), ...
                    slipRatios(2), tireContact.camberAngles(2), surfaceMu, dt, ...
                    longSpeedVec(2), computePeakMu, relaxationMode);
                vm.tire.updateCorner(tireRL, cornerLoads.RL, slipAngles(3), ...
                    slipRatios(3), tireContact.camberAngles(3), surfaceMu, dt, ...
                    longSpeedVec(3), computePeakMu, relaxationMode);
                vm.tire.updateCorner(tireRR, cornerLoads.RR, slipAngles(4), ...
                    slipRatios(4), tireContact.camberAngles(4), surfaceMu, dt, ...
                    longSpeedVec(4), computePeakMu, relaxationMode);
            end

            cosWh = tireContact.cosWheelHeading;
            sinWh = tireContact.sinWheelHeading;
            xPos = tireContact.xPositions;
            yPos = tireContact.yPositions;

            FxBodyFL = tireFL.Fx * cosWh(1) - tireFL.Fy * sinWh(1);
            FyBodyFL = tireFL.Fx * sinWh(1) + tireFL.Fy * cosWh(1);
            FxBodyFR = tireFR.Fx * cosWh(2) - tireFR.Fy * sinWh(2);
            FyBodyFR = tireFR.Fx * sinWh(2) + tireFR.Fy * cosWh(2);
            FxBodyRL = tireRL.Fx * cosWh(3) - tireRL.Fy * sinWh(3);
            FyBodyRL = tireRL.Fx * sinWh(3) + tireRL.Fy * cosWh(3);
            FxBodyRR = tireRR.Fx * cosWh(4) - tireRR.Fy * sinWh(4);
            FyBodyRR = tireRR.Fx * sinWh(4) + tireRR.Fy * cosWh(4);

            sumFxBody = FxBodyFL + FxBodyFR + FxBodyRL + FxBodyRR;
            sumFyBody = FyBodyFL + FyBodyFR + FyBodyRL + FyBodyRR;
            yawMoment = ...
                xPos(1) * FyBodyFL - yPos(1) * FxBodyFL + ...
                xPos(2) * FyBodyFR - yPos(2) * FxBodyFR + ...
                xPos(3) * FyBodyRL - yPos(3) * FxBodyRL + ...
                xPos(4) * FyBodyRR - yPos(4) * FxBodyRR + ...
                tireFL.Mz + tireFR.Mz + tireRL.Mz + tireRR.Mz;
            tireData.sumFxBody = sumFxBody;
            tireData.sumFyBody = sumFyBody;
            tireData.yawMoment = yawMoment;
        end

        function requireChassis(obj)
            % REQUIRECHASSIS The physics engine has one authoritative
            % sprung-mass/load path. Running without it used a second,
            % algebraic model with different conservation behavior.
            vm = obj.vehicleManager;
            if isempty(vm) || isempty(vm.chassis) || ...
                    ~isa(vm.chassis, 'lts.components.Chassis.ChassisComponent')
                error('lts_simulation_Simulator:ChassisRequired', ...
                    ['Simulator requires a ChassisComponent; the obsolete ' ...
                    'algebraic no-chassis physics path has been removed.']);
            end
            if isempty(vm.suspension) || ...
                    ~ismethod(vm.suspension, 'computeCornerLoadsFromChassis')
                error('lts_simulation_Simulator:ChassisSuspensionRequired', ...
                    'Simulator requires a chassis-coupled suspension model.');
            end
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

        function resetForSimulation(obj, preserveInitialComponentState)
            % RESETFORSIMULATION Start each full run from deterministic
            % component state. Continuation workflows should use step()
            % directly; simulate() represents a new run.
            if nargin < 2
                preserveInitialComponentState = false;
            end
            obj.warnedMaxSpeed = false;
            obj.freeSurfaceMu = 1;
            obj.cachedWheelInertia = struct([]);
            obj.cachedDiffLocksWheels = [];
            obj.cachedTireHasBatchUpdate = [];
            obj.cachedSuspensionHasKinematics = [];
            obj.cachedDriverInputMethod = [];
            obj.cachedFrontArm = NaN;
            obj.cachedRearArm = NaN;
            obj.cachedRollingResistanceCoeff = NaN;
            obj.cachedPowertrainHasCoastdown = [];
            obj.cachedMaxSteeringAngle = [];
            obj.cachedSteeringRampTime = [];
            obj.cachedPowertrainMode = [];
            obj.cachedPowertrainModeSource = [];
            obj.cachedNextRef = struct();

            vm = obj.vehicleManager;
            if isempty(vm) || preserveInitialComponentState
                return;
            end

            if ~isempty(vm.chassis) && ismethod(vm.chassis, 'reset')
                vm.chassis.reset();
            end
            if ~isempty(vm.suspension) && ismethod(vm.suspension, 'warmup')
                vm.suspension.warmup(vm.totalMass, obj.dt);
            end
            if ~isempty(vm.tire)
                corners = {'FL', 'FR', 'RL', 'RR'};
                for i = 1:numel(corners)
                    name = corners{i};
                    if isprop(vm.tire, name) && ...
                            ~isempty(vm.tire.(name)) && ...
                            ismethod(vm.tire.(name), 'reset')
                        vm.tire.(name).reset();
                    end
                end
            end
            if ~isempty(vm.powertrain) && isprop(vm.powertrain, 'state') && ...
                    ~isempty(vm.powertrain.state) && ...
                    ismethod(vm.powertrain.state, 'reset')
                vm.powertrain.state.reset();
            end
        end

        function [dirX, dirY] = bodyVelocityDirection(~, state)
            vx = state.vx;
            vy = state.vy;
            if ~isfinite(vx) || ~isfinite(vy)
                dirX = 1;
                dirY = 0;
                return;
            end
            speed = hypot(vx, vy);
            if speed > eps
                dirX = vx / speed;
                dirY = vy / speed;
            else
                dirX = 1;
                dirY = 0;
            end
        end

        function dynamics = computePlanarDynamics(obj, state, tireData, aeroInput)
            % Rolling resistance is applied as a per-wheel resistance torque
            % (see PacejkaTire.updateWheelDynamics), which feeds back through
            % the tire Fx — it is NOT applied here as a body force (that would
            % double-count it).
            vm = obj.vehicleManager;
            if isstruct(aeroInput)
                F_drag = localGetField(aeroInput, 'F_drag', 0);
                dragXPosition = localGetField(aeroInput, 'dragXPosition', 0);
            else
                % Backward-compatible direct helper/test call.
                F_drag = aeroInput;
                dragXPosition = 0;
            end
            [velocityDirX, velocityDirY] = obj.bodyVelocityDirection(state);
            dragForceX = -F_drag * velocityDirX;
            dragForceY = -F_drag * velocityDirY;

            netFx = tireData.sumFxBody + dragForceX;
            netFy = tireData.sumFyBody + dragForceY;

            dynamics.ax = netFx / vm.totalMass;
            dynamics.ay = netFy / vm.totalMass;
            dynamics.dragForceX = dragForceX;
            dynamics.dragForceY = dragForceY;
            dynamics.dragYawMoment = dragXPosition * dragForceY;
            dynamics.yawMoment = tireData.yawMoment + dynamics.dragYawMoment;
            dynamics.yawAccel = dynamics.yawMoment / max(vm.yawInertia, eps);
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
            cyNew = cos(yawNew); syNew = sin(yawNew);
            % P2-B: First-order small-angle update for the mid-yaw pair.
            % At FSAE speeds (yawRate <= 3 rad/s, dt = 1 ms) the angular
            % increment dYawMid <= 0.0015 rad, giving truncation error
            % O(dYawMid^2) < 3e-6 rad — negligible for engineering purposes.
            dYawMid = yawMid - yaw0;
            cyMid = cy0 - sy0 * dYawMid;
            syMid = sy0 + cy0 * dYawMid;

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

        function kin = getCornerKinematics(obj, steer)
            vm = obj.vehicleManager;
            if isempty(obj.cachedSuspensionHasKinematics)
                obj.cachedSuspensionHasKinematics = ~isempty(vm.suspension) && ...
                    ismethod(vm.suspension, 'getCornerKinematics');
            end
            if obj.cachedSuspensionHasKinematics
                kin = vm.suspension.getCornerKinematics();
                if isa(vm.suspension, 'lts.components.Suspension.SuspensionManager')
                    return;
                end
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

        function contact = computePlanarTireContactData(~, state, kin)
            vx = state.vx;
            vy = state.vy;
            yawRate = state.yawRate;

            xPos = [kin.FL.xPosition; kin.FR.xPosition; ...
                    kin.RL.xPosition; kin.RR.xPosition];
            yPos = [kin.FL.yPosition; kin.FR.yPosition; ...
                    kin.RL.yPosition; kin.RR.yPosition];
            wheelHeading = [ ...
                kin.FL.steerAngle + kin.FL.toeAngle; ...
                kin.FR.steerAngle + kin.FR.toeAngle; ...
                kin.RL.steerAngle + kin.RL.toeAngle; ...
                kin.RR.steerAngle + kin.RR.toeAngle];

            cosWh = cos(wheelHeading);
            sinWh = sin(wheelHeading);
            vxCorner = vx - yawRate .* yPos;
            vyCorner = vy + yawRate .* xPos;
            longSpeeds = vxCorner .* cosWh + vyCorner .* sinWh;
            latSpeeds = -vxCorner .* sinWh + vyCorner .* cosWh;

            contact.kin = kin;
            contact.slipAngles = atan2(-latSpeeds, max(abs(longSpeeds), 0.1));
            contact.longSpeeds = longSpeeds;
            contact.cosWheelHeading = cosWh;
            contact.sinWheelHeading = sinWh;
            contact.xPositions = xPos;
            contact.yPositions = yPos;
            contact.camberAngles = [kin.FL.camberAngle; kin.FR.camberAngle; ...
                                    kin.RL.camberAngle; kin.RR.camberAngle];
        end

        function longSpeeds = computeCornerLongitudinalSpeeds(obj, state)
            contact = obj.computePlanarTireContactData( ...
                state, obj.getCornerKinematics(state.steer));
            longSpeeds.FL = contact.longSpeeds(1);
            longSpeeds.FR = contact.longSpeeds(2);
            longSpeeds.RL = contact.longSpeeds(3);
            longSpeeds.RR = contact.longSpeeds(4);
        end

        function surfaceMu = getStateSurfaceMu(~, ~)
            % Compatibility value only; surface friction no longer scales
            % tire forces anywhere in the physics path.
            surfaceMu = 1;
        end

        function coeff = getRollingResistanceCoeff(obj)
            if isfinite(obj.cachedRollingResistanceCoeff)
                coeff = obj.cachedRollingResistanceCoeff;
                return;
            end
            coeff = 0;
            tire = obj.vehicleManager.tire;
            if ~isempty(tire) && isprop(tire, 'rollingResistanceCoeff')
                coeff = tire.rollingResistanceCoeff;
            end
            if isempty(coeff) || ~isfinite(coeff)
                coeff = 0;
            end
            obj.cachedRollingResistanceCoeff = coeff;
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

        function kappa = computeLocalSlipRatio(obj, cornerState, longitudinalSpeed)
            % COMPUTELOCALSLIPRATIO Magic Formula longitudinal-slip contract.
            % Delegate to the tire model so standalone and simulator contact
            % paths cannot drift to different definitions.
            tire = obj.vehicleManager.tire;
            if ~isempty(tire) && ...
                    ismethod(tire, 'computeSlipRatioFromKinematics')
                kappa = tire.computeSlipRatioFromKinematics( ...
                    cornerState, longitudinalSpeed);
                return;
            end

            % Generic TireModel fallback: ground-speed denominator with a
            % low-speed regularization, zero force-producing slip at exact
            % rest, and no symmetric wheel-speed normalization.
            wheelSpeed = cornerState.angularVelocity * cornerState.wheelRadius;
            slipSpeedFloor = 1.0;
            if abs(wheelSpeed) <= eps && abs(longitudinalSpeed) <= eps
                kappa = 0;
                return;
            end
            rawKappa = (wheelSpeed - longitudinalSpeed) / ...
                max(abs(longitudinalSpeed), slipSpeedFloor);
            kappa = lts.util.clamp(rawKappa, -1, 1.5);
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
