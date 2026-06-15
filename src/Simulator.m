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
        
        % Nominal/minimum simulation timestep [s]
        dt = 0.001

        % Enable spatially adaptive timesteps during full-lap simulation.
        % Single-step calls still use obj.dt unless a step-specific dt is
        % passed to step(...). Corners, curvature transitions, and grip changes
        % use shorter steps; simple straights use longer ones.
        enableAdaptiveTimeStep = true

        % Largest timestep allowed on low-complexity track sections [s].
        adaptiveMaxTimeStep = 0.005

        % Do not let a single integration step advance farther than this [m].
        % This keeps fast straights from skipping over short grip/geometry
        % features even when the spatial complexity profile is simple.
        adaptiveMaxDistanceStep = 0.40

        % Maximum centerline heading change per step [rad] from v*kappa*dt.
        % The spatial profile already shrinks dt in corners; this runtime cap
        % catches unusually high speed in a moderate-radius bend.
        adaptiveMaxHeadingStep = 0.01

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

            if nargin < 1
                vehicleManager = [];
            end
            if nargin < 2
                driverModel = [];
            end
            obj.vehicleManager = vehicleManager;
            obj = obj.ensureVehicleManager();
            obj.sanitizeVehicleSetup();
            obj.driverModel = driverModel;
            if nargin >= 3
                obj.dt = utils.positiveScalarOrDefault(dt, obj.dt);
            end
        end
        
        function [newState, forces] = step(obj, state, throttle, brake, curKappa, curMu, curHeading, steer, stepDt)
            % STEP Progress vehicle state by one timestep
            %   [newState, forces] = step(state, throttle, brake, curKappa, curMu, curHeading, steer)
            %   [newState, forces] = step(..., stepDt)
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

            obj = obj.ensureChassis();
            vm = obj.vehicleManager;
            if nargin < 2
                state = VehicleState();
            end
            if nargin < 3
                throttle = [];
            end
            if nargin < 4
                brake = [];
            end
            if nargin < 5
                curKappa = [];
            end
            if nargin < 6
                curMu = [];
            end
            if nargin < 7
                curHeading = [];
            end
            if nargin < 8
                steer = [];
            end
            state = Simulator.normalizeInitialState(state);
            state.vehicleManager = vm;
            v = state.speed;
            throttle = utils.unitScalarOrDefault(throttle, state.throttle);
            brake = utils.unitScalarOrDefault(brake, state.brake);
            curKappa = utils.scalarOrDefault(curKappa, 0);
            curMu = utils.nonnegativeScalarOrDefault(curMu, ...
                utils.nonnegativeScalarOrDefault(state.mu, 1.2));
            curHeading = utils.scalarOrDefault(curHeading, state.heading);
            steer = utils.scalarOrDefault(steer, state.steer);
            obj.dt = utils.positiveScalarOrDefault(obj.dt, 0.001);
            if nargin >= 9 && ~isempty(stepDt)
                % obj is a value object, so this assignment only changes the
                % local integration step used by this call and its helpers.
                obj.dt = utils.positiveScalarOrDefault(stepDt, obj.dt);
            end
            
            % Copy state (will be mutated by updateFromDynamics)
            newState = state;
            
            % --- AERODYNAMIC FORCES ---
            aeroForces = vm.aero.computeForces(state);
            F_downforce = aeroForces.Fz_front + aeroForces.Fz_rear;
            F_drag = aeroForces.F_drag;
            
            % --- WEIGHT AND PER-CORNER LOADS ---
            chassisKinematics = [];
            if ~isempty(vm.chassis)
                chassisKinematics = vm.chassis.computeCornerKinematics();
            end
            
            cornerLoads = vm.suspension.computeCornerLoads( ...
                state, aeroForces.Fz_front, aeroForces.Fz_rear, vm.totalMass, obj.dt, chassisKinematics);

            % --- POWERTRAIN STATE & DRIVE FORCE ---
            vm.powertrain.updateStateFromDrivenWheels( ...
                [vm.tire.RL.angularVelocity, vm.tire.RR.angularVelocity]);
            F_drive = vm.powertrain.computeDriveForce(v, throttle);

            % --- NORMAL LOAD SUM ---
            % The four per-corner normal loads define the vertical force budget
            % available to the tires and brake system in this timestep.
            totalNormalLoad = max(cornerLoads.FL + cornerLoads.FR + ...
                cornerLoads.RL + cornerLoads.RR, 0.1);

            % --- LATERAL DEMAND ESTIMATE ---
            if abs(curKappa) > 1e-6 && v > 0.5
                ayDemand = v^2 * curKappa;
            else
                ayDemand = 0;
            end

            % --- WHEEL TORQUE SETUP ---
            % RWD assumption: drive torque only on rear wheels.
            % Brake distribution: fixed front/rear bias from VehicleManager.
            % Drive torque is split equally between the two driven wheels,
            % but brake torque is per-corner because torque = force * that
            % tire's effective rolling radius.
            R_FL = obj.getCornerWheelRadius(vm.tire.FL);
            R_FR = obj.getCornerWheelRadius(vm.tire.FR);
            R_RL = obj.getCornerWheelRadius(vm.tire.RL);
            R_RR = obj.getCornerWheelRadius(vm.tire.RR);
            rearMeanRadius = 0.5 * (R_RL + R_RR);
            T_drive_front = 0;
            T_drive_rear_requested = obj.computeDrivenWheelTorque(vm, F_drive, rearMeanRadius);
            T_drive_rear = T_drive_rear_requested;
            T_drive_rear_brakeLimit = T_drive_rear_requested;
            driveTorqueLimitActive = false;
            rpmLimitCommandActive = false;
            if ~isempty(vm.powertrain.state)
                rpmLimitCommandActive = vm.powertrain.state.rpmLimitActive;
            end

            % --- BRAKE FORCE ---
            brakeCommand = max(0, min(1, brake));
            brakeBiasFront = max(0, min(1, vm.brakeBiasFront));
            brakeBiasRear = 1 - brakeBiasFront;
            brakeForceCapacity = max(0, vm.brakeForceCoefficient) * totalNormalLoad;

            % The brake system asks for torque, but the tire decides how much
            % force actually reaches the ground. Estimate residual longitudinal
            % grip from current wheel-plane slip angles; using previous-step Fy
            % would let a sudden steering input receive full straight-line
            % braking grip until the tire model is evaluated later this step.
            brakeWheelKinematics = obj.computeBrakeWheelKinematics( ...
                state, steer, vm);
            longGrip_FL = obj.computeResidualLongitudinalGrip( ...
                vm.tire, vm.tire.FL, cornerLoads.FL, curMu, ...
                brakeWheelKinematics.FL.slipAngle);
            longGrip_FR = obj.computeResidualLongitudinalGrip( ...
                vm.tire, vm.tire.FR, cornerLoads.FR, curMu, ...
                brakeWheelKinematics.FR.slipAngle);
            longGrip_RL = obj.computeResidualLongitudinalGrip( ...
                vm.tire, vm.tire.RL, cornerLoads.RL, curMu, ...
                brakeWheelKinematics.RL.slipAngle);
            longGrip_RR = obj.computeResidualLongitudinalGrip( ...
                vm.tire, vm.tire.RR, cornerLoads.RR, curMu, ...
                brakeWheelKinematics.RR.slipAngle);

            brakeGripLimit = inf;
            if brakeBiasFront > eps
                brakeGripLimit = min(brakeGripLimit, 2 * longGrip_FL / brakeBiasFront);
                brakeGripLimit = min(brakeGripLimit, 2 * longGrip_FR / brakeBiasFront);
            end
            if brakeBiasRear > eps
                brakeGripLimit = min(brakeGripLimit, 2 * longGrip_RL / brakeBiasRear);
                brakeGripLimit = min(brakeGripLimit, 2 * longGrip_RR / brakeBiasRear);
            end

            % The brake lock prediction runs before brake torque is known. Use
            % a conservative zero-brake motor-speed cap so the lock predictor
            % does not rely on rear drive torque that the RPM limiter may later
            % remove.
            [T_drive_rear_brakeLimit, ~] = obj.limitDriveTorqueForMotorSpeed( ...
                vm, T_drive_rear_requested, 0, 0);
            brakeWheelSpeeds = obj.extractWheelPlaneSpeeds(brakeWheelKinematics);
            wheelLockLimit = obj.computeWheelLockBrakeLimit( ...
                vm, brakeWheelSpeeds, T_drive_front, T_drive_rear_brakeLimit, ...
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
            T_brake_FL = (F_brake_front / 2) * R_FL;
            T_brake_FR = (F_brake_front / 2) * R_FR;
            T_brake_RL = (F_brake_rear / 2) * R_RL;
            T_brake_RR = (F_brake_rear / 2) * R_RR;

            % Direct-drive EV gearing cannot spin the driven wheels beyond the
            % motor speed limit. Limit torque before integration instead of
            % clipping wheel speed afterward; clipping would numerically delete
            % rotational energy rather than obeying the powertrain constraint.
            [T_drive_rear, driveTorqueLimitActive] = obj.limitDriveTorqueForMotorSpeed( ...
                vm, T_drive_rear_requested, T_brake_RL, T_brake_RR);
            appliedDriveForce = obj.updateAppliedPowertrainOutput( ...
                vm, throttle, T_drive_rear, rearMeanRadius, ...
                rpmLimitCommandActive || driveTorqueLimitActive);

            % Update wheel rotational state (uses previous-timestep Fx)
            omegaTelemetry_FL = vm.tire.updateWheelDynamics( ...
                vm.tire.FL, T_drive_front, T_brake_FL, obj.dt);
            omegaTelemetry_FR = vm.tire.updateWheelDynamics( ...
                vm.tire.FR, T_drive_front, T_brake_FR, obj.dt);
            omegaTelemetry_RL = vm.tire.updateWheelDynamics( ...
                vm.tire.RL, T_drive_rear, T_brake_RL, obj.dt);
            omegaTelemetry_RR = vm.tire.updateWheelDynamics( ...
                vm.tire.RR, T_drive_rear, T_brake_RR, obj.dt);

            vm.powertrain.updateStateFromDrivenWheels( ...
                [vm.tire.RL.angularVelocity, vm.tire.RR.angularVelocity]);
            driveOmegaAvg = 0.5 * (omegaTelemetry_RL.omegaMean ...
                + omegaTelemetry_RR.omegaMean);
            if ~isempty(vm.powertrain.state)
                vm.powertrain.state.updatePowerTelemetry(driveOmegaAvg);
            end

            % Evaluate tire forces with computed per-corner slip ratios
            tireInputState = state;
            tireInputState.steer = steer;
            tireInputState.curvature = curKappa;
            tireInputState.mu = curMu;
            vm.tire.updateAllFromState(tireInputState, vm, cornerLoads, curMu, obj.dt);

            tireForces = obj.computeTireForceBalance(vm, steer);
            lateralDynamics = obj.computeLateralDynamics( ...
                vm, state, obj.dt, tireForces, aeroForces);
            ay = lateralDynamics.ay;

            % --- LONGITUDINAL FORCE BALANCE ---
            % Newton's second law is applied to the forces that the tires
            % actually generated, not directly to driver commands. The
            % powertrain and brake commands only change wheel torques; tire
            % slip, normal load, and surface mu determine the contact-patch Fx.
            F_rollResist = obj.computeRollingResistance(v, tireForces.Fx, totalNormalLoad);
            F_net_long = tireForces.Fx - F_drag - F_rollResist;
            ax = F_net_long / max(vm.totalMass, eps) + state.vy * state.yawRate;
            if v <= 0 && ax < 0
                ax = 0;
            end
            lateralDeficit = max(0, abs(ayDemand) - abs(ay));
            
            % --- INTEGRATE STATE ---
            ds = obj.integrateForwardDistance(v, ax, obj.dt);
            
            newState.throttle = throttle;
            newState.brake = effectiveBrakeCommand;
            newState.steer = steer;
            if ~isempty(vm.chassis)
                longitudinalGroundForce = tireForces.Fx - F_rollResist;
                lateralGroundForce = tireForces.Fy ...
                    + lateralDynamics.lateralDampingForce;
                vm.chassis.updateFromAccelerations( ...
                    ax, ay, aeroForces, obj.dt, longitudinalGroundForce, ...
                    lateralGroundForce);
            end
            newState = newState.updateFromDynamics(ax, ay, ds, obj.dt, ...
                curKappa, curHeading, curMu, lateralDynamics.vy, ...
                lateralDynamics.yawRate, lateralDynamics.yawAccel);
            newState = newState.updatePathTracking( ...
                vm.trackHalfWidth, obj.dt, ds, state.vy, state.yawRate);
            
            % Sanity check: warn once if speed exceeds maxSpeed
            if newState.speed > vm.maxSpeed && ~obj.warnedMaxSpeed
                obj.warnedMaxSpeed = true;
                warning('Simulator:SpeedExceeded', ...
                    'Speed (%.1f m/s / %.1f km/h) exceeded maxSpeed (%.1f m/s). Check simulation.', ...
                    newState.speed, newState.speed * 3.6, vm.maxSpeed);
            end
            
            % --- RETURN FORCES ---
            forces.dt = obj.dt;
            forces.F_downforce = F_downforce;
            forces.F_drag = F_drag;
            forces.F_drive = F_drive;
            forces.F_drive_applied = appliedDriveForce;
            forces.F_brake = F_brake;
            forces.driveWheelTorqueRequested = T_drive_rear_requested;
            forces.driveWheelTorqueForBrakeLimit = T_drive_rear_brakeLimit;
            forces.driveWheelTorqueApplied = T_drive_rear;
            forces.driveTorqueLimitActive = driveTorqueLimitActive;
            % Equal rear-wheel torque means axle power is total axle torque
            % times mean rear-wheel angular speed over this timestep.
            forces.drivePowerOmegaAvg = driveOmegaAvg;
            forces.F_tire_long = tireForces.Fx;
            forces.F_tire_lat = tireForces.Fy;
            forces.F_aero_lat = lateralDynamics.aeroSideForce;
            forces.F_lateral_damping = lateralDynamics.lateralDampingForce;
            forces.F_net_long = F_net_long;
            forces.F_rollResist = F_rollResist;
            forces.F_ground_long = tireForces.Fx - F_rollResist;
            forces.F_brake_front = -F_brake_front;
            forces.F_brake_rear = -F_brake_rear;
            forces.F_brake_FL = -F_brake_front / 2;
            forces.F_brake_FR = -F_brake_front / 2;
            forces.F_brake_RL = -F_brake_rear / 2;
            forces.F_brake_RR = -F_brake_rear / 2;
            forces.brakeTorque_FL = T_brake_FL;
            forces.brakeTorque_FR = T_brake_FR;
            forces.brakeTorque_RL = T_brake_RL;
            forces.brakeTorque_RR = T_brake_RR;
            forces.brakeOmegaBefore_FL = omegaTelemetry_FL.omegaBefore;
            forces.brakeOmegaBefore_FR = omegaTelemetry_FR.omegaBefore;
            forces.brakeOmegaBefore_RL = omegaTelemetry_RL.omegaBefore;
            forces.brakeOmegaBefore_RR = omegaTelemetry_RR.omegaBefore;
            forces.brakeOmegaUnclamped_FL = omegaTelemetry_FL.omegaUnclamped;
            forces.brakeOmegaUnclamped_FR = omegaTelemetry_FR.omegaUnclamped;
            forces.brakeOmegaUnclamped_RL = omegaTelemetry_RL.omegaUnclamped;
            forces.brakeOmegaUnclamped_RR = omegaTelemetry_RR.omegaUnclamped;
            forces.brakeOmegaAvg_FL = omegaTelemetry_FL.omegaMean;
            forces.brakeOmegaAvg_FR = omegaTelemetry_FR.omegaMean;
            forces.brakeOmegaAvg_RL = omegaTelemetry_RL.omegaMean;
            forces.brakeOmegaAvg_RR = omegaTelemetry_RR.omegaMean;
            % Brake heat power is torque times angular speed integrated over
            % the timestep. The tire model reports the mean nonnegative wheel
            % speed, including partial-timestep lockup before the zero clamp.
            forces.brakePower_FL = T_brake_FL * omegaTelemetry_FL.omegaMean;
            forces.brakePower_FR = T_brake_FR * omegaTelemetry_FR.omegaMean;
            forces.brakePower_RL = T_brake_RL * omegaTelemetry_RL.omegaMean;
            forces.brakePower_RR = T_brake_RR * omegaTelemetry_RR.omegaMean;
            forces.brakePowerTotal = forces.brakePower_FL + forces.brakePower_FR ...
                + forces.brakePower_RL + forces.brakePower_RR;
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
            forces.brakeSlipAngle_FL = brakeWheelKinematics.FL.slipAngle;
            forces.brakeSlipAngle_FR = brakeWheelKinematics.FR.slipAngle;
            forces.brakeSlipAngle_RL = brakeWheelKinematics.RL.slipAngle;
            forces.brakeSlipAngle_RR = brakeWheelKinematics.RR.slipAngle;
            forces.brakeWheelSpeed_FL = brakeWheelSpeeds.FL;
            forces.brakeWheelSpeed_FR = brakeWheelSpeeds.FR;
            forces.brakeWheelSpeed_RL = brakeWheelSpeeds.RL;
            forces.brakeWheelSpeed_RR = brakeWheelSpeeds.RR;
            forces.motorRPM = 0;
            forces.motorTorque = 0;
            forces.wheelTorque = 0;
            forces.motorPower = 0;
            forces.wheelPower = 0;
            forces.drivetrainLossPower = 0;
            forces.drivenWheelRPM = 0;
            forces.powerDrivenWheelRPM = 0;
            forces.powerMotorRPM = 0;
            forces.rpmLimitActive = false;
            if ~isempty(vm.powertrain.state)
                forces.motorRPM = vm.powertrain.state.motorRPM;
                forces.motorTorque = vm.powertrain.state.motorTorque;
                forces.wheelTorque = vm.powertrain.state.wheelTorque;
                forces.motorPower = vm.powertrain.state.motorPower;
                forces.wheelPower = vm.powertrain.state.wheelPower;
                forces.drivetrainLossPower = vm.powertrain.state.drivetrainLossPower;
                forces.drivenWheelRPM = vm.powertrain.state.drivenWheelRPM;
                forces.powerDrivenWheelRPM = ...
                    vm.powertrain.state.powerDrivenWheelAngularVelocity * 60 / (2 * pi);
                forces.powerMotorRPM = vm.powertrain.state.powerMotorRPM;
                forces.rpmLimitActive = vm.powertrain.state.rpmLimitActive;
            end
            forces.aeroFz_front = aeroForces.Fz_front;
            forces.aeroFz_rear  = aeroForces.Fz_rear;
            forces.aeroAirSpeed = utils.getStructField(aeroForces, 'airSpeed', state.speed);
            forces.aeroSideslipAngle = utils.getStructField( ...
                aeroForces, 'aeroSideslipAngle', 0);
            forces.aeroYawMoment = lateralDynamics.aeroYawMoment;
            forces.aeroRollMoment = utils.getStructField(aeroForces, 'aeroRollMoment', 0);
            forces.aeroDragHeight = utils.getStructField(aeroForces, 'dragHeight', 0);
            forces.chassisHeave = 0;
            forces.chassisPitchRate = 0;
            forces.chassisRollAngle = 0;
            forces.chassisRollRate = 0;
            forces.chassisGroundPitchMoment = 0;
            forces.chassisAeroPitchMoment = 0;
            forces.chassisDragPitchMoment = 0;
            forces.chassisGroundRollMoment = 0;
            forces.chassisAeroRollMoment = 0;
            forces.yawAccel = lateralDynamics.yawAccel;
            forces.yawMoment = lateralDynamics.yawMoment;
            forces.sideslipAngle = lateralDynamics.sideslipAngle;
            forces.ayDemand = ayDemand;
            forces.ayTire = tireForces.Fy / max(vm.totalMass, eps);
            forces.ayAero = lateralDynamics.aeroSideForce / max(vm.totalMass, eps);
            forces.ayDamping = lateralDynamics.lateralDampingForce / max(vm.totalMass, eps);
            forces.lateralDeficit = lateralDeficit;
            forces.sideslipLimitExceeded = lateralDynamics.sideslipLimitExceeded;
            forces.headingError = newState.headingError;
            forces.lateralError = newState.lateralError;
            forces.lateralErrorRate = newState.lateralErrorRate;
            forces.trackProgressSpeed = newState.trackProgressSpeed;
            if ~isempty(vm.chassis)
                forces.chassisHeave = vm.chassis.state.heave;
                forces.chassisPitchRate = vm.chassis.state.pitchRate;
                forces.chassisRollAngle = vm.chassis.state.rollAngle;
                forces.chassisRollRate = vm.chassis.state.rollRate;
                forces.chassisGroundPitchMoment = ...
                    vm.chassis.state.groundLongitudinalPitchMoment;
                forces.chassisAeroPitchMoment = vm.chassis.state.aeroPitchMoment;
                forces.chassisDragPitchMoment = vm.chassis.state.dragPitchMoment;
                forces.chassisGroundRollMoment = ...
                    vm.chassis.state.groundLateralRollMoment;
                forces.chassisAeroRollMoment = vm.chassis.state.aeroRollMoment;
            end
        end
        
        function [stateLog, lapTime] = simulate(obj, initialState, track)
            % SIMULATE Run the full lap simulation
            %   [stateLog, lapTime] = simulate(initialState, track)
            %
            %   initialState - VehicleState at simulation start
            %   track        - Track object with geometry and surface data

            obj = obj.ensureChassis();
            vm = obj.vehicleManager;
            if nargin < 2
                initialState = VehicleState();
            else
                initialState = Simulator.normalizeInitialState(initialState);
            end
            if nargin < 3 || ~Simulator.hasTrackInterface(track)
                if Simulator.hasTrackInterface(vm.track)
                    track = vm.track;
                else
                    track = components.TestTrack('straight');
                end
            end
            track = Simulator.ensureUsableSimulationTrack(track);
            % The track argument is authoritative for this run. DriverModel
            % caches preview geometry, so keep VehicleManager.track and the
            % driver's cache synchronized before any throttle/brake decisions.
            vm.track = track;
            if isempty(obj.driverModel) || ~ismethod(obj.driverModel, 'computeInputs')
                obj.driverModel = DriverModel(vm);
            elseif ismethod(obj.driverModel, 'refreshTrackGeometry')
                obj.driverModel = obj.driverModel.refreshTrackGeometry();
            end
            if ~isempty(vm.chassis)
                vm.chassis.reset();
            end
            
            % Set vehicleManager reference on state so components can access constants
            initialState.vehicleManager = vm;
            
            % Get track data
            trackPts   = track.getTrackPoints();
            curvature  = track.getCurvature();
            mu         = track.getSurfaceFriction();
            heading    = track.getHeading();

            % Compute arc-length parameterization from sanitized waypoints. A
            % public trackLength can be stale or malformed after manual edits, so
            % prefer the geometry-derived length whenever it is usable.
            arcLen = components.Track.computeArcLength(trackPts);
            derivedTrackLen = 0;
            if ~isempty(arcLen)
                derivedTrackLen = arcLen(end);
            end
            reportedTrackLen = track.getTotalLength();
            if utils.isPositiveScalar(reportedTrackLen) ...
                    && (derivedTrackLen <= eps ...
                    || abs(derivedTrackLen - reportedTrackLen) ...
                    <= max(1e-9, 1e-9 * reportedTrackLen))
                trackLen = reportedTrackLen;
            elseif derivedTrackLen > eps
                trackLen = derivedTrackLen;
            else
                trackLen = utils.positiveScalarOrDefault(reportedTrackLen, 0);
            end
            timeStepProfile = obj.buildTrackTimeStepProfile(arcLen, curvature, mu);
            [minStepDt, maxStepDt] = obj.getAdaptiveTimeStepBounds();
            
            % Pre-allocate telemetry log
            % Size telemetry from a conservative crawl speed, not the launch
            % speed. The default run starts near rest; using that tiny initial
            % speed would allocate millions of unused rows before the car has
            % a chance to accelerate. Use the smallest adaptive step for the
            % allocation so the buffer remains safe even if an entire lap is
            % complex.
            logSizingSpeed = max(initialState.speed, 5);
            maxSteps = ceil(trackLen / (logSizingSpeed * minStepDt) * 5);
            maxSteps = max(maxSteps, 100000);
            stateLog = struct( ...
                'time',        zeros(maxSteps, 1), ...
                'dt',          zeros(maxSteps, 1), ...
                's',           zeros(maxSteps, 1), ...
                'controlS',    zeros(maxSteps, 1), ...
                'speed',       zeros(maxSteps, 1), ...
                'speedKmh',    zeros(maxSteps, 1), ...
                'controlTime', zeros(maxSteps, 1), ...
                'ax',          zeros(maxSteps, 1), ...
                'ay',          zeros(maxSteps, 1), ...
                'ayDemand',    zeros(maxSteps, 1), ...
                'ayTire',      zeros(maxSteps, 1), ...
                'vy',          zeros(maxSteps, 1), ...
                'yawRate',     zeros(maxSteps, 1), ...
                'yawAccel',    zeros(maxSteps, 1), ...
                'yawMoment',   zeros(maxSteps, 1), ...
                'sideslipAngle', zeros(maxSteps, 1), ...
                'throttle',    zeros(maxSteps, 1), ...
                'brake',       zeros(maxSteps, 1), ...
                'brakeRequested', zeros(maxSteps, 1), ...
                'steer',       zeros(maxSteps, 1), ...
                'curvature',   zeros(maxSteps, 1), ...
                'trackComplexity', zeros(maxSteps, 1), ...
                'heading',     zeros(maxSteps, 1), ...
                'trackHeading', zeros(maxSteps, 1), ...
                'F_downforce', zeros(maxSteps, 1), ...
                'F_drag',      zeros(maxSteps, 1), ...
                'F_drive',     zeros(maxSteps, 1), ...
                'F_drive_applied', zeros(maxSteps, 1), ...
                'driveWheelTorqueRequested', zeros(maxSteps, 1), ...
                'driveWheelTorqueForBrakeLimit', zeros(maxSteps, 1), ...
                'driveWheelTorqueApplied', zeros(maxSteps, 1), ...
                'drivePowerOmegaAvg', zeros(maxSteps, 1), ...
                'driveTorqueLimitActive', false(maxSteps, 1), ...
                'F_brake',     zeros(maxSteps, 1), ...
                'F_tire_long', zeros(maxSteps, 1), ...
                'F_tire_lat',  zeros(maxSteps, 1), ...
                'F_aero_lat',  zeros(maxSteps, 1), ...
                'F_lateral_damping', zeros(maxSteps, 1), ...
                'F_net_long',  zeros(maxSteps, 1), ...
                'F_rollResist', zeros(maxSteps, 1), ...
                'F_ground_long', zeros(maxSteps, 1), ...
                'lateralDeficit', zeros(maxSteps, 1), ...
                'sideslipLimitExceeded', false(maxSteps, 1), ...
                'headingError', zeros(maxSteps, 1), ...
                'lateralError', zeros(maxSteps, 1), ...
                'lateralErrorRate', zeros(maxSteps, 1), ...
                'trackProgressSpeed', zeros(maxSteps, 1), ...
                'trackHalfWidth', zeros(maxSteps, 1), ...
                'onTrack', true(maxSteps, 1), ...
                'F_brake_front', zeros(maxSteps, 1), ...
                'F_brake_rear', zeros(maxSteps, 1), ...
                'F_brake_FL',  zeros(maxSteps, 1), ...
                'F_brake_FR',  zeros(maxSteps, 1), ...
                'F_brake_RL',  zeros(maxSteps, 1), ...
                'F_brake_RR',  zeros(maxSteps, 1), ...
                'brakeTorque_FL', zeros(maxSteps, 1), ...
                'brakeTorque_FR', zeros(maxSteps, 1), ...
                'brakeTorque_RL', zeros(maxSteps, 1), ...
                'brakeTorque_RR', zeros(maxSteps, 1), ...
                'brakeOmegaBefore_FL', zeros(maxSteps, 1), ...
                'brakeOmegaBefore_FR', zeros(maxSteps, 1), ...
                'brakeOmegaBefore_RL', zeros(maxSteps, 1), ...
                'brakeOmegaBefore_RR', zeros(maxSteps, 1), ...
                'brakeOmegaUnclamped_FL', zeros(maxSteps, 1), ...
                'brakeOmegaUnclamped_FR', zeros(maxSteps, 1), ...
                'brakeOmegaUnclamped_RL', zeros(maxSteps, 1), ...
                'brakeOmegaUnclamped_RR', zeros(maxSteps, 1), ...
                'brakeOmegaAvg_FL', zeros(maxSteps, 1), ...
                'brakeOmegaAvg_FR', zeros(maxSteps, 1), ...
                'brakeOmegaAvg_RL', zeros(maxSteps, 1), ...
                'brakeOmegaAvg_RR', zeros(maxSteps, 1), ...
                'brakePower_FL', zeros(maxSteps, 1), ...
                'brakePower_FR', zeros(maxSteps, 1), ...
                'brakePower_RL', zeros(maxSteps, 1), ...
                'brakePower_RR', zeros(maxSteps, 1), ...
                'brakePowerTotal', zeros(maxSteps, 1), ...
                'brakeGrip_FL', zeros(maxSteps, 1), ...
                'brakeGrip_FR', zeros(maxSteps, 1), ...
                'brakeGrip_RL', zeros(maxSteps, 1), ...
                'brakeGrip_RR', zeros(maxSteps, 1), ...
                'brakeSlipAngle_FL', zeros(maxSteps, 1), ...
                'brakeSlipAngle_FR', zeros(maxSteps, 1), ...
                'brakeSlipAngle_RL', zeros(maxSteps, 1), ...
                'brakeSlipAngle_RR', zeros(maxSteps, 1), ...
                'brakeWheelSpeed_FL', zeros(maxSteps, 1), ...
                'brakeWheelSpeed_FR', zeros(maxSteps, 1), ...
                'brakeWheelSpeed_RL', zeros(maxSteps, 1), ...
                'brakeWheelSpeed_RR', zeros(maxSteps, 1), ...
                'motorRPM',    zeros(maxSteps, 1), ...
                'motorTorque', zeros(maxSteps, 1), ...
                'wheelTorque', zeros(maxSteps, 1), ...
                'motorPower', zeros(maxSteps, 1), ...
                'wheelPower', zeros(maxSteps, 1), ...
                'drivetrainLossPower', zeros(maxSteps, 1), ...
                'drivenWheelRPM', zeros(maxSteps, 1), ...
                'powerDrivenWheelRPM', zeros(maxSteps, 1), ...
                'powerMotorRPM', zeros(maxSteps, 1), ...
                'rpmLimitActive', false(maxSteps, 1), ...
                'pitchAngle',  zeros(maxSteps, 1), ...
                'rollAngle',   zeros(maxSteps, 1), ...
                'rideHeight',  zeros(maxSteps, 1), ...
                'chassisHeave', zeros(maxSteps, 1), ...
                'chassisPitchRate', zeros(maxSteps, 1), ...
                'chassisRollRate', zeros(maxSteps, 1), ...
                'chassisGroundPitchMoment', zeros(maxSteps, 1), ...
                'chassisAeroPitchMoment', zeros(maxSteps, 1), ...
                'chassisDragPitchMoment', zeros(maxSteps, 1), ...
                'chassisGroundRollMoment', zeros(maxSteps, 1), ...
                'chassisAeroRollMoment', zeros(maxSteps, 1), ...
                'aeroAirSpeed', zeros(maxSteps, 1), ...
                'aeroSideslipAngle', zeros(maxSteps, 1), ...
                'aeroYawMoment', zeros(maxSteps, 1), ...
                'aeroRollMoment', zeros(maxSteps, 1), ...
                'aeroDragHeight', zeros(maxSteps, 1), ...
                'ayAero', zeros(maxSteps, 1), ...
                'ayDamping', zeros(maxSteps, 1), ...
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
                'antiRollForce_FL', zeros(maxSteps, 1), ...
                'antiRollForce_FR', zeros(maxSteps, 1), ...
                'antiRollForce_RL', zeros(maxSteps, 1), ...
                'antiRollForce_RR', zeros(maxSteps, 1), ...
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
                'tireUsage_FL', zeros(maxSteps, 1), ...
                'tireUsage_FR', zeros(maxSteps, 1), ...
                'tireUsage_RL', zeros(maxSteps, 1), ...
                'tireUsage_RR', zeros(maxSteps, 1), ...
                'tireFrictionLimit_FL', zeros(maxSteps, 1), ...
                'tireFrictionLimit_FR', zeros(maxSteps, 1), ...
                'tireFrictionLimit_RL', zeros(maxSteps, 1), ...
                'tireFrictionLimit_RR', zeros(maxSteps, 1), ...
                'tirePeakMuLat_FL', zeros(maxSteps, 1), ...
                'tirePeakMuLat_FR', zeros(maxSteps, 1), ...
                'tirePeakMuLat_RL', zeros(maxSteps, 1), ...
                'tirePeakMuLat_RR', zeros(maxSteps, 1), ...
                'tirePeakMuLong_FL', zeros(maxSteps, 1), ...
                'tirePeakMuLong_FR', zeros(maxSteps, 1), ...
                'tirePeakMuLong_RL', zeros(maxSteps, 1), ...
                'tirePeakMuLong_RR', zeros(maxSteps, 1), ...
                'aeroFz_front', zeros(maxSteps, 1), ...
                'aeroFz_rear',  zeros(maxSteps, 1) ...
            );
            
            % Working state (will be updated each step)
            currentState = initialState;
            obj.initializeWheelSpeeds(currentState.speed);
            
            step = 0;
            fprintf('Starting simulation...\n');
            fprintf('Track length: %.1f m\n', trackLen);
            if obj.enableAdaptiveTimeStep
                fprintf('Adaptive dt: %.4f to %.4f s\n', minStepDt, maxStepDt);
            end
            
            while currentState.s < trackLen && currentState.onTrack
                step = step + 1;
                
                % Current track properties at the actual path coordinate.
                % Using the previous waypoint as a zero-order hold injects
                % artificial curvature/heading steps into the driver and yaw
                % model. Interpolation keeps the requested path geometry
                % continuous between sampled waypoints.
                trackSample = obj.interpolateTrackSample( ...
                    currentState.s, arcLen, curvature, mu, heading);
                curKappa = trackSample.curvature;
                curMu = trackSample.mu;
                curHeading = trackSample.heading;
                [stepDt, trackComplexity] = obj.computeAdaptiveTimeStep( ...
                    currentState, trackLen, curKappa, timeStepProfile);
                
                % Set current track properties on state before DriverModel
                % reads it. VehicleState.heading is the car's heading, not the
                % track tangent alone, so rebuild it from the interpolated
                % centerline heading and the path-frame heading error carried
                % from the previous timestep.
                currentState.curvature = curKappa;
                currentState.mu        = curMu;
                currentState.heading   = curHeading + currentState.headingError;
                
                % --- DRIVER MODEL: Compute throttle and brake ---
                [throttle, brake, steer] = obj.driverModel.computeInputs(currentState);
                
                % --- PHYSICS STEP ---
                [newState, forces] = obj.step( ...
                    currentState, throttle, brake, curKappa, curMu, curHeading, steer, stepDt);
                
                % --- LOG TELEMETRY ---
                if step <= maxSteps
                    stateLog.time(step)        = newState.time;
                    stateLog.dt(step)          = forces.dt;
                    stateLog.s(step)           = newState.s;
                    stateLog.controlS(step)    = currentState.s;
                    stateLog.speed(step)       = newState.speed;
                    stateLog.speedKmh(step)    = newState.speed * 3.6;
                    stateLog.controlTime(step) = currentState.time;
                    stateLog.ax(step)          = newState.ax;
                    stateLog.ay(step)          = newState.ay;
                    stateLog.ayDemand(step)    = forces.ayDemand;
                    stateLog.ayTire(step)      = forces.ayTire;
                    stateLog.vy(step)          = newState.vy;
                    stateLog.yawRate(step)     = newState.yawRate;
                    stateLog.yawAccel(step)    = forces.yawAccel;
                    stateLog.yawMoment(step)   = forces.yawMoment;
                    stateLog.sideslipAngle(step) = newState.sideslipAngle;
                    stateLog.throttle(step)    = throttle;
                    stateLog.brake(step)       = forces.brake;
                    stateLog.brakeRequested(step) = forces.brakeCommand;
                    stateLog.steer(step)       = steer;
                    stateLog.curvature(step)   = curKappa;
                    stateLog.trackComplexity(step) = trackComplexity;
                    stateLog.heading(step)     = newState.heading;
                    stateLog.trackHeading(step) = curHeading;
                    stateLog.F_downforce(step) = forces.F_downforce;
                    stateLog.F_drag(step)      = forces.F_drag;
                    stateLog.F_drive(step)     = forces.F_drive;
                    stateLog.F_drive_applied(step) = forces.F_drive_applied;
                    stateLog.driveWheelTorqueRequested(step) = forces.driveWheelTorqueRequested;
                    stateLog.driveWheelTorqueForBrakeLimit(step) = forces.driveWheelTorqueForBrakeLimit;
                    stateLog.driveWheelTorqueApplied(step) = forces.driveWheelTorqueApplied;
                    stateLog.drivePowerOmegaAvg(step) = forces.drivePowerOmegaAvg;
                    stateLog.driveTorqueLimitActive(step) = forces.driveTorqueLimitActive;
                    stateLog.F_brake(step)     = forces.F_brake;
                    stateLog.F_tire_long(step) = forces.F_tire_long;
                    stateLog.F_tire_lat(step)  = forces.F_tire_lat;
                    stateLog.F_aero_lat(step)  = forces.F_aero_lat;
                    stateLog.F_lateral_damping(step) = forces.F_lateral_damping;
                    stateLog.F_net_long(step)  = forces.F_net_long;
                    stateLog.F_rollResist(step) = forces.F_rollResist;
                    stateLog.F_ground_long(step) = forces.F_ground_long;
                    stateLog.lateralDeficit(step) = forces.lateralDeficit;
                    stateLog.sideslipLimitExceeded(step) = forces.sideslipLimitExceeded;
                    stateLog.headingError(step) = forces.headingError;
                    stateLog.lateralError(step) = forces.lateralError;
                    stateLog.lateralErrorRate(step) = forces.lateralErrorRate;
                    stateLog.trackProgressSpeed(step) = forces.trackProgressSpeed;
                    stateLog.trackHalfWidth(step) = vm.trackHalfWidth;
                    stateLog.onTrack(step)     = newState.onTrack;
                    stateLog.F_brake_front(step) = forces.F_brake_front;
                    stateLog.F_brake_rear(step) = forces.F_brake_rear;
                    stateLog.F_brake_FL(step)  = forces.F_brake_FL;
                    stateLog.F_brake_FR(step)  = forces.F_brake_FR;
                    stateLog.F_brake_RL(step)  = forces.F_brake_RL;
                    stateLog.F_brake_RR(step)  = forces.F_brake_RR;
                    stateLog.brakeTorque_FL(step) = forces.brakeTorque_FL;
                    stateLog.brakeTorque_FR(step) = forces.brakeTorque_FR;
                    stateLog.brakeTorque_RL(step) = forces.brakeTorque_RL;
                    stateLog.brakeTorque_RR(step) = forces.brakeTorque_RR;
                    stateLog.brakeOmegaBefore_FL(step) = forces.brakeOmegaBefore_FL;
                    stateLog.brakeOmegaBefore_FR(step) = forces.brakeOmegaBefore_FR;
                    stateLog.brakeOmegaBefore_RL(step) = forces.brakeOmegaBefore_RL;
                    stateLog.brakeOmegaBefore_RR(step) = forces.brakeOmegaBefore_RR;
                    stateLog.brakeOmegaUnclamped_FL(step) = forces.brakeOmegaUnclamped_FL;
                    stateLog.brakeOmegaUnclamped_FR(step) = forces.brakeOmegaUnclamped_FR;
                    stateLog.brakeOmegaUnclamped_RL(step) = forces.brakeOmegaUnclamped_RL;
                    stateLog.brakeOmegaUnclamped_RR(step) = forces.brakeOmegaUnclamped_RR;
                    stateLog.brakeOmegaAvg_FL(step) = forces.brakeOmegaAvg_FL;
                    stateLog.brakeOmegaAvg_FR(step) = forces.brakeOmegaAvg_FR;
                    stateLog.brakeOmegaAvg_RL(step) = forces.brakeOmegaAvg_RL;
                    stateLog.brakeOmegaAvg_RR(step) = forces.brakeOmegaAvg_RR;
                    stateLog.brakePower_FL(step) = forces.brakePower_FL;
                    stateLog.brakePower_FR(step) = forces.brakePower_FR;
                    stateLog.brakePower_RL(step) = forces.brakePower_RL;
                    stateLog.brakePower_RR(step) = forces.brakePower_RR;
                    stateLog.brakePowerTotal(step) = forces.brakePowerTotal;
                    stateLog.brakeGrip_FL(step) = forces.brakeGrip_FL;
                    stateLog.brakeGrip_FR(step) = forces.brakeGrip_FR;
                    stateLog.brakeGrip_RL(step) = forces.brakeGrip_RL;
                    stateLog.brakeGrip_RR(step) = forces.brakeGrip_RR;
                    stateLog.brakeSlipAngle_FL(step) = forces.brakeSlipAngle_FL;
                    stateLog.brakeSlipAngle_FR(step) = forces.brakeSlipAngle_FR;
                    stateLog.brakeSlipAngle_RL(step) = forces.brakeSlipAngle_RL;
                    stateLog.brakeSlipAngle_RR(step) = forces.brakeSlipAngle_RR;
                    stateLog.brakeWheelSpeed_FL(step) = forces.brakeWheelSpeed_FL;
                    stateLog.brakeWheelSpeed_FR(step) = forces.brakeWheelSpeed_FR;
                    stateLog.brakeWheelSpeed_RL(step) = forces.brakeWheelSpeed_RL;
                    stateLog.brakeWheelSpeed_RR(step) = forces.brakeWheelSpeed_RR;
                    stateLog.motorRPM(step)    = forces.motorRPM;
                    stateLog.motorTorque(step) = forces.motorTorque;
                    stateLog.wheelTorque(step) = forces.wheelTorque;
                    stateLog.motorPower(step) = forces.motorPower;
                    stateLog.wheelPower(step) = forces.wheelPower;
                    stateLog.drivetrainLossPower(step) = forces.drivetrainLossPower;
                    stateLog.drivenWheelRPM(step) = forces.drivenWheelRPM;
                    stateLog.powerDrivenWheelRPM(step) = forces.powerDrivenWheelRPM;
                    stateLog.powerMotorRPM(step) = forces.powerMotorRPM;
                    stateLog.rpmLimitActive(step) = forces.rpmLimitActive;
                    stateLog.pitchAngle(step)  = newState.pitchAngle;
                    stateLog.rollAngle(step)   = newState.rollAngle;
                    stateLog.rideHeight(step)  = newState.rideHeight;
                    stateLog.chassisHeave(step) = forces.chassisHeave;
                    stateLog.chassisPitchRate(step) = forces.chassisPitchRate;
                    stateLog.chassisRollRate(step) = forces.chassisRollRate;
                    stateLog.chassisGroundPitchMoment(step) = forces.chassisGroundPitchMoment;
                    stateLog.chassisAeroPitchMoment(step) = forces.chassisAeroPitchMoment;
                    stateLog.chassisDragPitchMoment(step) = forces.chassisDragPitchMoment;
                    stateLog.chassisGroundRollMoment(step) = forces.chassisGroundRollMoment;
                    stateLog.chassisAeroRollMoment(step) = forces.chassisAeroRollMoment;
                    stateLog.aeroAirSpeed(step) = forces.aeroAirSpeed;
                    stateLog.aeroSideslipAngle(step) = forces.aeroSideslipAngle;
                    stateLog.aeroYawMoment(step) = forces.aeroYawMoment;
                    stateLog.aeroRollMoment(step) = forces.aeroRollMoment;
                    stateLog.aeroDragHeight(step) = forces.aeroDragHeight;
                    stateLog.ayAero(step) = forces.ayAero;
                    stateLog.ayDamping(step) = forces.ayDamping;
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
                    stateLog.antiRollForce_FL(step) = susp.frontLeft.state.antiRollForce;
                    stateLog.antiRollForce_FR(step) = susp.frontRight.state.antiRollForce;
                    stateLog.antiRollForce_RL(step) = susp.rearLeft.state.antiRollForce;
                    stateLog.antiRollForce_RR(step) = susp.rearRight.state.antiRollForce;
                    
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
                    stateLog.tireUsage_FL(step) = vm.tire.FL.frictionUsage;
                    stateLog.tireUsage_FR(step) = vm.tire.FR.frictionUsage;
                    stateLog.tireUsage_RL(step) = vm.tire.RL.frictionUsage;
                    stateLog.tireUsage_RR(step) = vm.tire.RR.frictionUsage;
                    stateLog.tireFrictionLimit_FL(step) = vm.tire.FL.frictionLimit;
                    stateLog.tireFrictionLimit_FR(step) = vm.tire.FR.frictionLimit;
                    stateLog.tireFrictionLimit_RL(step) = vm.tire.RL.frictionLimit;
                    stateLog.tireFrictionLimit_RR(step) = vm.tire.RR.frictionLimit;
                    stateLog.tirePeakMuLat_FL(step) = vm.tire.FL.peakMu;
                    stateLog.tirePeakMuLat_FR(step) = vm.tire.FR.peakMu;
                    stateLog.tirePeakMuLat_RL(step) = vm.tire.RL.peakMu;
                    stateLog.tirePeakMuLat_RR(step) = vm.tire.RR.peakMu;
                    stateLog.tirePeakMuLong_FL(step) = vm.tire.FL.peakMuLong;
                    stateLog.tirePeakMuLong_FR(step) = vm.tire.FR.peakMuLong;
                    stateLog.tirePeakMuLong_RL(step) = vm.tire.RL.peakMuLong;
                    stateLog.tirePeakMuLong_RR(step) = vm.tire.RR.peakMuLong;
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

        function brakeLimit = computeWheelLockBrakeLimit(obj, vm, wheelSpeeds, ...
                driveTorqueFront, driveTorqueRear, ...
                brakeBiasFront, brakeBiasRear)
            % Compute the fixed-bias brake force limit that prevents any
            % corner from crossing the configured braking slip in one step.
            %
            % Slip ratio is based on each tire's own wheel-plane speed. A
            % steered or yawing car can have front-left, front-right, and rear
            % contact patches moving at different longitudinal speeds even when
            % the CG speed is one scalar. The anti-lock brake limiter must use
            % that same local speed basis or it can allow a slow inside tire to
            % lock while the CG-speed estimate still looks safe.

            cap_FL = obj.computeCornerLockBrakeForce(vm.tire.FL, wheelSpeeds.FL, ...
                driveTorqueFront);
            cap_FR = obj.computeCornerLockBrakeForce(vm.tire.FR, wheelSpeeds.FR, ...
                driveTorqueFront);
            cap_RL = obj.computeCornerLockBrakeForce(vm.tire.RL, wheelSpeeds.RL, ...
                driveTorqueRear);
            cap_RR = obj.computeCornerLockBrakeForce(vm.tire.RR, wheelSpeeds.RR, ...
                driveTorqueRear);

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
                wheelPlaneSpeed, driveTorque)
            % COMPUTECORNERLOCKBRAKEFORCE Limit brake torque in wheel space.
            %
            % Below walking speed slip ratio is poorly defined and a real ABS
            % controller also stops regulating aggressively. Let the brake
            % command pass through; the tire model and nonnegative vehicle
            % speed clamp keep the forward-only lap simulation bounded.
            if wheelPlaneSpeed < 0.5
                brakeForceCap = inf;
                return;
            end

            wheelRadius = obj.getCornerWheelRadius(cornerState);
            wheelInertia = obj.getWheelInertia();

            maxSlip = utils.unitScalarOrDefault(obj.maxBrakeSlipRatio, 0.15);
            maxSlip = min(maxSlip, 0.95);
            minOmega = (1 - maxSlip) * max(wheelPlaneSpeed, 0) / wheelRadius;
            omega = max(cornerState.angularVelocity, 0);

            % I*domega/dt = T_drive - T_brake - Fx*R. Solve for the largest
            % brake torque that still keeps omega above the lock threshold.
            tireReactionTorque = cornerState.Fx * wheelRadius;
            dtSafe = utils.positiveScalarOrDefault(obj.dt, 0.001);
            allowableBrakeTorque = driveTorque - tireReactionTorque + ...
                (omega - minOmega) * wheelInertia / dtSafe;

            brakeForceCap = max(0, allowableBrakeTorque / wheelRadius);
        end

        function wheelSpeeds = computeWheelPlaneSpeeds(obj, state, steer, vm)
            % COMPUTEWHEELPLANESPEEDS Return local longitudinal speed per tire.
            %
            % This mirrors the kinematic convention used by the tire models for
            % slip ratio: start with CG velocity, add yaw velocity at each
            % contact patch, then project the front wheels onto their steered
            % rolling axes. These speeds feed the brake lock predictor before
            % the tire model evaluates the current step.
            wheelSpeeds = obj.extractWheelPlaneSpeeds( ...
                obj.computeBrakeWheelKinematics(state, steer, vm));
        end

        function wheelSpeeds = extractWheelPlaneSpeeds(~, wheelKinematics)
            % EXTRACTWHEELPLANESPEEDS Keep old scalar speed telemetry shape.
            wheelSpeeds.FL = wheelKinematics.FL.longitudinalSpeed;
            wheelSpeeds.FR = wheelKinematics.FR.longitudinalSpeed;
            wheelSpeeds.RL = wheelKinematics.RL.longitudinalSpeed;
            wheelSpeeds.RR = wheelKinematics.RR.longitudinalSpeed;
        end

        function wheelKinematics = computeBrakeWheelKinematics(obj, state, steer, vm)
            % COMPUTEBRAKEWHEELKINEMATICS Predict current per-corner tire input.
            %
            % Brake force limits are decided before updateAllFromState()
            % mutates the tire states for this timestep. Recomputing slip angle
            % and wheel-plane speed here gives the brake allocator the same
            % current vehicle kinematics that the tires will later evaluate.
            [deltaFL, deltaFR] = obj.computeAckermannSteer( ...
                steer, vm.wheelbase, vm.trackWidth);

            lf = vm.wheelbase * (1 - vm.staticFrontWeight);
            lr = vm.wheelbase * vm.staticFrontWeight;
            halfTrack = max(vm.trackWidth, 0) / 2;

            wheelKinematics.FL = obj.computeCornerBrakeKinematics( ...
                state.speed, state.vy, state.yawRate, lf, halfTrack, deltaFL);
            wheelKinematics.FR = obj.computeCornerBrakeKinematics( ...
                state.speed, state.vy, state.yawRate, lf, -halfTrack, deltaFR);
            wheelKinematics.RL = obj.computeCornerBrakeKinematics( ...
                state.speed, state.vy, state.yawRate, -lr, halfTrack, 0);
            wheelKinematics.RR = obj.computeCornerBrakeKinematics( ...
                state.speed, state.vy, state.yawRate, -lr, -halfTrack, 0);
        end

        function speed = computeCornerWheelPlaneSpeed(~, vx, vy, yawRate, ...
                xOffset, yOffset, steerAngle)
            % COMPUTECORNERWHEELPLANESPEED Project patch velocity onto tire x.
            wheelVx = vx - yawRate * yOffset;
            wheelVy = vy + yawRate * xOffset;
            speed = wheelVx * cos(steerAngle) + wheelVy * sin(steerAngle);
            speed = max(speed, 0);
        end

        function kinematics = computeCornerBrakeKinematics(~, vx, vy, yawRate, ...
                xOffset, yOffset, steerAngle)
            % COMPUTECORNERBRAKEKINEMATICS Match tire slip-angle convention.
            wheelVx = vx - yawRate * yOffset;
            wheelVy = vy + yawRate * xOffset;
            localLongitudinalSpeed = wheelVx * cos(steerAngle) ...
                + wheelVy * sin(steerAngle);
            localLateralSpeed = -wheelVx * sin(steerAngle) ...
                + wheelVy * cos(steerAngle);
            patchSpeed = hypot(localLongitudinalSpeed, localLateralSpeed);

            slipAngle = 0;
            if patchSpeed >= 0.1
                slipAngle = -atan2(localLateralSpeed, ...
                    max(localLongitudinalSpeed, eps));
            end

            kinematics.longitudinalSpeed = max(localLongitudinalSpeed, 0);
            kinematics.slipAngle = slipAngle;
        end

        function wheelInertia = getWheelInertia(obj)
            % GETWHEELINERTIA Return the tire/wheel rotational inertia [kg*m^2].
            tire = obj.vehicleManager.tire;
            wheelInertia = 0.5;
            if isprop(tire, 'wheelInertia')
                wheelInertia = tire.wheelInertia;
            end
            wheelInertia = max(wheelInertia, eps);
        end

        function wheelRadius = getCornerWheelRadius(~, cornerState)
            % GETCORNERWHEELRADIUS Return one corner's effective rolling radius.
            % Tire models keep radius on TireState so staggered or mismatched
            % tires still produce the correct torque arm in wheel dynamics.
            wheelRadius = cornerState.wheelRadius;
            if ~isfinite(wheelRadius) || wheelRadius <= 0
                wheelRadius = 0.241935;
            end
            wheelRadius = max(wheelRadius, eps);
        end

        function initializeWheelSpeeds(obj, vehicleSpeed)
            tire = obj.vehicleManager.tire;
            if ~isempty(obj.vehicleManager.powertrain) ...
                    && ~isempty(obj.vehicleManager.powertrain.state) ...
                    && ismethod(obj.vehicleManager.powertrain.state, 'reset')
                obj.vehicleManager.powertrain.state.reset();
            end

            obj.initializeCornerWheelSpeed(tire.FL, vehicleSpeed);
            obj.initializeCornerWheelSpeed(tire.FR, vehicleSpeed);
            obj.initializeCornerWheelSpeed(tire.RL, vehicleSpeed);
            obj.initializeCornerWheelSpeed(tire.RR, vehicleSpeed);

            if ~isempty(obj.vehicleManager.powertrain)
                obj.vehicleManager.powertrain.updateStateFromDrivenWheels( ...
                    [tire.RL.angularVelocity, tire.RR.angularVelocity]);
            end
        end

        function initializeCornerWheelSpeed(~, cornerState, vehicleSpeed)
            wheelRadius = cornerState.wheelRadius;
            if ismethod(cornerState, 'reset')
                cornerState.reset();
                cornerState.wheelRadius = wheelRadius;
            end

            cornerState.angularVelocity = max(vehicleSpeed, 0) / ...
                max(cornerState.wheelRadius, eps);
        end

        function [minDt, maxDt] = getAdaptiveTimeStepBounds(obj)
            % GETADAPTIVETIMESTEPBOUNDS Return safe timestep bounds [s].
            %
            baseDt = utils.positiveScalarOrDefault(obj.dt, 0.001);
            if utils.logicalScalarOrDefault(obj.enableAdaptiveTimeStep, true)
                % Treat obj.dt as the requested fine timestep, but never let a
                % coarse caller setting disable the stability cap used by full
                % lap simulation. Single-step calls still use obj.dt unless a
                % step-specific dt is provided.
                maxDt = utils.positiveScalarOrDefault( ...
                    obj.adaptiveMaxTimeStep, 0.005);
                minDt = max(min(baseDt, maxDt), eps);
            else
                minDt = max(baseDt, eps);
                maxDt = minDt;
            end
        end

        function profile = buildTrackTimeStepProfile(obj, arcLen, curvature, mu)
            % BUILDTRACKTIMESTEPPROFILE Convert track layout into dt zones.
            %
            % The profile is computed once before the lap starts. It is purely
            % spatial: tight corners, fast curvature changes, and surface-mu
            % transitions receive a complexity score near 1 and keep the fine
            % timestep. Long, constant-grip straights score near 0 and can use
            % the larger timestep without skipping meaningful geometry.
            [minDt, maxDt] = obj.getAdaptiveTimeStepBounds();
            profile = struct('s', 0, 'complexity', 0, 'dt', maxDt);

            arcLen = arcLen(:);
            curvature = curvature(:);
            mu = mu(:);
            if numel(curvature) == 1 && numel(arcLen) > 1
                curvature = repmat(curvature, size(arcLen));
            end
            if numel(mu) == 1 && numel(arcLen) > 1
                mu = repmat(mu, size(arcLen));
            end
            n = min([numel(arcLen), numel(curvature), numel(mu)]);
            if n == 0
                profile.dt = minDt;
                return;
            end

            arcLen = arcLen(1:n);
            curvature = curvature(1:n);
            mu = mu(1:n);
            keep = [true; diff(arcLen) > eps];
            arcLen = arcLen(keep);
            curvature = curvature(keep);
            mu = mu(keep);

            if isempty(arcLen)
                profile.dt = minDt;
                return;
            end
            if numel(arcLen) == 1 ...
                    || ~utils.logicalScalarOrDefault(obj.enableAdaptiveTimeStep, true)
                profile.s = arcLen;
                profile.complexity = zeros(size(arcLen));
                profile.dt = minDt + (maxDt - minDt) ...
                    * ones(size(arcLen));
                return;
            end

            curvatureTerm = obj.normalizeComplexitySignal(abs(curvature));
            curvatureGradient = gradient(curvature, arcLen);
            curvatureGradientTerm = obj.normalizeComplexitySignal(curvatureGradient);
            muGradient = gradient(mu, arcLen);
            muGradientTerm = obj.normalizeComplexitySignal(muGradient);

            complexity = 0.65 * curvatureTerm ...
                + 0.25 * curvatureGradientTerm ...
                + 0.10 * muGradientTerm;
            complexity = max(0, min(1, complexity));
            dtProfile = maxDt - (maxDt - minDt) * complexity;

            profile.s = arcLen;
            profile.complexity = complexity;
            profile.dt = dtProfile;
        end

        function [stepDt, complexity] = computeAdaptiveTimeStep(obj, state, ...
                trackLen, curKappa, profile)
            % COMPUTEADAPTIVETIMESTEP Pick this step's integration interval.
            %
            % The track profile gives the baseline spatial timestep. Runtime
            % caps then prevent large distance or heading jumps caused by high
            % vehicle speed. With adaptive stepping enabled, a mistakenly
            % coarse obj.dt is clamped by adaptiveMaxTimeStep for full-lap
            % simulation; direct step() calls keep their requested dt.
            [minDt, maxDt] = obj.getAdaptiveTimeStepBounds();
            stepDt = minDt;
            complexity = 0;

            if utils.logicalScalarOrDefault(obj.enableAdaptiveTimeStep, true) ...
                    && isstruct(profile) ...
                    && isfield(profile, 's') && isfield(profile, 'dt')
                profileS = profile.s(:);
                profileDt = profile.dt(:);
                profileComplexity = zeros(size(profileS));
                if isfield(profile, 'complexity')
                    profileComplexity = profile.complexity(:);
                end

                n = min([numel(profileS), numel(profileDt), ...
                    numel(profileComplexity)]);
                profileS = profileS(1:n);
                profileDt = profileDt(1:n);
                profileComplexity = profileComplexity(1:n);
                if n == 1
                    stepDt = profileDt(1);
                    complexity = profileComplexity(1);
                elseif n > 1
                    sQuery = min(max(state.s, profileS(1)), profileS(end));
                    stepDt = interp1(profileS, profileDt, sQuery, ...
                        'linear', 'extrap');
                    complexity = interp1(profileS, profileComplexity, sQuery, ...
                        'linear', 'extrap');
                end
            end

            speed = max(state.speed, 0);
            maxDistanceStep = utils.nonnegativeScalarOrDefault( ...
                obj.adaptiveMaxDistanceStep, 0.40);
            if speed > eps && maxDistanceStep > 0
                distanceLimitedDt = maxDistanceStep / speed;
                stepDt = min(stepDt, max(minDt, distanceLimitedDt));
            end

            pathYawRate = speed * abs(curKappa);
            maxHeadingStep = utils.nonnegativeScalarOrDefault( ...
                obj.adaptiveMaxHeadingStep, 0.01);
            if pathYawRate > eps && maxHeadingStep > 0
                headingLimitedDt = maxHeadingStep / pathYawRate;
                stepDt = min(stepDt, max(minDt, headingLimitedDt));
            end

            finishLineDt = [];
            if speed > eps && isfinite(trackLen)
                remainingDistance = max(trackLen - state.s, 0);
                if remainingDistance > eps
                    finishLineDt = remainingDistance / speed;
                    stepDt = min(stepDt, finishLineDt);
                end
            end

            lowerBound = minDt;
            if ~isempty(finishLineDt)
                % A shorter final step is numerically safe and avoids forcing
                % the lap to overshoot the finish line by one nominal timestep.
                lowerBound = min(minDt, max(finishLineDt, eps));
            end
            stepDt = utils.positiveScalarOrDefault(stepDt, minDt);
            stepDt = max(lowerBound, min(maxDt, stepDt));
            complexity = max(0, min(1, complexity));
        end

        function normalized = normalizeComplexitySignal(~, values)
            % NORMALIZECOMPLEXITYSIGNAL Map a signal to [0,1] robustly.
            %
            % A single noisy curvature sample should not force the whole track
            % to use the fine timestep. Scaling by the 95th percentile makes
            % normal corners visible while clipping outliers to complexity 1.
            values = abs(values(:));
            values(~isfinite(values)) = 0;
            positive = values(values > eps);
            normalized = zeros(size(values));
            if isempty(positive)
                return;
            end

            sorted = sort(positive);
            idx = max(1, min(numel(sorted), ceil(0.95 * numel(sorted))));
            scale = sorted(idx);
            if scale <= eps
                scale = max(positive);
            end
            if scale <= eps
                return;
            end

            normalized = min(values / scale, 1);
        end

        function sample = interpolateTrackSample(~, s, arcLen, curvature, mu, heading)
            % INTERPOLATETRACKSAMPLE Read continuous track properties at s.
            %
            % Track waypoints are discrete, but the vehicle can stop anywhere
            % between them. Curvature and surface mu are scalar fields along
            % arc length, so linear interpolation is appropriate. Heading is an
            % angle, so unwrap before interpolation and wrap the result back to
            % the principal range to avoid jumps near +/-pi.
            arcLen = arcLen(:);
            curvature = curvature(:);
            mu = mu(:);
            heading = heading(:);
            n = min([numel(arcLen), numel(curvature), numel(mu), numel(heading)]);
            arcLen = arcLen(1:n);
            curvature = curvature(1:n);
            mu = mu(1:n);
            heading = heading(1:n);
            if n == 0
                sample.curvature = 0;
                sample.mu = 0;
                sample.heading = 0;
                return;
            end

            % Zero-length waypoint intervals carry no physical distance and
            % make interp1 ambiguous. Keep the entry-side sample so duplicate
            % imported points do not create discontinuities or NaNs.
            keep = [true; diff(arcLen) > eps];
            arcLen = arcLen(keep);
            curvature = curvature(keep);
            mu = mu(keep);
            heading = heading(keep);

            sQuery = min(max(s, arcLen(1)), arcLen(end));
            if numel(arcLen) == 1
                sample.curvature = curvature(1);
                sample.mu = mu(1);
                sample.heading = heading(1);
                return;
            end

            sample.curvature = interp1(arcLen, curvature, sQuery, 'linear', 'extrap');
            sample.mu = interp1(arcLen, mu, sQuery, 'linear', 'extrap');

            unwrappedHeading = unwrap(heading);
            headingValue = interp1(arcLen, unwrappedHeading, sQuery, 'linear', 'extrap');
            sample.heading = atan2(sin(headingValue), cos(headingValue));
        end

        function drivenWheelTorque = computeDrivenWheelTorque(~, vm, fallbackDriveForce, fallbackRadius)
            % COMPUTEDRIVENWHEELTORQUE Return requested torque per rear wheel.
            %
            % Powertrain models report a total driven-axle wheel torque in
            % their state. Using that torque directly avoids silently changing
            % motor torque when the tire effective rolling radius differs from
            % the radius used to build a tractive-force map.
            totalWheelTorque = fallbackDriveForce * fallbackRadius;
            if ~isempty(vm.powertrain.state)
                totalWheelTorque = vm.powertrain.state.wheelTorque;
            end

            drivenWheelTorque = totalWheelTorque / 2;
        end

        function appliedDriveForce = updateAppliedPowertrainOutput( ...
                obj, vm, throttle, appliedRearWheelTorque, rearMeanRadius, rpmLimitActive)
            % UPDATEAPPLIEDPOWERTRAINOUTPUT Store the torque that reached wheels.
            %
            % computeDriveForce() asks the powertrain map for available torque,
            % but the simulator may reduce that torque to obey motor speed. The
            % shared PowertrainState is used for power and energy telemetry, so
            % it must reflect the applied axle torque rather than the request.
            if nargin < 6
                rpmLimitActive = false;
            end

            appliedAxleTorque = 2 * max(0, appliedRearWheelTorque);
            appliedDriveForce = appliedAxleTorque / max(rearMeanRadius, eps);

            if isempty(vm.powertrain) || isempty(vm.powertrain.state)
                return;
            end

            ratio = 0;
            if ismethod(vm.powertrain, 'getTotalGearRatio')
                ratio = vm.powertrain.getTotalGearRatio();
            end
            efficiency = 1;
            if ismethod(vm.powertrain, 'getDrivetrainEfficiency')
                efficiency = vm.powertrain.getDrivetrainEfficiency();
            end

            motorTorque = 0;
            if ratio > 0 && efficiency > 0
                motorTorque = appliedAxleTorque / (ratio * efficiency);
            end

            vm.powertrain.state.updateOutputs( ...
                throttle, motorTorque, appliedAxleTorque, appliedDriveForce, ...
                efficiency, rpmLimitActive);
        end

        function [limitedTorque, limitActive] = limitDriveTorqueForMotorSpeed( ...
                obj, vm, requestedTorque, brakeTorqueRL, brakeTorqueRR)
            % LIMITDRIVETORQUEFORMOTORSPEED Enforce motor RPM through torque.
            %
            % PowertrainState computes motor speed from the average driven-wheel
            % angular velocity, which is the carrier speed for a differential.
            % Enforce the same quantity here; capping each rear wheel separately
            % would be over-conservative in a turn where inside/outside wheel
            % speeds are intentionally different.
            requestedTorque = max(0, requestedTorque);
            limitedTorque = requestedTorque;
            limitActive = false;
            if nargin < 5 || isempty(brakeTorqueRR)
                brakeTorqueRR = brakeTorqueRL;
            end

            if isempty(vm.powertrain) || ~ismethod(vm.powertrain, 'getMaxDrivenWheelAngularVelocity')
                return;
            end

            maxOmega = vm.powertrain.getMaxDrivenWheelAngularVelocity();
            if ~isfinite(maxOmega) || maxOmega <= 0
                return;
            end

            axleCap = obj.computeDrivenAxleTorqueSpeedCap( ...
                vm.tire.RL, vm.tire.RR, maxOmega, brakeTorqueRL, brakeTorqueRR);

            limitedTorque = min(requestedTorque, axleCap);
            limitedTorque = max(0, limitedTorque);
            limitActive = limitedTorque < requestedTorque - 1e-9;
        end

        function torqueCap = computeDrivenAxleTorqueSpeedCap(obj, leftCorner, ...
                rightCorner, maxOmega, brakeTorqueLeft, brakeTorqueRight)
            % COMPUTEDRIVENAXLETORQUESPEEDCAP Cap equal wheel torque by motor RPM.
            %
            % Rear-wheel drive torque is applied equally in this model. Solving
            % the two wheel equations for mean(omega_new) = maxOmega gives the
            % maximum per-wheel torque that keeps the differential carrier, and
            % therefore the motor, at or below the speed limit.
            wheelInertia = obj.getWheelInertia();
            dtSafe = utils.positiveScalarOrDefault(obj.dt, 0.001);
            leftBaseOmega = obj.computeCornerOmegaWithoutDrive( ...
                leftCorner, brakeTorqueLeft, wheelInertia);
            rightBaseOmega = obj.computeCornerOmegaWithoutDrive( ...
                rightCorner, brakeTorqueRight, wheelInertia);

            meanBaseOmega = 0.5 * (leftBaseOmega + rightBaseOmega);
            torqueCap = (maxOmega - meanBaseOmega) * wheelInertia / dtSafe;
            torqueCap = max(0, torqueCap);
        end

        function omegaNoDrive = computeCornerOmegaWithoutDrive(obj, cornerState, ...
                brakeTorque, wheelInertia)
            % COMPUTECORNEROMEGAWITHOUTDRIVE Predict wheel speed without drive.
            omega = max(cornerState.angularVelocity, 0);
            R = obj.getCornerWheelRadius(cornerState);
            tireReactionTorque = cornerState.Fx * R;
            brakeDirection = sign(omega);
            if brakeDirection == 0 && brakeTorque > 0
                % The tire models use the same rule: a stopped wheel under
                % braking still has a caliper torque opposing forward rolling.
                brakeDirection = 1;
            end
            noDriveTorque = -brakeDirection * brakeTorque - tireReactionTorque;
            dtSafe = utils.positiveScalarOrDefault(obj.dt, 0.001);
            omegaNoDrive = omega + noDriveTorque / max(wheelInertia, eps) ...
                * dtSafe;
        end

        function dyn = computeLateralDynamics(obj, vm, state, dt, tireForces, aeroForces)
            % COMPUTELATERALDYNAMICS Integrate side velocity and yaw rate.
            %
            % Body-axis equation used here:
            %   sum(Fy) = m * (vy_dot + u * r)
            % where u is forward speed and r is yaw rate. Rearranging gives
            % vy_dot = sum(Fy)/m - u*r. This keeps the lateral velocity state
            % tied to real tire and aero forces instead of forcing the car to
            % match the track curvature.
            if nargin < 6 || isempty(aeroForces)
                aeroForces = struct();
            end
            aeroSideForce = utils.getStructField(aeroForces, 'F_side', 0);
            aeroYawMoment = utils.getStructField(aeroForces, 'aeroYawMoment', 0);

            % Optional lateral damping is represented as a real force term so
            % the reported acceleration and the integrated side velocity obey
            % the same Newtonian balance. Keeping it only inside vy_dot would
            % silently remove lateral momentum without any force in telemetry.
            lateralDampingForce = -max(0, vm.lateralVelocityDamping) ...
                * max(vm.totalMass, eps) * state.vy;
            ay = (tireForces.Fy + aeroSideForce + lateralDampingForce) ...
                / max(vm.totalMass, eps);
            yawMoment = tireForces.yawMoment + aeroYawMoment ...
                - vm.yawRateDamping * state.yawRate;
            yawAccel = yawMoment / max(vm.yawInertia, eps);
            yawRate = state.yawRate + yawAccel * dt;

            vyDot = ay - max(state.speed, 0) * state.yawRate;
            vy = state.vy + vyDot * dt;

            sideslipAngle = atan2(vy, max(state.speed, eps));
            sideslipLimit = max(vm.maxSideslipAngle, 0);
            sideslipLimitExceeded = sideslipLimit > 0 ...
                && abs(sideslipAngle) > sideslipLimit;

            dyn.ay = ay;
            dyn.vy = vy;
            dyn.vyDot = vyDot;
            dyn.yawRate = yawRate;
            dyn.yawAccel = yawAccel;
            dyn.yawMoment = yawMoment;
            dyn.aeroSideForce = aeroSideForce;
            dyn.aeroYawMoment = aeroYawMoment;
            dyn.lateralDampingForce = lateralDampingForce;
            dyn.sideslipAngle = sideslipAngle;
            dyn.sideslipLimitExceeded = sideslipLimitExceeded;
        end

        function tireForces = computeTireForceBalance(obj, vm, steer)
            % COMPUTETIREFORCEBALANCE Sum tire forces in the vehicle body frame.
            %
            % TireState.Fx/Fy are local to each wheel. Front tires are steered,
            % so their local force vectors must be rotated before applying
            % Newton's laws to the chassis. The yaw moment includes each
            % contact patch's x/y lever arm about the CG.
            [deltaFL, deltaFR] = obj.computeAckermannSteer( ...
                steer, vm.wheelbase, vm.trackWidth);

            lf = vm.wheelbase * (1 - vm.staticFrontWeight);
            lr = vm.wheelbase * vm.staticFrontWeight;
            halfTrack = vm.trackWidth / 2;

            FL = obj.resolveCornerTireForce(vm.tire.FL, deltaFL,  lf,  halfTrack);
            FR = obj.resolveCornerTireForce(vm.tire.FR, deltaFR,  lf, -halfTrack);
            RL = obj.resolveCornerTireForce(vm.tire.RL, 0,       -lr,  halfTrack);
            RR = obj.resolveCornerTireForce(vm.tire.RR, 0,       -lr, -halfTrack);

            tireForces.FL = FL;
            tireForces.FR = FR;
            tireForces.RL = RL;
            tireForces.RR = RR;
            tireForces.Fx = FL.Fx + FR.Fx + RL.Fx + RR.Fx;
            tireForces.Fy = FL.Fy + FR.Fy + RL.Fy + RR.Fy;
            tireForces.yawMoment = FL.yawMoment + FR.yawMoment ...
                + RL.yawMoment + RR.yawMoment;
        end

        function cornerForce = resolveCornerTireForce(~, cornerState, steerAngle, xOffset, yOffset)
            % RESOLVECORNERTIREFORCE Rotate one tire's local force into body axes.
            %
            % Body x is forward and body y is left. Positive steer rotates the
            % tire's local axes left, giving:
            %   Fx_body = Fx_tire*cos(delta) - Fy_tire*sin(delta)
            %   Fy_body = Fx_tire*sin(delta) + Fy_tire*cos(delta)
            FxBody = cornerState.Fx * cos(steerAngle) ...
                - cornerState.Fy * sin(steerAngle);
            FyBody = cornerState.Fx * sin(steerAngle) ...
                + cornerState.Fy * cos(steerAngle);

            cornerForce.Fx = FxBody;
            cornerForce.Fy = FyBody;
            cornerForce.yawMoment = xOffset * FyBody - yOffset * FxBody ...
                + cornerState.Mz;
        end

        function [deltaFL, deltaFR] = computeAckermannSteer(~, steerInput, wheelbase, trackWidth)
            % COMPUTEACKERMANNSTEER Match the front-wheel angles used by tires.
            if abs(steerInput) < 1e-6 || trackWidth <= 0 || wheelbase <= 0
                deltaFL = steerInput;
                deltaFR = steerInput;
                return;
            end

            turnRadius = wheelbase / tan(abs(steerInput));
            halfTrack = trackWidth / 2;
            innerRadius = max(turnRadius - halfTrack, 0.1);
            outerRadius = turnRadius + halfTrack;
            innerAngle = atan(wheelbase / innerRadius);
            outerAngle = atan(wheelbase / outerRadius);

            if steerInput > 0
                deltaFL = innerAngle;
                deltaFR = outerAngle;
            else
                deltaFL = -outerAngle;
                deltaFR = -innerAngle;
            end
        end

        function residualGrip = computeResidualLongitudinalGrip(~, tireModel, ...
                cornerState, normalLoad, surfaceMu, currentSlipAngle)
            % COMPUTERESIDUALLONGITUDINALGRIP Estimate braking room left in
            % the tire friction ellipse after current lateral demand.
            if normalLoad <= 0
                residualGrip = 0;
                return;
            end

            effectiveMuLat = min(max(tireModel.getPeakFriction(normalLoad), 0), ...
                max(surfaceMu, 0));
            effectiveMuLong = effectiveMuLat;
            if ismethod(tireModel, 'getPeakLongitudinalFriction')
                effectiveMuLong = min(max( ...
                    tireModel.getPeakLongitudinalFriction(normalLoad), 0), ...
                    max(surfaceMu, 0));
            end
            % Validate and clamp friction coefficients to physically reasonable range
            if ~isfinite(effectiveMuLat) || effectiveMuLat < 0
                effectiveMuLat = 0;
            end
            if ~isfinite(effectiveMuLong) || effectiveMuLong < 0
                effectiveMuLong = 0;
            end
            effectiveMuLat = min(effectiveMuLat, 3.0);
            effectiveMuLong = min(effectiveMuLong, 3.0);

            lateralLimit = effectiveMuLat * max(normalLoad, 0);
            longitudinalLimit = effectiveMuLong * max(normalLoad, 0);
            if lateralLimit <= 0 || longitudinalLimit <= 0
                residualGrip = 0;
                return;
            end

            if nargin >= 6 && ~isempty(currentSlipAngle)
                lateralForce = tireModel.computeLateralForce( ...
                    normalLoad, currentSlipAngle, surfaceMu);
            else
                lateralForce = cornerState.Fy;
            end
            if ~isfinite(lateralForce)
                lateralForce = 0;
            end

            lateralUsage = min(abs(lateralForce) / lateralLimit, 1);
            residualGrip = longitudinalLimit * sqrt(max(0, 1 - lateralUsage^2));
        end

        function F_rollResist = computeRollingResistance(obj, vehicleSpeed, tireLongForce, normalLoad)
            % COMPUTEROLLINGRESISTANCE Rolling loss opposes forward motion.
            coeff = max(0, obj.vehicleManager.rollingResistanceCoefficient);
            if vehicleSpeed > 0.05 || tireLongForce > 0
                direction = 1;
            else
                direction = 0;
            end
            F_rollResist = coeff * max(normalLoad, 0) * direction;
        end

        function ds = integrateForwardDistance(~, initialSpeed, ax, dt)
            % INTEGRATEFORWARDDISTANCE Prevent negative distance after stopping.
            initialSpeed = max(initialSpeed, 0);
            dt = utils.positiveScalarOrDefault(dt, 0);
            if dt <= 0
                ds = 0;
                return;
            end
            if ax < 0 && initialSpeed > 0
                timeToStop = -initialSpeed / ax;
                if timeToStop < dt
                    ds = initialSpeed * timeToStop + 0.5 * ax * timeToStop^2;
                    ds = max(ds, 0);
                    return;
                end
            end

            ds = max(0, initialSpeed * dt + 0.5 * ax * dt^2);
        end


        function obj = ensureChassis(obj)
            obj = obj.ensureVehicleManager();
            obj.sanitizeVehicleSetup();
            vm = obj.vehicleManager;
            if ~Simulator.hasMethod(vm.aero, 'computeForces')
                vm.aero = components.Aero.AeroManager();
            end
            if ~Simulator.hasMethod(vm.powertrain, 'updateStateFromDrivenWheels') ...
                    || ~Simulator.hasMethod(vm.powertrain, 'computeDriveForce')
                vm.powertrain = components.Powertrain.SimplePowertrain();
            end
            if ~Simulator.hasUsableTireModel(vm.tire)
                vm.tire = components.Tire.SimpleTire();
            end
            if ~Simulator.hasMethod(vm.chassis, 'computeCornerKinematics')
                vm.chassis = components.Chassis.SimpleChassis(vm);
            end
            if ~Simulator.hasMethod(vm.suspension, 'computeCornerLoads')
                vm.suspension = components.Suspension.SuspensionManager( ...
                    vm, ...
                    0.55, ...
                    45000, 3000, 4500, ...
                    42000, 2800, 4200, ...
                    0.95, ...
                    0.025, ...
                    200000, ...
                    200000, ...
                    25);
            end
            if Simulator.hasMethod(vm.suspension, 'syncVehicleGeometry')
                vm.suspension.syncVehicleGeometry(vm);
            end
            if Simulator.hasMethod(vm.chassis, 'syncVehicleGeometry')
                vm.chassis.syncVehicleGeometry(vm);
            end
            if Simulator.hasMethod(vm.chassis, 'syncSuspensionRates')
                vm.chassis.syncSuspensionRates(vm.suspension);
            end
        end

        function obj = ensureVehicleManager(obj)
            if isa(obj.vehicleManager, 'VehicleManager')
                return;
            end

            track = components.TestTrack('straight');
            tire = components.Tire.SimpleTire();
            obj.vehicleManager = VehicleManager( ...
                components.Aero.AeroManager(), ...
                [], ...
                components.Powertrain.SimplePowertrain(), ...
                tire, ...
                track);
        end

        function sanitizeVehicleSetup(obj)
            if ~isempty(obj.vehicleManager) ...
                    && ismethod(obj.vehicleManager, 'sanitizeSetup')
                obj.vehicleManager.sanitizeSetup();
            end
        end
    end

    methods (Static, Access = private)






        function valid = hasMethod(candidate, methodName)
            valid = false;
            if isempty(candidate)
                return;
            end
            try
                valid = ismethod(candidate, methodName);
            catch
                valid = false;
            end
        end

        function valid = hasUsableTireModel(candidate)
            valid = Simulator.hasMethod(candidate, 'computeLateralForce') ...
                && Simulator.hasMethod(candidate, 'computeLongitudinalForce') ...
                && Simulator.hasMethod(candidate, 'getPeakFriction') ...
                && Simulator.hasMethod(candidate, 'updateWheelDynamics') ...
                && Simulator.hasMethod(candidate, 'updateAllFromState') ...
                && Simulator.hasTireCorner(candidate, 'FL') ...
                && Simulator.hasTireCorner(candidate, 'FR') ...
                && Simulator.hasTireCorner(candidate, 'RL') ...
                && Simulator.hasTireCorner(candidate, 'RR');
        end

        function valid = hasTireCorner(tireModel, cornerName)
            valid = false;
            try
                if ~isprop(tireModel, cornerName)
                    return;
                end
                cornerState = tireModel.(cornerName);
                valid = ~isempty(cornerState) ...
                    && isprop(cornerState, 'angularVelocity') ...
                    && isprop(cornerState, 'wheelRadius') ...
                    && isprop(cornerState, 'Fx') ...
                    && isprop(cornerState, 'Fy') ...
                    && isprop(cornerState, 'Mz');
            catch
                valid = false;
            end
        end

        function valid = hasTrackInterface(candidate)
            valid = Simulator.hasMethod(candidate, 'getTrackPoints') ...
                && Simulator.hasMethod(candidate, 'getCurvature') ...
                && Simulator.hasMethod(candidate, 'getSurfaceFriction') ...
                && Simulator.hasMethod(candidate, 'getTotalLength') ...
                && Simulator.hasMethod(candidate, 'getHeading');
        end

        function track = ensureUsableSimulationTrack(track)
            if ~Simulator.hasTrackInterface(track)
                track = components.TestTrack('straight');
                return;
            end
            if Simulator.hasUsableSimulationTrackData(track)
                return;
            end

            points = Simulator.sanitizePointMatrixForSimulation( ...
                track.getTrackPoints());
            arcLen = components.Track.computeArcLength(points);
            reportedLength = utils.positiveScalarOrDefault( ...
                track.getTotalLength(), 0);
            if size(points, 1) < 2 || isempty(arcLen) || arcLen(end) <= eps
                if reportedLength <= eps
                    track = components.TestTrack('straight');
                    return;
                end
                points = [0, 0; reportedLength, 0];
                arcLen = components.Track.computeArcLength(points);
            end

            nPts = size(points, 1);
            defaultCurvature = components.Track.computeCurvature(points);
            defaultHeading = components.Track.computeHeading(points);
            curvature = Simulator.sanitizeTrackVector( ...
                track.getCurvature(), nPts, defaultCurvature, false);
            heading = Simulator.sanitizeTrackVector( ...
                track.getHeading(), nPts, defaultHeading, false);
            mu = Simulator.sanitizeTrackVector( ...
                track.getSurfaceFriction(), nPts, 1.2 * ones(nPts, 1), true);

            repairedTrack = components.TestTrack('straight');
            repairedTrack.trackPoints = points;
            repairedTrack.trackCurvature = curvature;
            repairedTrack.trackHeading = heading;
            repairedTrack.trackMu = mu;
            repairedTrack.trackLength = arcLen(end);
            track = repairedTrack;
        end

        function valid = hasUsableSimulationTrackData(track)
            valid = false;
            try
                points = Simulator.sanitizePointMatrixForSimulation( ...
                    track.getTrackPoints());
                arcLen = components.Track.computeArcLength(points);
                if size(points, 1) < 2 || isempty(arcLen) || arcLen(end) <= eps
                    return;
                end
                nPts = size(points, 1);
                valid = Simulator.hasUsableTrackVector(track.getCurvature(), nPts) ...
                    && Simulator.hasUsableTrackVector(track.getSurfaceFriction(), nPts) ...
                    && Simulator.hasUsableTrackVector(track.getHeading(), nPts);
            catch
                valid = false;
            end
        end

        function valid = hasUsableTrackVector(values, nPts)
            valid = (isnumeric(values) || islogical(values)) ...
                && isreal(values) ...
                && (numel(values) == 1 || numel(values) >= nPts) ...
                && all(isfinite(values(:)));
        end

        function values = sanitizeTrackVector(values, nPts, defaultValues, nonnegative)
            if nargin < 4
                nonnegative = false;
            end
            defaultValues = defaultValues(:);
            if numel(defaultValues) == 1
                defaultValues = repmat(defaultValues, nPts, 1);
            else
                defaultValues = defaultValues(1:min(numel(defaultValues), nPts));
                if numel(defaultValues) < nPts
                    defaultValues(end + 1:nPts, 1) = defaultValues(end);
                end
            end

            if ~(isnumeric(values) || islogical(values)) || ~isreal(values) ...
                    || isempty(values)
                values = defaultValues;
                return;
            end

            values = double(values(:));
            if numel(values) == 1
                values = repmat(values, nPts, 1);
            elseif numel(values) >= nPts
                values = values(1:nPts);
            else
                values(end + 1:nPts, 1) = values(end);
            end

            invalid = ~isfinite(values);
            if nonnegative
                invalid = invalid | values < 0;
            end
            values(invalid) = defaultValues(invalid);
        end

        function points = sanitizePointMatrixForSimulation(points)
            if ~(isnumeric(points) || islogical(points)) || ~isreal(points) ...
                    || ~ismatrix(points) || size(points, 2) < 2
                points = zeros(0, 2);
                return;
            end

            points = double(points(:, 1:2));
            if isempty(points)
                points = zeros(0, 2);
                return;
            end

            finiteRows = all(isfinite(points), 2);
            if all(finiteRows)
                return;
            end
            if ~any(finiteRows)
                points = zeros(size(points));
                return;
            end

            sampleIdx = (1:size(points, 1))';
            validIdx = sampleIdx(finiteRows);
            for colIdx = 1:2
                column = points(:, colIdx);
                column(~finiteRows) = interp1(validIdx, column(finiteRows), ...
                    sampleIdx(~finiteRows), 'linear', 'extrap');
                column(~isfinite(column)) = 0;
                points(:, colIdx) = column;
            end
        end

        function state = normalizeInitialState(candidate)
            if isa(candidate, 'VehicleState')
                state = candidate;
                return;
            end

            state = VehicleState();
            if ~isstruct(candidate) || ~isscalar(candidate)
                return;
            end

            names = fieldnames(candidate);
            for i = 1:numel(names)
                name = names{i};
                if isprop(state, name)
                    try
                        state.(name) = candidate.(name);
                    catch
                    end
                end
            end
        end
    end
end
