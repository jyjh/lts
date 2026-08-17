classdef Simulator < handle
    % Four-wheel vehicle physics and simulation orchestration.
    
    properties
        vehicleManager
        driverModel
        dt = 0.001

        steeringRampTime = 0.10

        % Semi-implicit wheel/contact iterations.
        wheelSolveIterations = 2

        telemetryMode = "full"
        verbose = true

        enforcePedalExclusivity = true
        applySteeringSlew = true
        brakeMode = "ratio"
        powertrainMode = "throttle"
        limitMotorTorqueByPackPower = false
        stopOnOffTrack = true
        stopAtTrackEnd = true
        stopTime = inf
        referenceMode = "track"

        warnedMaxSpeed = false
    end

    properties (Transient)
        % Empty/NaN means uncached.
        cachedWheelInertia = struct([])
        cachedDiffLocksWheels
        cachedTireHasBatchUpdate
        cachedSuspensionHasKinematics
        cachedDriverInputMethod
        cachedFrontArm = NaN
        cachedRearArm = NaN
        cachedRollingResistanceCoeff = NaN
        cachedPowertrainHasCoastdown
        cachedMaxSteeringAngle
        cachedSteeringRampTime
        cachedPowertrainMode = []
        cachedPowertrainModeSource = []
        cachedNextRef = struct()
    end
    
    methods
        function obj = Simulator(vehicleManager, driverModel, dt)
            obj.vehicleManager = vehicleManager;
            obj.driverModel = driverModel;
            if nargin >= 3
                obj.dt = dt;
            end
        end
        
        function [newState, forces] = step(obj, state, input, ref)
            % Advance one force-first physics step.
            
            vm = obj.vehicleManager;
            obj.requireChassis();
            input = obj.normalizeDriverInput(input, state);
            throttle = input.throttle;
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
            
            newState = state;
            
            aeroForces = vm.aero.computeForces(state);
            F_downforce = aeroForces.Fz_front + aeroForces.Fz_rear;
            F_drag = aeroForces.F_drag;
            [dragDirX, dragDirY] = obj.bodyVelocityDirection(state);
            % Components are positive magnitudes opposing velocity.
            aeroForces.F_drag_longitudinal = F_drag * dragDirX;
            aeroForces.F_drag_lateral = F_drag * dragDirY;
            
            W = vm.totalMass * vm.g;
            
            % Use the sprung-mass attitude from the previous completed step
            % to impose corner motion on the suspension. The chassis is then
            % advanced later in this step with the newly computed
            % accelerations, giving a stable one-step stagger between tire
            % loads and platform attitude.
            cornerLoads = obj.getCurrentCornerLoads(steer);
            
            % Motor speed follows the differential carrier.
            carrierOmega0 = 0.5 * (vm.tire.RL.angularVelocity + vm.tire.RR.angularVelocity);
            vm.powertrain.updateStateFromDrivenWheels(carrierOmega0);
            [totalDriveTorque, totalCoastdownTorque] = ...
                obj.computePowertrainTorques(state, input, throttle);

            R = vm.tire.RL.wheelRadius;  % all corners share same radius
            T_drive_front = 0;
            inertia = obj.getWheelInertia();  % wheel inertia plus carrier coupling
            diffOut = obj.solveDifferential( ...
                totalDriveTorque, totalCoastdownTorque, ...
                vm.tire.RL.angularVelocity, vm.tire.RR.angularVelocity, ...
                inertia.RL, obj.dt);
            T_drive_RL = diffOut.TL;
            T_drive_RR = diffOut.TR;

            totalNormalLoad = W + F_downforce;
            brakeForces = lts.simulation.BrakeForcePolicy.compute( ...
                input, totalNormalLoad, vm, obj.brakeMode);
            brakeCommand = brakeForces.requestedCommand;
            effectiveBrakeCommand = brakeForces.effectiveCommand;
            F_brake_front_cmd = brakeForces.frontForce;
            F_brake_rear_cmd = brakeForces.rearForce;
            
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

                % Re-solve after wheel speeds change.
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

                vm.powertrain.updateStateFromDrivenWheels(diffOut.carrierOmega);
                if iter < nWheelIter
                    % Feed carrier speed back into the next torque candidate.
                    [totalDriveTorque, totalCoastdownTorque] = ...
                        obj.computePowertrainTorques(state, input, throttle);
                    diffOut = obj.solveDifferential( ...
                        totalDriveTorque, totalCoastdownTorque, ...
                        vm.tire.RL.angularVelocity, vm.tire.RR.angularVelocity, ...
                        inertia.RL, obj.dt);
                    T_drive_RL = diffOut.TL;
                    T_drive_RR = diffOut.TR;
                end

                % Commit tire relaxation only on the final iteration.
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
            % Rolling resistance is already included in tire Fx.
            rollingResistanceCoeff = obj.getRollingResistanceCoeff();
            if rollingResistanceCoeff ~= 0
                corners = vm.tire;
                F_rollResist = rollingResistanceCoeff * ...
                    (corners.FL.normalForce + corners.FR.normalForce + ...
                     corners.RL.normalForce + corners.RR.normalForce);
            else
                F_rollResist = 0;
            end

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
                    state.s + dsFree, xNew, yNew, yawNew);
            else
                % Track reference mode projects the freely integrated x/y back
                % onto the centerline for progress, curvature/mu lookup, and
                % lateral-error telemetry. Projection does not overwrite x/y.
                nextRef = obj.projectToReference(xNew, yNew, ref.trackData, ref.idx);
            end
            obj.cachedNextRef = nextRef;
            
            newState.throttle = throttle;
            newState.brake = effectiveBrakeCommand;
            newState.steer = steer;
            newState = newState.updateFromPlanarDynamics( ...
                dynamics.ax, dynamics.ay, dynamics.yawAccel, ...
                vxNew, vyNew, yawRateNew, yawNew, xNew, yNew, ...
                nextRef.s, nextRef.heading, nextRef.curvature, ...
                nextRef.lateralError, obj.dt, ...
                dynamics.frontAxleAy, dynamics.rearAxleAy);
            newState.onTrack = nextRef.onTrack;
            
            if newState.speed > vm.maxSpeed && ~obj.warnedMaxSpeed
                obj.warnedMaxSpeed = true;
                warning('lts_simulation_Simulator:SpeedExceeded', ...
                    'Speed (%.1f m/s / %.1f km/h) exceeded maxSpeed (%.1f m/s). Check simulation.', ...
                    newState.speed, newState.speed * 3.6, vm.maxSpeed);
            end
            
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
            % Run a complete simulation and record telemetry.
            
            parser = inputParser;
            parser.addParameter('PreserveInitialComponentState', false, ...
                @(x) (islogical(x) || isnumeric(x)) && isscalar(x));
            parser.parse(varargin{:});

            vm = obj.vehicleManager;
            obj.resetForSimulation( ...
                logical(parser.Results.PreserveInitialComponentState));
            
            initialState.vehicleManager = vm;

            trackPtsBase  = track.getTrackPoints();
            curvatureBase = track.getCurvature();
            % Surface friction is compatibility telemetry only. Tire forces
            % are defined entirely by the tire data, so every reference point
            % carries neutral mu = 1.
            muBase        = ones(size(curvatureBase));
            headingBase   = track.getHeading();
            baseTrackLen  = track.getTotalLength();
            trackWidth    = track.getTrackWidth();

            % Preserve asymmetric side widths across repeated laps.
            [leftHalfWidthBase, rightHalfWidthBase] = ...
                obj.sideHalfWidthsFromTrack(track, size(trackPtsBase, 1), trackWidth);

            warmupLaps = lts.simulation.TrackReference.warmupLaps(track);
            recordedLaps = lts.simulation.TrackReference.recordedLaps(track);
            totalLaps = warmupLaps + recordedLaps;

            closedLoop = lts.simulation.TrackReference.isClosedLoop(track, trackPtsBase);
            if totalLaps > 1 && ~closedLoop
                error('lts_simulation_Simulator:WarmupRequiresClosedLoop', ...
                    'Track warmup/recorded laps require a closed-loop track.');
            end
            if closedLoop
                [trackPts, curvature, mu, heading] = obj.repeatClosedTrack( ...
                    trackPtsBase, curvatureBase, muBase, headingBase, totalLaps);
                hasClosurePoint = norm(trackPtsBase(1, :) - trackPtsBase(end, :)) <= 0.05;
                leftHalfWidth = lts.simulation.TrackReference.repeatClosedColumn( ...
                    leftHalfWidthBase, hasClosurePoint, totalLaps);
                rightHalfWidth = lts.simulation.TrackReference.repeatClosedColumn( ...
                    rightHalfWidthBase, hasClosurePoint, totalLaps);
            else
                trackPts = trackPtsBase;
                curvature = curvatureBase;
                mu = muBase;
                heading = headingBase;
                leftHalfWidth = leftHalfWidthBase;
                rightHalfWidth = rightHalfWidthBase;
            end
            nPts = size(trackPts, 1);
            
            dx = diff(trackPts(:,1));
            dy = diff(trackPts(:,2));
            segLen = sqrt(dx.^2 + dy.^2);
            arcLen = [0; cumsum(segLen)];
            trackLen = arcLen(end);
            recordStartS = warmupLaps * baseTrackLen;
            recordEndS = min(trackLen, recordStartS + recordedLaps * baseTrackLen);
            % Scalar fallback for consumers without asymmetric widths.
            trackHalfWidth = mean(leftHalfWidth + rightHalfWidth) / 2;
            trackData = struct( ...
                'points', trackPts, ...
                'arcLen', arcLen, ...
                'curvature', curvature(:), ...
                'mu', mu(:), ...
                'heading', heading(:), ...
                'length', trackLen, ...
                'trackWidth', trackWidth, ...
                'trackHalfWidth', trackHalfWidth, ...
                'trackLeftHalfWidth', leftHalfWidth(:), ...
                'trackRightHalfWidth', rightHalfWidth(:), ...
                'closedLoop', closedLoop, ...
                'baseTrackLength', baseTrackLen, ...
                'totalLaps', totalLaps, ...
                'lapBreakS', (0:totalLaps)' * baseTrackLen, ...
                'nPts', nPts);
            trackData = obj.precomputeTrackSegments(trackData);
            initialState = obj.initializePlanarState( ...
                initialState, trackData, obj.referenceMode);
            if ~isempty(obj.driverModel) && ...
                    ismethod(obj.driverModel, 'prepareForSimulation')
                obj.driverModel = obj.driverModel.prepareForSimulation( ...
                    initialState, trackData, obj.dt);
            end
            
            maxSteps = round(trackLen / (max(initialState.speed, 5) * obj.dt) * 5);
            if isfinite(obj.stopTime)
                maxSteps = max(maxSteps, ceil(max(0, obj.stopTime - initialState.time) / obj.dt) + 2);
            end
            maxSteps = max(maxSteps, 100000);
            leanTelemetry = obj.isLeanTelemetry();
            stateLog = localCreateStateLog(maxSteps, leanTelemetry);
            
            currentState = initialState;
            lts.simulation.DrivelineSupport.initializeWheelSpeeds( ...
                obj.vehicleManager, currentState.speed);
            if obj.isFreeReferenceMode()
                currentRef = obj.freeReferenceForState( ...
                    currentState.s, currentState.x, currentState.y, currentState.yaw);
            else
                currentRef = obj.projectToReference(currentState.x, ...
                    currentState.y, trackData, 1);
            end
            
            step = 0;
            if obj.verbose
                fprintf('Starting simulation...\n');
                fprintf('Track length: %.1f m\n', trackLen);
            end
            if warmupLaps > 0 && obj.verbose
                fprintf('Telemetry: dropping %d warmup lap(s), recording %d lap(s)\n', ...
                    warmupLaps, recordedLaps);
            end
            
            finishTolerance = 1e-6;
            hasFiniteStopTime = isfinite(obj.stopTime);
            stopTimeBound     = obj.stopTime - 0.5 * obj.dt;
            trackEndBound     = trackLen - finishTolerance;
            stopCheck = @(st) ~( ...
                (obj.stopAtTrackEnd  && st.s    >= trackEndBound) || ...
                (hasFiniteStopTime  && st.time >= stopTimeBound)  || ...
                (obj.stopOnOffTrack && ~st.onTrack));
            while stopCheck(currentState)
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

                ref = currentRef;
                ref.trackData = trackData;
                input = obj.computeDriverInput(currentState, ref);
                
                [newState, forces] = obj.step(currentState, input, ref);
                
                if step <= maxSteps
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
                        refIdxStep = find(trackData.arcLen <= newState.refS, 1, 'last');
                        if isempty(refIdxStep)
                            refIdxStep = 1;
                        end
                        refIdxStep = max(1, min(refIdxStep, ...
                            numel(trackData.trackLeftHalfWidth)));
                        % Track width remains a scalar telemetry channel.
                        stateLog.trackWidth(step) = ...
                            trackData.trackLeftHalfWidth(refIdxStep) + ...
                            trackData.trackRightHalfWidth(refIdxStep);
                        [localLeft, localRight] = ...
                            lts.simulation.TrackReference.sideHalfWidthsAt( ...
                            trackData, refIdxStep);
                        stateLog.trackLimitMargin(step) = ...
                            lts.simulation.TrackReference.sideMargin( ...
                            localLeft, localRight, newState.lateralError, ...
                            trackData.trackHalfWidth);
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
                    stateLog = localLogCornerTelemetry( ...
                        stateLog, step, vm, leanTelemetry);
                end
                
                currentState = newState;
                currentRef = obj.cachedNextRef;
                
                if obj.verbose && mod(step, 5000) == 0
                    progress = obj.simulationProgress(currentState, input, trackLen);
                    fprintf('  Progress: %5.1f%% | Speed: %5.1f km/h | s: %6.1f m | replay: %s\n', ...
                        progress * 100, currentState.speed * 3.6, ...
                        currentState.s, obj.replayProgressText(input));
                end
                
                if step >= maxSteps
                    warning('Simulation reached maximum steps (%d). Stopping.', maxSteps);
                    break;
                end
            end
            
            simulationSteps = step;

            fields = fieldnames(stateLog);
            for i = 1:numel(fields)
                stateLog.(fields{i}) = stateLog.(fields{i})(1:step);
            end

            % Replay-bounded runs use their complete time-domain window.
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
            
            if obj.verbose
                fprintf('\n=== Simulation Complete ===\n');
                fprintf('Lap Time:   %.3f s\n', lapTime);
                fprintf('Track Length: %.1f m\n', recordedLength);
                fprintf('Max Speed:  %.1f km/h\n', maxSpeedKmh);
                fprintf('Steps:      %d simulated, %d recorded\n', ...
                    simulationSteps, recordedSteps);
            end
        end

        function [stateLog, lapTime] = simulateReplay(obj, initialState, track, replayProfile, varargin)
            % Run with a replay profile object or normalized replay CSV.
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
            cleanup = onCleanup(@() obj.restoreReplayPolicies( ...
                previousDriver, previousMethod, previousPedalPolicy, ...
                previousSteerPolicy, previousBrakeMode, previousOffTrackPolicy, ...
                previousTrackEndPolicy, previousStopTime, ...
                previousReferenceMode, previousPowertrainMode, ...
                previousLimitMotorTorqueByPackPower));

            obj.driverModel = lts.correlation.TelemetryReplayDriver(profile, ...
                'ReplayDomain', parser.Results.ReplayDomain);
            obj.cachedDriverInputMethod = [];
            obj.enforcePedalExclusivity = ~logical(parser.Results.AllowPedalOverlap);
            obj.applySteeringSlew = logical(parser.Results.ApplySteeringSlew);
            obj.brakeMode = lts.simulation.BrakeForcePolicy.validateMode( ...
                parser.Results.BrakeMode);
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

            channels = { ...
                'replayThrottle','throttle'; 'replayBrake','brake'; ...
                'replaySteer','steer'; 'replaySpeed','speed'; ...
                'replayYawRate','yawRate'};
            for i = 1:size(channels, 1)
                stateLog.(channels{i, 1}) = localInterpProfileChannel( ...
                    profile.time, profile.(channels{i, 2}), queryTime);
            end
            if profile.hasBrakePressure()
                channels = { ...
                    'replayBrakePressureFrontBar','brakePressureFrontBar'; ...
                    'replayBrakePressureRearBar','brakePressureRearBar'};
                for i = 1:size(channels, 1)
                    stateLog.(channels{i, 1}) = localInterpProfileChannel( ...
                        profile.time, profile.(channels{i, 2}), queryTime);
                end
            end
            optionalChannels = { ...
                profile.hasRegenTorque(),'replayRegenTorqueNm','regenTorqueNm'; ...
                profile.hasMotorTorqueCommand(),'replayMotorTorqueCommandNm','motorTorqueCommandNm'; ...
                profile.hasMotorTorqueDelivered(),'replayMotorTorqueDeliveredNm','motorTorqueDeliveredNm'; ...
                profile.hasMotorRpm(),'replayMotorRpm','motorRpm'};
            for i = 1:size(optionalChannels, 1)
                if optionalChannels{i, 1}
                    stateLog.(optionalChannels{i, 2}) = localInterpProfileChannel( ...
                        profile.time, profile.(optionalChannels{i, 3}), queryTime);
                end
            end
            if profile.hasPackPower()
                channels = { ...
                    'replayPackVoltageV','packVoltageV'; ...
                    'replayPackCurrentA','packCurrentA'};
                for i = 1:size(channels, 1)
                    stateLog.(channels{i, 1}) = localInterpProfileChannel( ...
                        profile.time, profile.(channels{i, 2}), queryTime);
                end
                stateLog.replayPackPowerW = ...
                    stateLog.replayPackVoltageV .* stateLog.replayPackCurrentA;
            end
            if profile.hasWheelSpeeds()
                corners = {'FL','FR','RL','RR'};
                for i = 1:numel(corners)
                    corner = corners{i};
                    replayField = ['replayWheelSpeed' corner];
                    simulatedField = ['tireSpeed_' corner];
                    stateLog.(replayField) = localInterpProfileChannel( ...
                        profile.time, profile.(['wheelSpeed' corner]), queryTime);
                    if isfield(stateLog, simulatedField)
                        stateLog.(['wheelSpeedError' corner]) = ...
                            stateLog.(simulatedField) - stateLog.(replayField);
                    end
                end
            end
            if profile.hasLatAccel()
                channels = { ...
                    'replayLatAccelG','latAccelG'; ...
                    'replayFrontLatAccelG','frontLatAccelG'; ...
                    'replayRearLatAccelG','rearLatAccelG'};
                for i = 1:size(channels, 1)
                    stateLog.(channels{i, 1}) = localInterpProfileChannel( ...
                        profile.time, profile.(channels{i, 2}), queryTime);
                end
            end
            if profile.hasLongAccel()
                channels = { ...
                    'replayLongAccelG','longAccelG'; ...
                    'replayFrontLongAccelG','frontLongAccelG'; ...
                    'replayRearLongAccelG','rearLongAccelG'};
                for i = 1:size(channels, 1)
                    stateLog.(channels{i, 1}) = localInterpProfileChannel( ...
                        profile.time, profile.(channels{i, 2}), queryTime);
                end
            end
        end

        function restoreReplayPolicies(obj, driverModel, inputMethod, pedalPolicy, ...
                steerPolicy, brakeMode, offTrackPolicy, trackEndPolicy, stopTime, ...
                referenceMode, powertrainMode, limitMotorTorqueByPackPower)
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
                obj.powertrainMode = powertrainMode;
            end
            if nargin >= 12
                obj.limitMotorTorqueByPackPower = limitMotorTorqueByPackPower;
            end
            obj.cachedPowertrainMode = [];
            obj.cachedPowertrainModeSource = [];
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

        function [totalDriveTorque, totalCoastdownTorque] = computePowertrainTorques(obj, state, input, throttle)
            vm = obj.vehicleManager;
            % Cache the normalized mode for the run.
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

        function state = initializePlanarState(~, state, trackData, referenceMode)
            if nargin < 4 || isempty(referenceMode)
                referenceMode = "track";
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
                state.mu = 1;
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

        function ref = freeReferenceForState(~, s, x, y, yaw)
            ref = struct( ...
                'idx', 1, ...
                's', max(0, s), ...
                'x', x, ...
                'y', y, ...
                'heading', yaw, ...
                'curvature', 0, ...
                'mu', 1, ...
                'lateralError', 0, ...
                'trackWidth', 0, ...
                'trackHalfWidth', 0, ...
                'leftHalfWidth', 0, ...
                'rightHalfWidth', 0, ...
                'trackLimitMargin', 0, ...
                'onTrack', true, ...
                'referenceMode', 'free');
        end

        function tf = isLeanTelemetry(obj)
            tf = lower(string(obj.telemetryMode)) == "lean";
        end

        function stateLog = createLeanStateLog(~, maxSteps)
            stateLog = localCreateStateLog(maxSteps, true);
        end

        function [points, curvature, mu, heading] = repeatClosedTrack(~, ...
                points, curvature, mu, heading, lapCount)
            [points, curvature, mu, heading] = ...
                lts.simulation.TrackReference.repeatClosed(points, curvature, mu, heading, lapCount);
        end

        function [leftHalfWidth, rightHalfWidth] = sideHalfWidthsFromTrack(~, ...
                track, nWaypoints, trackWidth)
            % Return asymmetric widths or broadcast the scalar track width.
            if ismethod(track, 'getTrackSideWidths')
                [leftHalfWidth, rightHalfWidth] = track.getTrackSideWidths();
                leftHalfWidth = leftHalfWidth(:);
                rightHalfWidth = rightHalfWidth(:);
            else
                halfWidth = trackWidth / 2;
                leftHalfWidth = repmat(halfWidth, nWaypoints, 1);
                rightHalfWidth = leftHalfWidth;
            end
        end

        function [stateLog, lapTime, recordedSteps] = applyTelemetryLapWindow(~, ...
                stateLog, recordStartS, recordEndS)
            [stateLog, lapTime, recordedSteps] = ...
                lts.simulation.TelemetryWindow.apply(stateLog, recordStartS, recordEndS);
        end

        function input = computeDriverInput(obj, state, observation)
            if isempty(obj.driverModel)
                input = struct('throttle', 0, 'brake', 0, 'steer', 0);
                return;
            end

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

        function ref = projectToReference(~, x, y, trackData, previousIdx)
            ref = lts.simulation.TrackReference.projectToReference( ...
                x, y, trackData, previousIdx);
        end

        function tireData = updatePlanarTireForces(obj, state, cornerLoads, dt, computePeakMu, relaxationMode, tireContact)
            % Evaluate four tire contacts and assemble body forces.
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
                    computePeakMu, relaxationMode);
            else
                % Fallback for tire models without a batch update: evaluate
                % each corner individually. Wheel omega is integrated in the
                % main step() wheel-contact solve, not here.
                vm.tire.updateCorner(tireFL, cornerLoads.FL, slipAngles(1), ...
                    slipRatios(1), tireContact.camberAngles(1), dt, ...
                    longSpeedVec(1), computePeakMu, relaxationMode);
                vm.tire.updateCorner(tireFR, cornerLoads.FR, slipAngles(2), ...
                    slipRatios(2), tireContact.camberAngles(2), dt, ...
                    longSpeedVec(2), computePeakMu, relaxationMode);
                vm.tire.updateCorner(tireRL, cornerLoads.RL, slipAngles(3), ...
                    slipRatios(3), tireContact.camberAngles(3), dt, ...
                    longSpeedVec(3), computePeakMu, relaxationMode);
                vm.tire.updateCorner(tireRR, cornerLoads.RR, slipAngles(4), ...
                    slipRatios(4), tireContact.camberAngles(4), dt, ...
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
            % Read chassis-driven tire loads from the suspension.
            vm = obj.vehicleManager;
            loads = vm.suspension.computeCornerLoadsFromChassis( ...
                vm.chassis, steer, obj.dt);
        end

        function resetForSimulation(obj, preserveInitialComponentState)
            % Reset cached and component state for a new run.
            if nargin < 2
                preserveInitialComponentState = false;
            end
            obj.warnedMaxSpeed = false;
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
            speed = hypot(state.vx, state.vy);
            if speed > eps
                dirX = state.vx / speed;
                dirY = state.vy / speed;
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
            % Integrate in world space, then return to the new body frame.
            vx0 = state.vx;
            vy0 = state.vy;
            yaw0 = state.yaw;
            yawRate0 = state.yawRate;

            yawRateNew = yawRate0 + dynamics.yawAccel * dt;
            yawNew = yaw0 + yawRate0 * dt + 0.5 * dynamics.yawAccel * dt^2;
            yawMid = yaw0 + 0.5 * yawRate0 * dt + 0.125 * dynamics.yawAccel * dt^2;

            cy0 = cos(yaw0); sy0 = sin(yaw0);
            cyNew = cos(yawNew); syNew = sin(yawNew);
            % First-order mid-yaw update; dt keeps the angle increment small.
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

        function inertia = getWheelInertia(obj)
            if ~isempty(obj.cachedWheelInertia)
                inertia = obj.cachedWheelInertia;
                return;
            end
            inertia = lts.simulation.DrivelineSupport.wheelInertia(obj.vehicleManager);
            obj.cachedWheelInertia = inertia;
        end

        function out = solveDifferential(obj, totalDriveTorque, ...
                totalCoastdownTorque, omegaL, omegaR, wheelInertia, stepDt)
            out = lts.simulation.DrivelineSupport.solveDifferential( ...
                obj.vehicleManager, totalDriveTorque, totalCoastdownTorque, ...
                omegaL, omegaR, wheelInertia, stepDt);
        end

        function locked = differentialLocksWheels(obj)
            % Cache whether the differential locks wheel speeds.
            if ~isempty(obj.cachedDiffLocksWheels)
                locked = obj.cachedDiffLocksWheels;
                return;
            end
            locked = lts.simulation.DrivelineSupport.locksWheels(obj.vehicleManager);
            obj.cachedDiffLocksWheels = locked;
        end

    end
end

function stateLog = localCreateStateLog(n, lean)
zeroFields = { ...
    'time','s','controlS','x','y','yaw','vx','vy','bodySlipAngle', ...
    'speed','speedKmh','controlTime','ax','ay','frontAxleAy','rearAxleAy', ...
    'yawRate','yawAccel','refS','refHeading','refCurvature','lateralError', ...
    'trackWidth','trackLimitMargin','throttle','brake','brakeRequested','steer', ...
    'curvature','heading','F_downforce','F_drag','F_drive','F_brake', ...
    'F_tire_long','F_tire_lat','yawMoment','rollResistance','F_brake_front', ...
    'F_brake_rear','F_brake_FL','F_brake_FR','F_brake_RL','F_brake_RR', ...
    'brakeGrip_FL','brakeGrip_FR','brakeGrip_RL','brakeGrip_RR', ...
    'driveTorqueTotal','driveTorque_RL','driveTorque_RR','brakeTorque_FL', ...
    'brakeTorque_FR','brakeTorque_RL','brakeTorque_RR','motorRPM','motorTorque', ...
    'motorTorqueRequested','wheelTorque','drivenWheelRPM','pitchAngle','rollAngle', ...
    'rollRate','frontRollAngle','rearRollAngle','frontRollRate','rearRollRate', ...
    'twistAngle','twistRate','rideHeight','aeroFz_front','aeroFz_rear', ...
    'tireSpeed_FL','tireSpeed_FR','tireSpeed_RL','tireSpeed_RR'};
nanFields = { ...
    'brakePressureFrontBar','brakePressureRearBar','targetSpeed','axRef', ...
    'targetLateralError','lineCurvature','speedError','motorTorquePowerLimitNm', ...
    'packVoltageV','packCurrentA','packPowerW'};
logicalFields = { ...
    'onTrack','brakePressureMode','motorTorquePowerLimitActive','rpmLimitActive'};

stateLog = struct();
for i = 1:numel(zeroFields)
    stateLog.(zeroFields{i}) = zeros(n, 1);
end
for i = 1:numel(nanFields)
    stateLog.(nanFields{i}) = NaN(n, 1);
end
for i = 1:numel(logicalFields)
    stateLog.(logicalFields{i}) = false(n, 1);
end

if lean
    return;
end
corners = {'FL','FR','RL','RR'};
cornerFields = { ...
    'Fz','suspensionForce','antiRollBarForce','suspensionDemand','tireDeflection', ...
    'damperPos','damperVel','sprungPosition','unsprungPosition','sprungVelocity', ...
    'unsprungVelocity','wheelTravel','camber','toe','wheelSteer','slipAngle', ...
    'slipRatio','peakMu','tireUtilization','omega','tireFx','tireFy'};
for i = 1:numel(cornerFields)
    for j = 1:numel(corners)
        stateLog.([cornerFields{i} '_' corners{j}]) = zeros(n, 1);
    end
end
end

function stateLog = localLogCornerTelemetry(stateLog, step, vm, lean)
corners = {'FL','FR','RL','RR'};
suspensionCorners = {'frontLeft','frontRight','rearLeft','rearRight'};
suspensionFields = { ...
    'Fz','tireNormalForce'; 'suspensionForce','suspensionForce'; ...
    'antiRollBarForce','antiRollBarForce'; 'suspensionDemand','demandedLoad'; ...
    'tireDeflection','tireDeflection'; 'damperPos','damperPosition'; ...
    'damperVel','damperVelocity'; 'sprungPosition','sprungPosition'; ...
    'unsprungPosition','unsprungPosition'; 'sprungVelocity','sprungVelocity'; ...
    'unsprungVelocity','unsprungVelocity'; 'wheelTravel','wheelTravel'; ...
    'camber','camberAngle'; 'toe','toeAngle'; 'wheelSteer','steerAngle'};
tireFields = { ...
    'slipAngle','slipAngle'; 'slipRatio','slipRatio'; 'peakMu','peakMu'; ...
    'omega','angularVelocity'; 'tireFx','Fx'; 'tireFy','Fy'};

for j = 1:numel(corners)
    corner = corners{j};
    tire = vm.tire.(corner);
    stateLog.(['tireSpeed_' corner])(step) = ...
        abs(tire.angularVelocity * tire.wheelRadius);
    if lean
        continue;
    end

    suspension = vm.suspension.(suspensionCorners{j}).state;
    for i = 1:size(suspensionFields, 1)
        stateLog.([suspensionFields{i, 1} '_' corner])(step) = ...
            suspension.(suspensionFields{i, 2});
    end
    for i = 1:size(tireFields, 1)
        stateLog.([tireFields{i, 1} '_' corner])(step) = ...
            tire.(tireFields{i, 2});
    end
    capacity = max(tire.peakMu, 0) * max(tire.normalForce, 0);
    utilization = 0;
    if capacity > eps
        utilization = hypot(tire.Fx, tire.Fy) / capacity;
        if ~isfinite(utilization)
            utilization = 0;
        end
    end
    stateLog.(['tireUtilization_' corner])(step) = utilization;
end
end

function value = localGetField(s, fieldName, defaultValue)
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
elseif isscalar(axis)
    values = repmat(channel(1), size(query));
else
    [axis, uniqueIdx] = unique(axis, 'stable');
    channel = channel(uniqueIdx);
    query = max(axis(1), min(axis(end), query));
    values = interp1(axis, channel, query, 'linear');
end
end
