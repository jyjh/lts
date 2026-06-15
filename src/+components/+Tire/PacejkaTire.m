classdef PacejkaTire < components.Tire.TireModel
    % PACEJKATIRE Pacejka Magic Formula tire model via MFeval (4-corner manager)
    %
    % Manages four per-corner TireState objects (FL, FR, RL, RR), each with
    % independent inputs (slip angle, slip ratio, camber, normal load) and
    % outputs (Fx, Fy, Mz, etc.). All corners share a single TireConstants
    % object that holds the parsed .tir file coefficients.
    %
    % Architecture mirrors SuspensionManager:
    %   TireConstants — shared immutable Pacejka coefficients (like suspension params)
    %   TireState     — per-corner mutable state (like SuspensionState)
    %   PacejkaTire   — manager that creates states and evaluates MFeval
    %
    % Dependencies:
    %   MFeval toolbox — https://www.mathworks.com/matlabcentral/fileexchange/63618-mfeval
    %
    % Usage:
    %   tire = components.Tire.PacejkaTire('43105_18x7.5_10_R25B_7.tir')
    %   tire.updateCorner(tire.FL, Fz, alpha, kappa, gamma, mu)
    %   tire.FL.Fy   % lateral force on front-left
    %   mu = tire.getPeakFriction(Fz)

    properties
        % Shared tire coefficients (from .tir file)
        tireConstants

        % Per-corner tire state objects (handle objects, mutated in-place)
        FL   % TireState — front-left
        FR   % TireState — front-right
        RL   % TireState — rear-left
        RR   % TireState — rear-right

        % Wheel rotational inertia per corner [kg·m^2]
        % (wheel + tire + brake disc rotating assembly)
        wheelInertia = 0.5

        % Minimum normal load passed into MFeval [N]. Returned forces are
        % scaled back to the actual normal load for lightly loaded tires.
        minEvaluationLoad = 100

        % Tire relaxation lengths. These make slip angle/ratio build over
        % distance instead of appearing instantly at the contact patch.
        enableRelaxation = true
        lateralRelaxationLength = 2.5      % [m]
        longitudinalRelaxationLength = 0.8 % [m]

        % Peak-mu scans are expensive MFeval calls used heavily by driver
        % planning. Cache them by rounded load so repeated preview estimates
        % reuse the same tire curve scans.
        peakMuCacheLoadResolution = 25     % [N]
        peakMuLatCacheLoads = []
        peakMuLatCacheValues = []
        peakMuLongCacheLoads = []
        peakMuLongCacheValues = []

        % Some tire files include force/moment offsets at zero slip. In this
        % vehicle model a symmetric straight-running tire should not generate
        % net drive, lateral force, or yaw moment at kappa=alpha=gamma=0, so
        % subtract those offsets before forces reach the chassis.
        zeroSlipOffsetLoads = []
        zeroSlipOffsetCambers = []
        zeroSlipOffsetOutputs = []
    end

    methods
        function obj = PacejkaTire(tirFilePath)
            % PACEJKATIRE Construct from a .tir file, creating 4 corner states
            %   PacejkaTire(tirFilePath)
            %
            %   tirFilePath — path to the .tir file. If relative, resolved
            %                 relative to the +Tire/ folder.

            % Load shared tire constants
            obj.tireConstants = components.Tire.TireConstants(tirFilePath);

            % Create per-corner state objects
            obj.FL = components.Tire.TireState();
            obj.FR = components.Tire.TireState();
            obj.RL = components.Tire.TireState();
            obj.RR = components.Tire.TireState();

            fprintf('  PacejkaTire: 4 corner states created (FL, FR, RL, RR)\n');
        end

        %% ---- Per-corner evaluation ----

        function updateCorner(obj, cornerState, normalLoad, slipAngle, slipRatio, camberAngle, mu)
            % UPDATECORNER Evaluate MFeval for one corner and update its state
            %   updateCorner(cornerState, normalLoad, slipAngle, slipRatio, camberAngle, mu)
            %
            %   cornerState  — TireState handle for this corner
            %   normalLoad   — Tire normal force Fz [N]
            %   slipAngle    — Tire slip angle alpha [rad]
            %   slipRatio    — Tire slip ratio kappa [-1 to 1]
            %   camberAngle  — Inclination angle gamma [rad]
            %   mu           — Surface friction multiplier (1.0 = nominal)
            %
            %   In this codebase, mu is treated as an absolute surface grip cap.
            %   Mutates cornerState in-place with computed forces and moments.

	            normalLoad = utils.nonnegativeScalarOrDefault(normalLoad, 0);
	            slipAngle = utils.scalarOrDefault(slipAngle, 0);
	            slipRatio = utils.unitSignedScalarOrDefault(slipRatio, 0);
	            camberAngle = utils.scalarOrDefault(camberAngle, 0);
	            mu = utils.nonnegativeScalarOrDefault(mu, 0);
	            minEvaluationLoad = utils.positiveScalarOrDefault( ...
	                obj.minEvaluationLoad, 100);

	            % Store inputs
	            cornerState.normalForce = normalLoad;
            cornerState.slipAngle   = slipAngle;
            cornerState.slipRatio   = slipRatio;
            cornerState.camberAngle = camberAngle;

            if normalLoad <= 0
                cornerState.Fy = 0;
                cornerState.Fx = 0;
                cornerState.Mx = 0;
                cornerState.My = 0;
                cornerState.Mz = 0;
                cornerState.peakMu = 0;
                cornerState.peakMuLong = 0;
                cornerState.frictionLimit = 0;
                cornerState.frictionUsage = 0;
                return;
            end

            % Unpack for MFeval call
            kappa = slipRatio;
            alpha = slipAngle;
	            Fz    = max(normalLoad, minEvaluationLoad);
            gamma = camberAngle;
            V     = obj.tireConstants.refVelocity;
            P     = obj.tireConstants.nomPressure;
            params = obj.tireConstants.params;
            loadScale = normalLoad / Fz;

            % Build MFeval inputs rows: [Fz, kappa, alpha, gamma, phit, Vx, P]
            % Row 1 is the requested combined-slip state. Row 2 keeps the
            % same longitudinal slip but removes lateral slip so Fy/Mz offsets
            % from pure drive/brake slip can be subtracted. Row 3 keeps the
            % same lateral slip but removes longitudinal slip so Fx offsets
            % from pure cornering can be subtracted.
            inputsMF = [
                Fz, kappa, alpha, gamma, 0, V, P
                Fz, kappa, 0,     gamma, 0, V, P
                Fz, 0,     alpha, gamma, 0, V, P
            ];

            % Evaluate Pacejka Magic Formula via MFeval (useMode=111: combined)
            outputsAll = mfeval(params, inputsMF, 111);
            outputs = outputsAll(1, :);
            zeroAlphaOutput = outputsAll(2, :);
            zeroKappaOutput = outputsAll(3, :);

            rawPeakMuLat = obj.computePeakMuInternal(Fz, gamma, P, params);
            rawPeakMuLong = obj.computePeakLongitudinalMuInternal(Fz, gamma, P, params);
            surfaceScaleLat = obj.computeSurfaceScale(rawPeakMuLat, mu);
            surfaceScaleLong = obj.computeSurfaceScale(rawPeakMuLong, mu);
            momentScale = min(surfaceScaleLat, surfaceScaleLong);

            % Store outputs capped by the current surface friction coefficient.
            % Longitudinal and lateral peaks in a real tire file are not
            % necessarily equal. Scale each force axis against its own pure-slip
            % peak, then use an ellipse as the final surface cap. Moments do not
            % map cleanly to one axis, so they use the more conservative scale.
            cornerState.Fy = -(outputs(:,2) - zeroAlphaOutput(:,2)) ...
                * surfaceScaleLat * loadScale;
            cornerState.Fx = (outputs(:,1) - zeroKappaOutput(:,1)) ...
                * surfaceScaleLong * loadScale;
            cornerState.Mx = outputs(:,4) * momentScale * loadScale;
            cornerState.My = outputs(:,5) * momentScale * loadScale;
            cornerState.Mz = (outputs(:,6) - zeroAlphaOutput(:,6)) ...
                * momentScale * loadScale;
            cornerState.peakMu = rawPeakMuLat * surfaceScaleLat;
            cornerState.peakMuLong = rawPeakMuLong * surfaceScaleLong;

            [usage, directionalLimit] = obj.computeFrictionEllipseUsage( ...
                cornerState.Fx, cornerState.Fy, normalLoad, ...
                cornerState.peakMuLong, cornerState.peakMu);
            if usage > 1
                limitScale = 1 / usage;
                cornerState.Fx = cornerState.Fx * limitScale;
                cornerState.Fy = cornerState.Fy * limitScale;
                cornerState.Mx = cornerState.Mx * limitScale;
                cornerState.My = cornerState.My * limitScale;
                cornerState.Mz = cornerState.Mz * limitScale;
                usage = 1;
            end
            cornerState.frictionLimit = directionalLimit;
            cornerState.frictionUsage = usage;
        end

        %% ---- TireModel interface methods ----

        function Fy = computeLateralForce(obj, normalLoad, slipAngle, mu)
            % COMPUTELATERALFORCE Lateral force [N] for a single evaluation
            %   Fy = computeLateralForce(obj, normalLoad, slipAngle, mu)
            %
            %   This is the TireModel interface method for standalone queries.
            %   For per-corner state tracking, use updateCorner() instead.

	            normalLoad = utils.nonnegativeScalarOrDefault(normalLoad, 0);
	            slipAngle = utils.scalarOrDefault(slipAngle, 0);
	            mu = utils.nonnegativeScalarOrDefault(mu, 0);
	            minEvaluationLoad = utils.positiveScalarOrDefault( ...
	                obj.minEvaluationLoad, 100);
	            if normalLoad <= 0
	                Fy = 0;
	                return;
	            end

	            evalLoad = max(normalLoad, minEvaluationLoad);
            loadScale = normalLoad / evalLoad;
            inputsMF = [
                evalLoad, 0, slipAngle, 0, 0, ...
                    obj.tireConstants.refVelocity, obj.tireConstants.nomPressure
                evalLoad, 0, 0,         0, 0, ...
                    obj.tireConstants.refVelocity, obj.tireConstants.nomPressure
            ];
            outputs = mfeval(obj.tireConstants.params, inputsMF, 111);

            rawPeakMu = obj.computePeakMuInternal(evalLoad, 0, ...
                obj.tireConstants.nomPressure, obj.tireConstants.params);
            surfaceScale = obj.computeSurfaceScale(rawPeakMu, mu);
            Fy = -(outputs(1, 2) - outputs(2, 2)) * surfaceScale * loadScale;
        end

        function Fx = computeLongitudinalForce(obj, normalLoad, slipRatio, mu)
            % COMPUTELONGITUDINALFORCE Longitudinal force [N] for a single evaluation
            %   Fx = computeLongitudinalForce(obj, normalLoad, slipRatio, mu)
            %
            %   This is the TireModel interface method for standalone queries.
            %   For per-corner state tracking, use updateCorner() instead.

	            normalLoad = utils.nonnegativeScalarOrDefault(normalLoad, 0);
	            slipRatio = utils.unitSignedScalarOrDefault(slipRatio, 0);
	            mu = utils.nonnegativeScalarOrDefault(mu, 0);
	            minEvaluationLoad = utils.positiveScalarOrDefault( ...
	                obj.minEvaluationLoad, 100);
	            if normalLoad <= 0
	                Fx = 0;
	                return;
	            end

	            evalLoad = max(normalLoad, minEvaluationLoad);
            loadScale = normalLoad / evalLoad;
            inputsMF = [
                evalLoad, slipRatio, 0, 0, 0, ...
                    obj.tireConstants.refVelocity, obj.tireConstants.nomPressure
                evalLoad, 0,         0, 0, 0, ...
                    obj.tireConstants.refVelocity, obj.tireConstants.nomPressure
            ];
            outputs = mfeval(obj.tireConstants.params, inputsMF, 111);

            rawPeakMu = obj.computePeakLongitudinalMuInternal(evalLoad, 0, ...
                obj.tireConstants.nomPressure, obj.tireConstants.params);
            surfaceScale = obj.computeSurfaceScale(rawPeakMu, mu);
            Fx = (outputs(1, 1) - outputs(2, 1)) * surfaceScale * loadScale;
        end

        function peakMu = getPeakFriction(obj, normalLoad)
            % GETPEAKFRICTION Peak friction coefficient at given load
            %   peakMu = getPeakFriction(obj, normalLoad)
            %
            %   Scans the lateral force curve to find max |Fy|/Fz.
            %   Accounts for load sensitivity inherent in the Magic Formula.

	            normalLoad = utils.nonnegativeScalarOrDefault(normalLoad, 0);
	            if normalLoad <= 0
	                peakMu = 0;
	                return;
            end

            evalLoad = obj.quantizePeakMuLoad(normalLoad);
            [peakMu, hit] = obj.lookupPeakMuCache( ...
                obj.peakMuLatCacheLoads, obj.peakMuLatCacheValues, evalLoad);
            if hit
                return;
            end

            peakMu = obj.computePeakMuInternal(evalLoad, 0, ...
                obj.tireConstants.nomPressure, obj.tireConstants.params);
            obj.peakMuLatCacheLoads(end + 1, 1) = evalLoad;
            obj.peakMuLatCacheValues(end + 1, 1) = peakMu;
        end

        function peakMu = getPeakLongitudinalFriction(obj, normalLoad)
            % GETPEAKLONGITUDINALFRICTION Peak longitudinal mu at given load.
            %
            % Brake and drive limits consume longitudinal grip. Reading this
            % from the .tir longitudinal curve avoids assuming the lateral peak
            % is also the braking/traction peak.
	            normalLoad = utils.nonnegativeScalarOrDefault(normalLoad, 0);
	            if normalLoad <= 0
	                peakMu = 0;
	                return;
            end

            evalLoad = obj.quantizePeakMuLoad(normalLoad);
            [peakMu, hit] = obj.lookupPeakMuCache( ...
                obj.peakMuLongCacheLoads, obj.peakMuLongCacheValues, evalLoad);
            if hit
                return;
            end

            peakMu = obj.computePeakLongitudinalMuInternal(evalLoad, 0, ...
                obj.tireConstants.nomPressure, obj.tireConstants.params);
            obj.peakMuLongCacheLoads(end + 1, 1) = evalLoad;
            obj.peakMuLongCacheValues(end + 1, 1) = peakMu;
        end

        %% ---- Slip angle computation ----

        function slipAngles = computeSlipAngles(obj, vx, vy, yawRate, steerInput, wheelbase, frontWeightFrac)
            % COMPUTESLIPANGLES Compute per-corner tire slip angles [rad]
            %   slipAngles = computeSlipAngles(vx, vy, yawRate, steerInput, wheelbase, frontWeightFrac)
            %
            %   Uses per-corner wheel-plane kinematics. When track width is
            %   supplied through updateAllFromState, the front wheels use
            %   simple Ackermann steering angles.
            %
            %   Inputs:
            %     vx              - forward velocity [m/s]
            %     vy              - lateral velocity at CG [m/s]
            %     yawRate         - yaw rate [rad/s]
            %     steerInput      - driver steering input [rad]
            %     wheelbase       - vehicle wheelbase [m]
            %     frontWeightFrac - static front weight distribution [0-1]
            %
            %   Returns struct with:
            %     slipAngles.FL, .FR, .RL, .RR  [rad]

            wheelKinematics = obj.computeWheelKinematics( ...
                vx, vy, yawRate, steerInput, wheelbase, frontWeightFrac, 0);
            slipAngles = struct('FL', wheelKinematics.FL.slipAngle, ...
                'FR', wheelKinematics.FR.slipAngle, ...
                'RL', wheelKinematics.RL.slipAngle, ...
                'RR', wheelKinematics.RR.slipAngle);

        end

        function wheelKinematics = computeWheelKinematics(obj, vx, vy, yawRate, steerInput, wheelbase, frontWeightFrac, trackWidth)
            % COMPUTEWHEELKINEMATICS Per-corner slip angle and wheel-plane speed
            vx = utils.scalarOrDefault(vx, 0);
            vy = utils.scalarOrDefault(vy, 0);
            yawRate = utils.scalarOrDefault(yawRate, 0);
            steerInput = utils.scalarOrDefault(steerInput, 0);
            wheelbase = utils.positiveScalarOrDefault(wheelbase, 1.55);
            frontWeightFrac = utils.unitScalarOrDefault(frontWeightFrac, 0.45);
            wheelKinematics = obj.emptyWheelKinematics(hypot(vx, vy));
            if nargin < 8 || isempty(trackWidth)
                trackWidth = 0;
            end
            trackWidth = utils.nonnegativeScalarOrDefault(trackWidth, 0);

            lf = wheelbase * (1 - frontWeightFrac);
            lr = wheelbase * frontWeightFrac;
            halfTrack = max(trackWidth, 0) / 2;
            [deltaFL, deltaFR] = obj.computeAckermannSteer(steerInput, wheelbase, trackWidth);

            wheelKinematics.FL = obj.computeCornerKinematics(vx, vy, yawRate, lf, halfTrack, deltaFL);
            wheelKinematics.FR = obj.computeCornerKinematics(vx, vy, yawRate, lf, -halfTrack, deltaFR);
            wheelKinematics.RL = obj.computeCornerKinematics(vx, vy, yawRate, -lr, halfTrack, 0);
            wheelKinematics.RR = obj.computeCornerKinematics(vx, vy, yawRate, -lr, -halfTrack, 0);
        end

        function wheel = computeCornerKinematics(obj, vx, vy, yawRate, xOffset, yOffset, steerAngle)
            vx = utils.scalarOrDefault(vx, 0);
            vy = utils.scalarOrDefault(vy, 0);
            yawRate = utils.scalarOrDefault(yawRate, 0);
            xOffset = utils.scalarOrDefault(xOffset, 0);
            yOffset = utils.scalarOrDefault(yOffset, 0);
            steerAngle = utils.scalarOrDefault(steerAngle, 0);
            wheelVx = vx - yawRate * yOffset;
            wheelVy = vy + yawRate * xOffset;

            % Resolve velocity in tire-local coordinates. A small CG forward
            % speed does not imply zero tire slip: yaw and lateral velocity can
            % move a contact patch quickly even while the car is nearly stopped
            % along the track.
            localLongitudinalSpeed = wheelVx * cos(steerAngle) + wheelVy * sin(steerAngle);
            localLateralSpeed = -wheelVx * sin(steerAngle) + wheelVy * cos(steerAngle);
            patchSpeed = hypot(localLongitudinalSpeed, localLateralSpeed);
            slipAngle = 0;
            if patchSpeed >= 0.1
                % Match the SimpleTire sign convention: positive slip angle
                % produces positive body-left tire force.
                slipAngle = -atan2(localLateralSpeed, max(localLongitudinalSpeed, eps));
            end

            wheel = struct( ...
                'slipAngle', slipAngle, ...
                'longitudinalSpeed', max(localLongitudinalSpeed, 0));
        end

        function wheelKinematics = emptyWheelKinematics(~, vehicleSpeed)
            vehicleSpeed = utils.nonnegativeScalarOrDefault( ...
                vehicleSpeed, 0);
            zeroWheel = struct('slipAngle', 0, 'longitudinalSpeed', max(vehicleSpeed, 0));
            wheelKinematics = struct('FL', zeroWheel, 'FR', zeroWheel, ...
                'RL', zeroWheel, 'RR', zeroWheel);
        end

        %% ---- Slip ratio computation ----

        function kappa = computeSlipRatio(obj, cornerState, vehicleSpeed)
            % COMPUTESLIPRATIO Compute longitudinal slip ratio for one corner
            %   kappa = computeSlipRatio(cornerState, vehicleSpeed)
            %
            %   Slip ratio definition:
            %     kappa = (omega * R - V) / max(|omega * R|, |V|, epsilon)
            %
            %   kappa > 0 → driving (wheel faster than vehicle)
            %   kappa < 0 → braking (wheel slower than vehicle)
            %
            %   Inputs:
            %     cornerState  - TireState with angularVelocity and wheelRadius
            %     vehicleSpeed - Vehicle forward speed [m/s]
            %
            %   Returns:
            %     kappa - Slip ratio [-1, 1]

	            omega = utils.nonnegativeScalarOrDefault( ...
	                cornerState.angularVelocity, 0);
	            R     = utils.positiveScalarOrDefault(cornerState.wheelRadius, 0.241935);
	            V     = utils.nonnegativeScalarOrDefault(vehicleSpeed, 0);   % no reverse

            wheelSpeed = omega * R;
            denom = max(abs(wheelSpeed), abs(V));

            if denom < 0.1
                % At very low speed, slip ratio is ill-defined
                kappa = 0;
            else
                kappa = (wheelSpeed - V) / denom;
            end

            % Clamp to [-1, 1]
            kappa = max(-1, min(1, kappa));
        end

        function omegaTelemetry = updateWheelDynamics(obj, cornerState, driveTorque, brakeTorque, dt)
            % UPDATEWHEELDYNAMICS Integrate wheel angular velocity forward
            %   updateWheelDynamics(cornerState, driveTorque, brakeTorque, dt)
            %
            %   Rotational equation of motion:
            %     I * d(omega)/dt = T_drive - T_brake - Fx * R
            %
            %   where:
            %     T_drive = applied drive torque at this wheel [Nm]
            %     T_brake = applied brake torque at this wheel [Nm] (positive value)
            %     Fx      = longitudinal tire force from previous evaluation [N]
            %     R       = effective wheel radius [m]
            %     I       = wheel rotational inertia [kg·m^2]
            %
            %   Uses explicit Euler integration.
            %
            %   Inputs:
            %     cornerState - TireState handle (angularVelocity is mutated)
            %     driveTorque - Net drive torque at this wheel [Nm]
            %     brakeTorque - Brake torque at this wheel [Nm] (positive magnitude)
            %     dt          - Timestep [s]

	            omega = utils.nonnegativeScalarOrDefault( ...
	                cornerState.angularVelocity, 0);
	            R     = utils.positiveScalarOrDefault(cornerState.wheelRadius, 0.241935);
	            I     = utils.positiveScalarOrDefault(obj.wheelInertia, 0.5);
	            Fx    = utils.scalarOrDefault(cornerState.Fx, 0);  % from previous tire evaluation
	            driveTorque = utils.scalarOrDefault(driveTorque, 0);
	            brakeTorque = utils.nonnegativeScalarOrDefault(brakeTorque, 0);
	            dt = utils.nonnegativeScalarOrDefault(dt, 0);

            % Net torque: drive accelerates, brake and tire Fx decelerate.
            % Fx > 0 means driving force, so the road reaction opposes wheel
            % spin. If omega is exactly zero under braking, keep brake torque
            % opposing forward rolling rather than letting a locked wheel
            % instantly spin up from tire reaction torque.
            brakeDirection = sign(omega);
            if brakeDirection == 0 && brakeTorque > 0
                brakeDirection = 1;
            end
            netTorque = driveTorque - brakeDirection * brakeTorque - Fx * R;

            % Angular acceleration
            alpha = netTorque / I;

            % Euler integration
            omega_new_unclamped = omega + alpha * dt;

            % Prevent wheel from spinning backwards (one-direction clutch)
            omega_new = max(0, omega_new_unclamped);

            cornerState.angularVelocity = omega_new;
            omegaTelemetry.omegaBefore = max(omega, 0);
            omegaTelemetry.omegaUnclamped = omega_new_unclamped;
            omegaTelemetry.omegaAfter = omega_new;
            omegaTelemetry.omegaMean = obj.computeMeanPositiveAngularVelocity( ...
                omega, omega_new_unclamped);
        end

        %% ---- All-corners batch update ----

        function updateAllCorners(obj, Fz_FL, Fz_FR, Fz_RL, Fz_RR, ...
                slipAngle_FL, slipAngle_FR, slipAngle_RL, slipAngle_RR, ...
                kappa_FL, kappa_FR, kappa_RL, kappa_RR, mu)
            % UPDATEALLCORNERS Evaluate all four corners at once
            %   updateAllCorners(Fz_FL, Fz_FR, Fz_RL, Fz_RR, ...
            %       slipAngle_FL, slipAngle_FR, slipAngle_RL, slipAngle_RR, ...
            %       kappa_FL, kappa_FR, kappa_RL, kappa_RR, mu)
            %
            %   Updates all four corner states with per-corner slip ratios.
            %   Camber defaults to 0 for all corners.

            obj.updateCorner(obj.FL, Fz_FL, slipAngle_FL, kappa_FL, 0, mu);
            obj.updateCorner(obj.FR, Fz_FR, slipAngle_FR, kappa_FR, 0, mu);
            obj.updateCorner(obj.RL, Fz_RL, slipAngle_RL, kappa_RL, 0, mu);
            obj.updateCorner(obj.RR, Fz_RR, slipAngle_RR, kappa_RR, 0, mu);
        end

        function updateAllFromState(obj, state, vehicleManager, cornerLoads, mu, dt)
            % UPDATEALLFROMSTATE Compute slip angles/ratios and update all corners
            %   updateAllFromState(state, vehicleManager, cornerLoads, mu, dt)
            %
            %   Computes per-corner slip angles from vehicle kinematics and
            %   per-corner slip ratios from wheel rotational state, then
            %   delegates to updateAllCorners().
            %
            %   Inputs:
            %     state          - VehicleState with speed, vy, yawRate, steer
            %     vehicleManager - VehicleManager for geometry (wheelbase, weight dist)
            %     cornerLoads    - struct with .FL, .FR, .RL, .RR normal forces [N]
            %     mu             - Surface friction multiplier
            %                      Treated as an absolute surface grip cap here.
            %     dt             - Timestep [s] for tire relaxation
            if nargin < 6
                dt = 0;
            end

            % Compute per-corner slip angles and local wheel-plane speeds
            wheelKinematics = obj.computeWheelKinematics( ...
                state.speed, state.vy, state.yawRate, state.steer, ...
                vehicleManager.wheelbase, vehicleManager.staticFrontWeight, ...
                vehicleManager.trackWidth);

            % Compute per-corner slip ratios from wheel rotational state
            kappa_FL = obj.computeSlipRatio(obj.FL, wheelKinematics.FL.longitudinalSpeed);
            kappa_FR = obj.computeSlipRatio(obj.FR, wheelKinematics.FR.longitudinalSpeed);
            kappa_RL = obj.computeSlipRatio(obj.RL, wheelKinematics.RL.longitudinalSpeed);
            kappa_RR = obj.computeSlipRatio(obj.RR, wheelKinematics.RR.longitudinalSpeed);

            % Relaxation length is distance travelled by each contact patch.
            % During yaw or Ackermann steering, local wheel-plane speeds differ
            % from CG speed, so each tire advances its own relaxation state.
            [alpha_FL, kappa_FL] = obj.relaxSlipInputs( ...
                obj.FL, wheelKinematics.FL.slipAngle, kappa_FL, ...
                wheelKinematics.FL.longitudinalSpeed, dt);
            [alpha_FR, kappa_FR] = obj.relaxSlipInputs( ...
                obj.FR, wheelKinematics.FR.slipAngle, kappa_FR, ...
                wheelKinematics.FR.longitudinalSpeed, dt);
            [alpha_RL, kappa_RL] = obj.relaxSlipInputs( ...
                obj.RL, wheelKinematics.RL.slipAngle, kappa_RL, ...
                wheelKinematics.RL.longitudinalSpeed, dt);
            [alpha_RR, kappa_RR] = obj.relaxSlipInputs( ...
                obj.RR, wheelKinematics.RR.slipAngle, kappa_RR, ...
                wheelKinematics.RR.longitudinalSpeed, dt);

	            obj.updateAllCorners( ...
	                utils.cornerLoadOrDefault(cornerLoads, 'FL'), ...
	                utils.cornerLoadOrDefault(cornerLoads, 'FR'), ...
	                utils.cornerLoadOrDefault(cornerLoads, 'RL'), ...
	                utils.cornerLoadOrDefault(cornerLoads, 'RR'), ...
	                alpha_FL, alpha_FR, alpha_RL, alpha_RR, ...
	                kappa_FL, kappa_FR, kappa_RL, kappa_RR, mu);
	        end
    end

    methods (Access = private)

	        function evalLoad = quantizePeakMuLoad(obj, normalLoad)
	            normalLoad = utils.nonnegativeScalarOrDefault(normalLoad, 0);
	            minEvaluationLoad = utils.positiveScalarOrDefault( ...
	                obj.minEvaluationLoad, 100);
	            evalLoad = max(normalLoad, minEvaluationLoad);
	            resolution = utils.positiveScalarOrDefault( ...
	                obj.peakMuCacheLoadResolution, 25);
	            evalLoad = max(minEvaluationLoad, ...
	                round(evalLoad / resolution) * resolution);
	        end

        function [peakMu, hit] = lookupPeakMuCache(~, loads, values, evalLoad)
            hit = false;
            peakMu = 0;
            if isempty(loads)
                return;
            end

            idx = find(loads == evalLoad, 1, 'first');
            if isempty(idx)
                return;
            end

            peakMu = values(idx);
            hit = true;
        end

        function zeroOutput = getZeroSlipOutput(obj, Fz, gamma, P, params)
            evalLoad = obj.quantizePeakMuLoad(Fz);
            gammaKey = round(gamma / 1e-4) * 1e-4;

            if ~isempty(obj.zeroSlipOffsetLoads)
                idx = find(obj.zeroSlipOffsetLoads == evalLoad ...
                    & obj.zeroSlipOffsetCambers == gammaKey, 1, 'first');
                if ~isempty(idx)
                    zeroOutput = obj.zeroSlipOffsetOutputs(idx, :);
                    return;
                end
            end

            inputsMF = [evalLoad, 0, 0, gammaKey, 0, ...
                obj.tireConstants.refVelocity, P];
            zeroOutput = mfeval(params, inputsMF, 111);

            obj.zeroSlipOffsetLoads(end + 1, 1) = evalLoad;
            obj.zeroSlipOffsetCambers(end + 1, 1) = gammaKey;
            obj.zeroSlipOffsetOutputs(end + 1, :) = zeroOutput;
        end

	        function [deltaFL, deltaFR] = computeAckermannSteer(obj, steerInput, wheelbase, trackWidth)
	            % COMPUTEACKERMANNSTEER Convert bicycle steer to front wheel angles.
	            steerInput = utils.scalarOrDefault(steerInput, 0);
	            wheelbase = utils.positiveScalarOrDefault(wheelbase, 1.55);
	            trackWidth = utils.nonnegativeScalarOrDefault(trackWidth, 0);
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

	        function surfaceScale = computeSurfaceScale(obj, rawPeakMu, surfaceMu)
	            % COMPUTESURFACESCALE Scale tire forces so surface mu is an absolute cap.
	            rawPeakMu = utils.nonnegativeScalarOrDefault(rawPeakMu, 0);
	            surfaceMu = utils.nonnegativeScalarOrDefault(surfaceMu, 0);
	            if rawPeakMu <= 0
                surfaceScale = 0;
            else
                surfaceScale = min(1, surfaceMu / rawPeakMu);
            end
        end

	        function [alpha, kappa] = relaxSlipInputs(obj, cornerState, targetAlpha, targetKappa, vehicleSpeed, dt)
	            targetAlpha = utils.scalarOrDefault(targetAlpha, 0);
	            targetKappa = utils.unitSignedScalarOrDefault(targetKappa, 0);
	            vehicleSpeed = utils.nonnegativeScalarOrDefault(vehicleSpeed, 0);
	            dt = utils.nonnegativeScalarOrDefault(dt, 0);
	            cornerState.targetSlipAngle = targetAlpha;
	            cornerState.targetSlipRatio = targetKappa;

	            enableRelaxation = utils.logicalScalarOrDefault( ...
	                obj.enableRelaxation, true);
	            if ~enableRelaxation || dt <= 0 || vehicleSpeed < 0.5
                cornerState.relaxedSlipAngle = targetAlpha;
                cornerState.relaxedSlipRatio = targetKappa;
                cornerState.slipStateInitialized = true;
                alpha = targetAlpha;
                kappa = targetKappa;
                return;
            end

            if ~cornerState.slipStateInitialized
                cornerState.relaxedSlipAngle = targetAlpha;
                cornerState.relaxedSlipRatio = targetKappa;
                cornerState.slipStateInitialized = true;
            end

	            lateralRelaxationLength = utils.positiveScalarOrDefault( ...
	                obj.lateralRelaxationLength, 2.5);
	            longitudinalRelaxationLength = utils.positiveScalarOrDefault( ...
	                obj.longitudinalRelaxationLength, 0.8);
	            alphaGain = min(1, ...
	                vehicleSpeed * dt / max(lateralRelaxationLength, eps));
	            kappaGain = min(1, ...
	                vehicleSpeed * dt / max(longitudinalRelaxationLength, eps));

            cornerState.relaxedSlipAngle = cornerState.relaxedSlipAngle + ...
                alphaGain * (targetAlpha - cornerState.relaxedSlipAngle);
            cornerState.relaxedSlipRatio = cornerState.relaxedSlipRatio + ...
                kappaGain * (targetKappa - cornerState.relaxedSlipRatio);

            alpha = cornerState.relaxedSlipAngle;
            kappa = cornerState.relaxedSlipRatio;
        end

	        function peakMu = computePeakMuInternal(obj, Fz, gamma, P, params)
	            % COMPUTEPEAKMUINTERNAL Scan lateral curve to find peak mu
	            %   Vectorized: builds a matrix of 50 input rows, single mfeval call
	            Fz = utils.positiveScalarOrDefault(Fz, 100);
	            gamma = utils.scalarOrDefault(gamma, 0);
	            P = utils.positiveScalarOrDefault(P, obj.tireConstants.nomPressure);

	            alphaScan = linspace(-0.21, 0.21, 50);  % ±12 deg in rad
            V = obj.tireConstants.refVelocity;
            nScan = numel(alphaScan);

            % Build inputs matrix: each row = [Fz, kappa, alpha, gamma, phit, Vx, P]
            inputsMF = [repmat(Fz, nScan, 1), ...    % Fz
                        zeros(nScan, 1), ...          % kappa = 0 (pure lateral)
                        alphaScan(:), ...             % alpha scan
                        repmat(gamma, nScan, 1), ...  % gamma
                        zeros(nScan, 1), ...          % phit = 0
                        repmat(V, nScan, 1), ...      % Vx
                        repmat(P, nScan, 1)];         % P

            outputs = mfeval(params, inputsMF, 111);
            zeroOutput = obj.getZeroSlipOutput(Fz, gamma, P, params);
            peakMu = max(abs(outputs(:,2) - zeroOutput(:,2))) / Fz;
            % Validate and clamp result to physically reasonable range
            if ~isfinite(peakMu) || peakMu < 0
                peakMu = 0;
            end
            peakMu = min(peakMu, 2.5);  % Maximum realistic peak friction
        end

	        function peakMu = computePeakLongitudinalMuInternal(obj, Fz, gamma, P, params)
	            % COMPUTEPEAKLONGITUDINALMUINTERNAL Scan pure longitudinal curve.
	            %
	            % Pacejka longitudinal and lateral peak coefficients often differ.
	            % This scan is used for braking/drive limits and for the combined
	            % friction ellipse telemetry.
	            Fz = utils.positiveScalarOrDefault(Fz, 100);
	            gamma = utils.scalarOrDefault(gamma, 0);
	            P = utils.positiveScalarOrDefault(P, obj.tireConstants.nomPressure);

	            kappaScan = linspace(-0.25, 0.25, 50);
            V = obj.tireConstants.refVelocity;
            nScan = numel(kappaScan);

            inputsMF = [repmat(Fz, nScan, 1), ...    % Fz
                        kappaScan(:), ...            % kappa scan
                        zeros(nScan, 1), ...          % alpha = 0
                        repmat(gamma, nScan, 1), ...  % gamma
                        zeros(nScan, 1), ...          % phit = 0
                        repmat(V, nScan, 1), ...      % Vx
                        repmat(P, nScan, 1)];         % P

            outputs = mfeval(params, inputsMF, 111);
            zeroOutput = obj.getZeroSlipOutput(Fz, gamma, P, params);
            peakMu = max(abs(outputs(:,1) - zeroOutput(:,1))) / Fz;
            % Validate and clamp result to physically reasonable range
            if ~isfinite(peakMu) || peakMu < 0
                peakMu = 0;
            end
            peakMu = min(peakMu, 2.5);  % Maximum realistic peak friction
        end

	        function [usage, directionalLimit] = computeFrictionEllipseUsage( ...
	                obj, Fx, Fy, normalLoad, peakMuLong, peakMuLat)
	            % COMPUTEFRICTIONELLIPSEUSAGE Directional combined-slip usage.
	            Fx = utils.scalarOrDefault(Fx, 0);
	            Fy = utils.scalarOrDefault(Fy, 0);
	            normalLoad = utils.nonnegativeScalarOrDefault(normalLoad, 0);
	            peakMuLong = utils.nonnegativeScalarOrDefault(peakMuLong, 0);
	            peakMuLat = utils.nonnegativeScalarOrDefault(peakMuLat, 0);
	            FxLimit = max(peakMuLong, 0) * max(normalLoad, 0);
            FyLimit = max(peakMuLat, 0) * max(normalLoad, 0);

            if FxLimit <= 0 || FyLimit <= 0
                usage = 0;
                directionalLimit = 0;
                return;
            end

            usage = hypot(Fx / FxLimit, Fy / FyLimit);
            forceMagnitude = hypot(Fx, Fy);
            if forceMagnitude > 0
                cosForce = Fx / forceMagnitude;
                sinForce = Fy / forceMagnitude;
                directionalLimit = 1 / sqrt((cosForce / FxLimit)^2 ...
                    + (sinForce / FyLimit)^2);
            else
                directionalLimit = min(FxLimit, FyLimit);
	            end
	        end
	    end
	end
