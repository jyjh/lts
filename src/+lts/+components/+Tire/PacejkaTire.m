classdef PacejkaTire < lts.components.Tire.TireModel
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
    %   tire = lts.components.Tire.PacejkaTire('43105_18x7.5_10_R25B_7.tir')
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

        % Lateral tire relaxation length [m]. Models the first-order
        % contact-patch lag between kinematic slip angle and force-producing
        % slip angle:
        %   sigma * d(alpha)/dt + V * alpha = V * alpha_ss
        % Solved in the exact, unconditionally-stable exponential form.
        % 0 disables the transient layer (pure steady-state Magic Formula).
        relaxationLength = 0.30

        % Longitudinal relaxation length [m] for slip ratio. NaN preserves
        % backward compatibility by using relaxationLength for both states.
        % A separate value avoids imposing the substantially longer lateral
        % carcass/contact-patch response on the driven-wheel torque loop.
        longitudinalRelaxationLength = NaN

        % Contact-patch load-response length [m]. Tire forces do not follow
        % a normal-load step instantaneously: the patch pressure profile
        % rebuilds as the contact geometry rolls onto the new load, so the
        % force response to Fz changes lags on a contact-patch transit time
        % scale (sigma_Fz / V). Filtering the load the Magic Formula sees
        % with the same exact exponential lag used for slip gives:
        %   - identical steady-state forces (no static distortion), and
        %   - a finite high-frequency Fz->Fx/Fy gain, which breaks the
        %     algebraic positive-feedback loop
        %     Fx -> ax -> chassis attitude -> Fz -> Cx/mu*Fz -> Fx
        %     that otherwise sustains a nonphysical ~10-15 Hz pitch/load
        %     oscillation under heavy longitudinal loading.
        % 0 (the default) disables the filter and evaluates at the
        % instantaneous load (legacy behavior).
        normalLoadRelaxationLength = 0

        % Multiplier on force-evaluation slip angle. This is a correlation
        % knob for cornering-stiffness sensitivity; stored kinematic and
        % relaxed slip states remain in physical radians.
        lateralStiffnessScale = 1.0

        % Optional [FL FR RL RR] multipliers applied after the global scale.
        % This changes axle cornering stiffness without changing peak grip or
        % the physical kinematic/relaxed slip states.
        lateralStiffnessScaleByCorner = [1 1 1 1]

        % Rolling-resistance coefficient [-]. Acts as a wheel-resistance
        % torque T_rr = Crr * Fz * R opposing rotation, so a free-rolling
        % wheel coast-down is driven by the contact patch (not just a body
        % force). Typical racing slick Crr ~ 0.010-0.015. Set to 0.015 to
        % match the legacy body-force model; the body force is removed to
        % avoid double-counting.
        rollingResistanceCoeff = 0.015

        % Bearing drag coefficient [N·m·s/rad]. Viscous wheel-hub drag
        % T_bearing = C_bearing * omega opposing spin. A small value lets a
        % freely spinning wheel decay to the road speed instead of remaining
        % locked to it indefinitely. 0 disables it (default); enable for
        % extra coast-down drag.
        bearingDragCoeff = 0

        % Deprecated compatibility flag retained for configs that set it.
        % Wheel angular velocity is no longer direction-clamped.
        allowReverseRotation = false

        % Cache peak-mu scans by rounded load/camber/speed. The public
        % string-keyed cache is kept readable for tests/debugging; the
        % numeric cache is the hot-loop lookup path.
        peakMuCache
        peakMuNumericCache

        % Deprecated compatibility property. Surface-dependent friction
        % scaling has been removed; the raw tire file always defines grip.
        % Legacy callers may still read or assign this value, but it has no
        % effect on forces and is fixed conceptually at unity.
        surfaceMuReference = 1.0

        % Resolved MFeval low-speed floor [m/s]. The .tir params struct is
        % immutable, so whether it carries VXLOW (and its value) is a run
        % invariant; cached to avoid an isfield check per corner per step.
        cachedMFevalLowSpeed = NaN
    end
    
    methods
        function obj = PacejkaTire(tirFilePath, varargin)
            % PACEJKATIRE Construct from a .tir file, creating 4 corner states
            %   PacejkaTire(tirFilePath)
            %   PacejkaTire(tirFilePath, 'Verbose', true)
            %
            %   tirFilePath — path to the .tir file. If relative, resolved
            %                 relative to the +Tire/ folder.
            %   'Verbose'   — forwarded to TireConstants and gates the
            %                 corner-state summary print (default false).

            % Load shared tire constants
            obj.tireConstants = lts.components.Tire.TireConstants( ...
                tirFilePath, varargin{:});
            obj.cachedMFevalLowSpeed = obj.resolveMFevalLowSpeed();

            % Create per-corner state objects
            obj.FL = lts.components.Tire.TireState();
            obj.FR = lts.components.Tire.TireState();
            obj.RL = lts.components.Tire.TireState();
            obj.RR = lts.components.Tire.TireState();
            obj.peakMuCache = containers.Map('KeyType', 'char', 'ValueType', 'double');
            obj.peakMuNumericCache = containers.Map('KeyType', 'double', 'ValueType', 'double');

            verboseParser = inputParser;
            verboseParser.addParameter('Verbose', false, ...
                @(x) islogical(x) || (isnumeric(x) && isscalar(x)));
            verboseParser.parse(varargin{:});
            if logical(verboseParser.Results.Verbose)
                fprintf('  PacejkaTire: 4 corner states created (FL, FR, RL, RR)\n');
            end
        end
        
        %% ---- Per-corner evaluation ----
        
        function updateCorner(obj, cornerState, normalLoad, slipAngle, slipRatio, camberAngle, varargin)
            % UPDATECORNER Evaluate MFeval for one corner and update its state
            %   updateCorner(cornerState, normalLoad, slipAngle, slipRatio, camberAngle)
            %   updateCorner(cornerState, normalLoad, slipAngle, slipRatio, camberAngle, dt, longSpeed)
            %
            %   cornerState  — TireState handle for this corner
            %   normalLoad   — Tire normal force Fz [N]
            %   slipAngle    — Steady-state (kinematic) slip angle alpha [rad]
            %   slipRatio    — Steady-state (kinematic) slip ratio kappa [-1 to 1]
            %   camberAngle  — Inclination angle gamma [rad]
            %   dt           — Timestep [s] (optional; enables relaxation lag)
            %   longSpeed    — Contact-patch longitudinal speed [m/s] (optional)
            %
            %   Legacy surface-Mu arguments are accepted for API compatibility
            %   but ignored. MFeval forces are used without surface scaling.
            %   Mutates cornerState in-place with computed forces and moments.

            [~, dt, longSpeed, computePeakMu, relaxationMode] = ...
                obj.parseCornerOptionalArgs(varargin{:});
            relaxationMode = obj.resolveRelaxationMode(dt, relaxationMode);
            
            % Store inputs
            cornerState.normalForce = normalLoad;
            ssAlpha = lts.util.clamp(slipAngle, -0.3, 0.3);   % steady-state (kinematic) target
            ssKappa = obj.clampSlipRatio(slipRatio);

            % Apply first-order contact-patch relaxation to obtain the
            % transient (force-producing) slip. With relaxationLength = 0
            % the transient slip equals the steady-state slip (baseline).
            [alpha, kappa] = obj.applyRelaxation( ...
                cornerState, ssAlpha, ssKappa, longSpeed, dt, relaxationMode);
            FzEval = obj.applyLoadRelaxation({cornerState}, normalLoad, ...
                longSpeed, dt, relaxationMode, ...
                strcmp(relaxationMode, 'hold'));

            cornerState.ssSlipAngle = ssAlpha;
            cornerState.ssSlipRatio = ssKappa;
            % Commit the advanced lagged state only when dt > 0; during
            % intermediate wheel-solve iterations (dt = 0) preserve the
            % previous lagged slip for the next physics step.
            if strcmp(relaxationMode, 'advance') && dt > 0
                cornerState.slipAngle = alpha;
                cornerState.slipRatio = kappa;
            end
            cornerState.camberAngle = camberAngle;

            if FzEval <= 0
                cornerState.Fy = 0;
                cornerState.Fx = 0;
                cornerState.Mx = 0;
                cornerState.My = 0;
                cornerState.Mz = 0;
                cornerState.peakMu = 0;
                return;
            end

            % Unpack for MFeval call. Evaluate at the contact-patch
            % longitudinal speed (speed-sensitive Pacejka) so load/speed
            % dependence is captured; this matches updateAllCorners.
            Fz    = FzEval;
            gamma = camberAngle;
            V     = obj.computeMFevalSpeed(longSpeed);
            P     = obj.tireConstants.nomPressure;
            params = obj.tireConstants.params;

            isRight = obj.isRightCorner(cornerState);
            alphaEval = obj.evaluationSlipAngle(alpha);
            if isRight
                % The bundled TIR data is for a left-side tire. Mirror the
                % slip input when the same tire is mounted on the right.
                alphaEval = -alphaEval;
            end

            % Build MFeval inputs row: [Fz, kappa, alpha, gamma, phit, Vx, P]
            inputsMF = [Fz, kappa, alphaEval, gamma, 0, V, P];

            % Evaluate Pacejka Magic Formula via MFeval (useMode=111: combined)
            outputs = mfeval(params, inputsMF, 111);

            if computePeakMu
                rawPeakMu = obj.getCachedPeakMu(Fz, gamma, P, params, longSpeed);
                cornerState.peakMu = rawPeakMu;
            end

            % Store the raw Magic-Formula forces.
            cornerState.Fx = outputs(:,1);
            cornerState.My = outputs(:,5);
            if isRight
                cornerState.Fy = outputs(:,2);
                cornerState.Mx = -outputs(:,4);
                cornerState.Mz = outputs(:,6);
            else
                cornerState.Fy = -outputs(:,2);
                cornerState.Mx = outputs(:,4);
                % MFeval's lateral convention is opposite the simulator's
                % (Fy is negated above), so its aligning moment changes sign too.
                cornerState.Mz = -outputs(:,6);
            end
        end
        
        %% ---- TireModel interface methods ----
        
        function Fy = computeLateralForce(obj, normalLoad, slipAngle, mu)
            % COMPUTELATERALFORCE Lateral force [N] for a single evaluation
            %   Fy = computeLateralForce(obj, normalLoad, slipAngle, mu)
            %
            %   This is the TireModel interface method for standalone queries.
            %   For per-corner state tracking, use updateCorner() instead.
            
            if normalLoad <= 0
                Fy = 0;
                return;
            end
            inputsMF = [normalLoad, 0, obj.evaluationSlipAngle(slipAngle), 0, 0, ...
                obj.tireConstants.refVelocity, obj.tireConstants.nomPressure];
            outputs = mfeval(obj.tireConstants.params, inputsMF, 111);
            
            Fy = -outputs(:,2);
        end

        function Fx = computeLongitudinalForce(obj, normalLoad, slipRatio, mu)
            % COMPUTELONGITUDINALFORCE Longitudinal force [N] for a single evaluation
            %   Fx = computeLongitudinalForce(obj, normalLoad, slipRatio, mu)
            %
            %   This is the TireModel interface method for standalone queries.
            %   For per-corner state tracking, use updateCorner() instead.
            
            if normalLoad <= 0
                Fx = 0;
                return;
            end
            slipRatio = obj.clampSlipRatio(slipRatio);
            inputsMF = [normalLoad, slipRatio, 0, 0, 0, ...
                obj.tireConstants.refVelocity, obj.tireConstants.nomPressure];
            outputs = mfeval(obj.tireConstants.params, inputsMF, 111);
            
            Fx = outputs(:,1);
        end
        
        function peakMu = getPeakFriction(obj, normalLoad)
            % GETPEAKFRICTION Peak friction coefficient at given load
            %   peakMu = getPeakFriction(obj, normalLoad)
            %
            %   Scans the lateral force curve to find max |Fy|/Fz.
            %   Accounts for load sensitivity inherent in the Magic Formula.
            
            if normalLoad <= 0
                peakMu = 0;
                return;
            end
            
            peakMu = obj.getCachedPeakMu(normalLoad, 0, ...
                obj.tireConstants.nomPressure, obj.tireConstants.params);
        end
        
        %% ---- Slip angle computation ----
        
        function slipAngles = computeSlipAngles(obj, vx, vy, yawRate, steerInput, vehicleManager)
            % COMPUTESLIPANGLES Compute per-corner tire slip angles [rad]
            %   slipAngles = computeSlipAngles(vx, vy, yawRate, steerInput, vehicleManager)
            %
            %   Uses per-corner wheel kinematics:
            %     alpha_i = steer_i + toe_i - atan2(vy_i, vx_i)
            %
            %   steer_i and toe_i come from the suspension geometry model,
            %   allowing Ackermann, bump steer, rear steer, and toe curves.
            %
            %   Inputs:
            %     vx              - forward velocity [m/s]
            %     vy              - lateral velocity at CG [m/s]
            %     yawRate         - yaw rate [rad/s]
            %     steerInput      - driver steering input [rad]
            %     vehicleManager  - vehicle/component manager with geometry
            %
            %   Returns struct with:
            %     slipAngles.FL, .FR, .RL, .RR  [rad]

            slipAngles = struct('FL', 0, 'FR', 0, 'RL', 0, 'RR', 0);
            
            % At very low speed, slip angles are undefined → return zeros
            if vx < 0.5
                return;
            end
            
            suspensionKinematics = obj.getSuspensionKinematics(vehicleManager, steerInput);
            [xFL, yFL] = obj.getKinematicPosition(vehicleManager, 'FL', suspensionKinematics.FL);
            [xFR, yFR] = obj.getKinematicPosition(vehicleManager, 'FR', suspensionKinematics.FR);
            [xRL, yRL] = obj.getKinematicPosition(vehicleManager, 'RL', suspensionKinematics.RL);
            [xRR, yRR] = obj.getKinematicPosition(vehicleManager, 'RR', suspensionKinematics.RR);

            slipAngles.FL = obj.computeCornerSlipAngle(vx, vy, yawRate, ...
                xFL, yFL, suspensionKinematics.FL);
            slipAngles.FR = obj.computeCornerSlipAngle(vx, vy, yawRate, ...
                xFR, yFR, suspensionKinematics.FR);
            slipAngles.RL = obj.computeCornerSlipAngle(vx, vy, yawRate, ...
                xRL, yRL, suspensionKinematics.RL);
            slipAngles.RR = obj.computeCornerSlipAngle(vx, vy, yawRate, ...
                xRR, yRR, suspensionKinematics.RR);
        end
        
        %% ---- Slip ratio computation ----
        
        function kappa = computeSlipRatio(obj, cornerState, vehicleSpeed)
            % COMPUTESLIPRATIO Compute longitudinal slip ratio for one corner
            %   kappa = computeSlipRatio(cornerState, vehicleSpeed)
            %
            %   Slip ratio definition (the contract used by MFeval):
            %     kappa = (omega * R - V) / max(|V|, lowSpeedFloor)
            %
            %   kappa > 0 → driving (wheel faster than vehicle)
            %   kappa < 0 → braking (wheel slower than vehicle)
            %
            %   Inputs:
            %     cornerState  - TireState with angularVelocity and wheelRadius
            %     vehicleSpeed - Vehicle forward speed [m/s]
            %
            %   Returns:
            %     kappa - Slip ratio [-1, KPUMAX]
            
            omega = cornerState.angularVelocity;
            R     = cornerState.wheelRadius;
            V     = max(vehicleSpeed, 0);   % no reverse
            
            kappa = obj.computeKinematicSlipRatio(omega * R, V);
        end

        function kappa = computeSlipRatioFromKinematics(obj, cornerState, longitudinalSpeed)
            % COMPUTESLIPRATIOFROMKINEMATICS MFeval-consistent local-wheel slip.
            % This is the shared simulator/contact-solver entry point. Unlike
            % computeSlipRatio, it preserves the signed wheel-frame road speed.
            kappa = obj.computeKinematicSlipRatio( ...
                cornerState.angularVelocity * cornerState.wheelRadius, ...
                longitudinalSpeed);
        end
        
        function updateWheelDynamics(obj, cornerState, driveTorque, brakeTorque, dt, inertia, longitudinalSpeed)
            % UPDATEWHEELDYNAMICS Integrate wheel angular velocity forward
            %   updateWheelDynamics(cornerState, driveTorque, brakeTorque, dt)
            %   updateWheelDynamics(cornerState, driveTorque, brakeTorque, dt, inertia)
            %
            %   Rotational equation of motion:
            %     I * d(omega)/dt = T_drive - sign*T_brake - Fx*R - T_resist
            %
            %   where:
            %     T_drive  = applied drive torque at this wheel [Nm]
            %     T_brake  = applied brake torque at this wheel [Nm] (positive value)
            %     Fx       = longitudinal tire force from previous evaluation [N]
            %     R        = effective wheel radius [m]
            %     I        = wheel rotational inertia [kg·m^2]
            %     T_resist = rolling-resistance + bearing-drag torque opposing
            %                spin, so a free-rolling wheel coast-down is driven
            %                by the contact patch (T_rr = Crr*Fz*R + C_b*omega).
            %
            %   Uses explicit Euler integration.
            %
            %   Inputs:
            %     cornerState - TireState handle (angularVelocity is mutated)
            %     driveTorque - Net drive torque at this wheel [Nm]
            %     brakeTorque - Brake torque at this wheel [Nm] (positive magnitude)
            %     dt          - Timestep [s]
            %     inertia     - Optional per-wheel inertia override [kg*m^2].
            %                   Defaults to obj.wheelInertia. Differential-
            %                   carrier inertia is handled by the coupled
            %                   driven-wheel method below.

            omega = cornerState.angularVelocity;
            R     = cornerState.wheelRadius;
            I     = obj.wheelInertia;
            if nargin >= 6 && ~isempty(inertia) && inertia > 0
                I = inertia;  % per-wheel override (driven axle: +reflected rotor)
            end
            if nargin < 7 || isempty(longitudinalSpeed)
                longitudinalSpeed = omega * R;
            end
            Fx    = cornerState.Fx;  % from previous tire evaluation

            % Net torque: drive accelerates, brake and tire Fx decelerate
            % Fx > 0 means driving force → reaction torque opposes wheel spin
            brakeSign = obj.computeBrakeTorqueSign(omega, longitudinalSpeed, driveTorque);

            % Resistance torque opposing spin: rolling resistance (load-proportional)
            % plus viscous bearing drag (speed-proportional). Kept separate from
            % the brake torque so its magnitude does not depend on the brake path.
            spinSign = sign(omega);
            if spinSign == 0
                spinSign = brakeSign;  % use the brake/roll direction when omega ~ 0
            end
            Fz = max(cornerState.normalForce, 0);
            resistTorque = spinSign * (obj.rollingResistanceCoeff * Fz * R ...
                + obj.bearingDragCoeff * abs(omega));

            netTorque = driveTorque - brakeSign * brakeTorque - Fx * R - resistTorque;

            % Angular acceleration
            alpha = netTorque / I;

            % Euler integration
            omega_new = omega + alpha * dt;

            cornerState.angularVelocity = omega_new;
        end

        function updateDrivenWheelPairDynamics(obj, leftState, rightState, ...
                driveTorqueLeft, driveTorqueRight, brakeTorqueLeft, brakeTorqueRight, ...
                dt, inertiaLeft, inertiaRight, reflectedRotorInertia, ...
                longitudinalSpeedLeft, longitudinalSpeedRight)
            % UPDATEDRIVENWHEELPAIRDYNAMICS Integrate a differential axle.
            % Reflected motor inertia belongs to carrier motion,
            % omega_c = (omega_L + omega_R)/2. It resists common-mode
            % acceleration without adding phantom differential-mode inertia.
            if nargin < 9 || isempty(inertiaLeft)
                inertiaLeft = obj.wheelInertia;
            end
            if nargin < 10 || isempty(inertiaRight)
                inertiaRight = obj.wheelInertia;
            end
            if nargin < 11 || isempty(reflectedRotorInertia) || ...
                    ~isfinite(reflectedRotorInertia)
                reflectedRotorInertia = 0;
            end
            if nargin < 12 || isempty(longitudinalSpeedLeft)
                longitudinalSpeedLeft = leftState.angularVelocity * leftState.wheelRadius;
            end
            if nargin < 13 || isempty(longitudinalSpeedRight)
                longitudinalSpeedRight = rightState.angularVelocity * rightState.wheelRadius;
            end

            tau = [obj.computeWheelNetTorque(leftState, driveTorqueLeft, ...
                       brakeTorqueLeft, longitudinalSpeedLeft); ...
                   obj.computeWheelNetTorque(rightState, driveTorqueRight, ...
                       brakeTorqueRight, longitudinalSpeedRight)];
            carrierCoupling = max(reflectedRotorInertia, 0) / 4;
            massMatrix = [max(inertiaLeft, eps) + carrierCoupling, carrierCoupling; ...
                          carrierCoupling, max(inertiaRight, eps) + carrierCoupling];
            angularAccel = massMatrix \ tau;
            leftState.angularVelocity = leftState.angularVelocity + angularAccel(1) * dt;
            rightState.angularVelocity = rightState.angularVelocity + angularAccel(2) * dt;
        end

        function solveWheelContact(obj, cornerState, normalLoad, slipAngle, ...
                camberAngle, mu, longitudinalSpeed, driveTorque, brakeTorque, dt)
            % SOLVEWHEELCONTACT Semi-implicitly couple wheel speed and tire Fx.
            %   I*domega/dt = T_drive - T_brake - Fx(kappa(omega))*R

            if nargin < 10 || isempty(dt)
                dt = 0.001;
            end

            omegaOld = cornerState.angularVelocity;
            omegaNew = omegaOld;
            R = max(cornerState.wheelRadius, eps);
            I = max(obj.wheelInertia, eps);
            dt = max(dt, 0);
            slipAngle = lts.util.clamp(slipAngle, -0.3, 0.3);

            finalFx = 0;
            finalFy = 0;
            finalMx = 0;
            finalMy = 0;
            finalMz = 0;
            finalPeakMu = 0;
            finalKappa = cornerState.slipRatio;
            isRight = obj.isRightCorner(cornerState);

            for iter = 1:5 %#ok<NASGU>
                finalKappa = obj.computeSlipRatioFromOmega( ...
                    cornerState, omegaNew, longitudinalSpeed);
                [finalFx, finalFy, finalMx, finalMy, finalMz, finalPeakMu] = ...
                    obj.evaluateForces(normalLoad, slipAngle, finalKappa, ...
                    camberAngle, mu, longitudinalSpeed, false, isRight);

                brakeSign = obj.computeBrakeTorqueSign( ...
                    omegaNew, longitudinalSpeed, driveTorque);
                netTorque = driveTorque - brakeSign * brakeTorque - finalFx * R;
                omegaCandidate = omegaOld + (netTorque / I) * dt;

                if abs(omegaCandidate - omegaNew) < 1e-4
                    omegaNew = omegaCandidate;
                    break;
                end
                omegaNew = omegaCandidate;
            end

            finalKappa = obj.computeSlipRatioFromOmega( ...
                cornerState, omegaNew, longitudinalSpeed);
            [finalFx, finalFy, finalMx, finalMy, finalMz, finalPeakMu] = ...
                obj.evaluateForces(normalLoad, slipAngle, finalKappa, ...
                camberAngle, mu, longitudinalSpeed, true, isRight);

            cornerState.normalForce = normalLoad;
            cornerState.slipAngle = slipAngle;
            cornerState.slipRatio = finalKappa;
            cornerState.camberAngle = camberAngle;
            cornerState.angularVelocity = omegaNew;
            cornerState.Fx = finalFx;
            cornerState.Fy = finalFy;
            cornerState.Mx = finalMx;
            cornerState.My = finalMy;
            cornerState.Mz = finalMz;
            cornerState.peakMu = finalPeakMu;
        end
        
        %% ---- All-corners batch update ----
        
        function updateAllCorners(obj, Fz_FL, Fz_FR, Fz_RL, Fz_RR, ...
                slipAngle_FL, slipAngle_FR, slipAngle_RL, slipAngle_RR, ...
                kappa_FL, kappa_FR, kappa_RL, kappa_RR, ...
                camber_FL, camber_FR, camber_RL, camber_RR, dt, longSpeeds, ...
                surfaceMu, computePeakMu, relaxationMode)
            % UPDATEALLCORNERS Evaluate all four corners at once
            %   updateAllCorners(Fz_FL, Fz_FR, Fz_RL, Fz_RR, ...
            %       slipAngle_FL, slipAngle_FR, slipAngle_RL, slipAngle_RR, ...
            %       kappa_FL, kappa_FR, kappa_RL, kappa_RR)
            %   updateAllCorners(..., camber_FL..camber_RR, dt, longSpeeds)
            %
            %   Updates all four corner states with per-corner slip ratios.
            %   Slip angles/ratios passed in are the steady-state (kinematic)
            %   values; a first-order contact-patch relaxation is applied
            %   before MFeval when dt and longSpeeds are supplied.
            %   Camber defaults to 0 for all corners.
            %
            %   Legacy surface-Mu inputs are accepted but ignored; grip comes
            %   directly from the tire file.

            if nargin < 14
                camber_FL = 0;
                camber_FR = 0;
                camber_RL = 0;
                camber_RR = 0;
            end
            if nargin < 18 || isempty(dt)
                dt = 0;
            end
            if nargin < 19 || isempty(longSpeeds)
                longSpeeds = repmat(obj.tireConstants.refVelocity, 4, 1);
            else
                longSpeeds = longSpeeds(:);
            end
            if nargin < 20 || isempty(surfaceMu)
                surfaceMu = obj.surfaceMuReference;
            end
            if nargin < 21 || isempty(computePeakMu)
                computePeakMu = true;
            end
            if nargin < 22
                relaxationMode = '';
            end
            relaxationMode = obj.resolveRelaxationMode(dt, relaxationMode);

            Fz = [Fz_FL; Fz_FR; Fz_RL; Fz_RR];
            ssAlpha = lts.util.clamp(...
                [slipAngle_FL; slipAngle_FR; slipAngle_RL; slipAngle_RR], -0.3, 0.3);
            ssKappa = obj.clampSlipRatio( ...
                [kappa_FL; kappa_FR; kappa_RL; kappa_RR]);
            gamma = [camber_FL; camber_FR; camber_RL; camber_RR];
            longSpeed = longSpeeds(:);
            states = {obj.FL, obj.FR, obj.RL, obj.RR};
            P = obj.tireConstants.nomPressure;
            params = obj.tireConstants.params;
            advanceMode = strcmp(relaxationMode, 'advance');
            holdMode = strcmp(relaxationMode, 'hold');
            % Contact-patch load response: the Magic Formula evaluates at a
            % load that tracks the suspension Fz through the same exact
            % exponential lag used for slip. Committed with the same
            % advance/preview/hold/steady semantics as the slip states.
            FzEval = obj.applyLoadRelaxation(states, Fz, longSpeed, dt, ...
                relaxationMode, holdMode);

            if computePeakMu
                peakVx = obj.computeMFevalSpeed(longSpeed);
                peakFzKey = round(FzEval / 10) * 10;
                peakGammaKey = round(gamma * 1000) / 1000;
                peakVxKey = round(peakVx * 10) / 10;
                peakNumericKey = obj.packPeakMuCacheKey( ...
                    peakFzKey, peakGammaKey, P, peakVxKey);
            end

            % Apply per-corner relaxation to obtain the transient (force-
            % producing) slip. The lagged slip stored on each corner only
            % advances when dt > 0 (i.e. the final call of a step); during
            % intermediate wheel-solve iterations (dt = 0) the force is
            % evaluated at the steady-state kinematic slip while the lagged
            % state is preserved for the next physics step.
            previousAlpha = [obj.FL.slipAngle; obj.FR.slipAngle; ...
                             obj.RL.slipAngle; obj.RR.slipAngle];
            previousKappa = [obj.FL.slipRatio; obj.FR.slipRatio; ...
                             obj.RL.slipRatio; obj.RR.slipRatio];
            sigmaAlpha = obj.relaxationLength;
            sigmaKappa = obj.resolvedLongitudinalRelaxationLength();
            if holdMode
                alpha = ssAlpha;
                kappa = ssKappa;
                if sigmaAlpha > 0
                    alpha = previousAlpha;
                end
                if sigmaKappa > 0
                    kappa = previousKappa;
                end
            elseif dt <= 0 || strcmp(relaxationMode, 'steady')
                alpha = ssAlpha;
                kappa = ssKappa;
            else
                V_eff = max(abs(longSpeed), 1.0);
                if sigmaAlpha > 0
                    decayAlpha = exp(-V_eff * dt / sigmaAlpha);
                    alpha = ssAlpha - ...
                        (ssAlpha - previousAlpha) .* decayAlpha;
                else
                    alpha = ssAlpha;
                end
                if sigmaKappa > 0
                    decayKappa = exp(-V_eff * dt / sigmaKappa);
                    kappa = ssKappa - ...
                        (ssKappa - previousKappa) .* decayKappa;
                else
                    kappa = ssKappa;
                end
                kappa = obj.clampSlipRatio(kappa);
            end
            % A stopped contact patch has no kinematic longitudinal slip.
            % Never retain a previous braking force after both wheel and road
            % speed have reached zero.
            rest = abs(longSpeed) <= 1e-9 & abs(ssKappa) <= 1e-12;
            kappa(rest) = 0;
            for i = 1:4
                states{i}.normalForce = Fz(i);
                states{i}.ssSlipAngle = ssAlpha(i);
                states{i}.ssSlipRatio = ssKappa(i);
                states{i}.camberAngle = gamma(i);
                if advanceMode && dt > 0
                    % Commit the advanced lagged state for next step.
                    states{i}.slipAngle = alpha(i);
                    states{i}.slipRatio = kappa(i);
                end
            end

            active = FzEval > 0;
            if any(active)
                nActive = nnz(active);
                Vx = obj.computeMFevalSpeed(longSpeed(active));
                alphaEval = obj.evaluationSlipAnglesByCorner(alpha);
                alphaEval([2, 4]) = -alphaEval([2, 4]);
                inputsMF = [FzEval(active), kappa(active), alphaEval(active), ...
                    gamma(active), zeros(nActive, 1), ...
                    Vx, repmat(P, nActive, 1)];
                outputs = mfeval(params, inputsMF, 111);

                activeIdx = find(active);
                for j = 1:numel(activeIdx)
                    i = activeIdx(j);
                    if computePeakMu
                        numericKey = peakNumericKey(i);
                        if states{i}.peakMuCacheKey == numericKey
                            rawPeakMu = states{i}.rawPeakMu;
                        elseif isKey(obj.peakMuNumericCache, numericKey)
                            rawPeakMu = obj.peakMuNumericCache(numericKey);
                            states{i}.peakMuCacheKey = numericKey;
                            states{i}.rawPeakMu = rawPeakMu;
                        else
                            rawPeakMu = obj.computePeakMuInternal( ...
                                FzEval(i), gamma(i), P, params, peakVx(i));
                            obj.peakMuNumericCache(numericKey) = rawPeakMu;
                            key = sprintf('%.0f_%.3f_%.0f_%.1f', ...
                                peakFzKey(i), peakGammaKey(i), P, peakVxKey(i));
                            obj.peakMuCache(key) = rawPeakMu;
                            states{i}.peakMuCacheKey = numericKey;
                            states{i}.rawPeakMu = rawPeakMu;
                        end
                        states{i}.peakMu = rawPeakMu;
                    end
                    states{i}.Fx = outputs(j,1);
                    states{i}.My = outputs(j,5);
                    if i == 2 || i == 4
                        states{i}.Fy = outputs(j,2);
                        states{i}.Mx = -outputs(j,4);
                        states{i}.Mz = outputs(j,6);
                    else
                        states{i}.Fy = -outputs(j,2);
                        states{i}.Mx = outputs(j,4);
                        states{i}.Mz = -outputs(j,6);
                    end
                end
            end

            inactiveIdx = find(~active);
            for j = 1:numel(inactiveIdx)
                i = inactiveIdx(j);
                states{i}.Fx = 0;
                states{i}.Fy = 0;
                states{i}.Mx = 0;
                states{i}.My = 0;
                states{i}.Mz = 0;
                states{i}.peakMu = 0;
            end
        end
        
        function updateAllFromState(obj, state, vehicleManager, cornerLoads, mu)
            % UPDATEALLFROMSTATE Compute slip angles/ratios and update all corners
            %   updateAllFromState(state, vehicleManager, cornerLoads)
            %
            %   Computes per-corner slip angles from vehicle kinematics and
            %   per-corner slip ratios from wheel rotational state, then
            %   delegates to updateAllCorners().
            %
            %   Inputs:
            %     state          - lts.simulation.VehicleState with speed, vy, yawRate, steer
            %     vehicleManager - lts.vehicle.VehicleManager for geometry (wheelbase, weight dist)
            %     cornerLoads    - struct with .FL, .FR, .RL, .RR normal forces [N]
            if nargin < 5 || isempty(mu)
                mu = obj.surfaceMuReference;
            end

            % Compute per-corner slip angles and suspension geometry
            slipAngles = obj.computeSlipAngles( ...
                state.speed, state.vy, state.yawRate, state.steer, ...
                vehicleManager);
            suspensionKinematics = obj.getSuspensionKinematics(vehicleManager, state.steer);

            % Compute per-corner slip ratios from wheel rotational state
            kappa_FL = obj.computeSlipRatio(obj.FL, state.speed);
            kappa_FR = obj.computeSlipRatio(obj.FR, state.speed);
            kappa_RL = obj.computeSlipRatio(obj.RL, state.speed);
            kappa_RR = obj.computeSlipRatio(obj.RR, state.speed);

            obj.updateAllCorners( ...
                cornerLoads.FL, cornerLoads.FR, cornerLoads.RL, cornerLoads.RR, ...
                slipAngles.FL, slipAngles.FR, slipAngles.RL, slipAngles.RR, ...
                kappa_FL, kappa_FR, kappa_RL, kappa_RR, ...
                suspensionKinematics.FL.camberAngle, ...
                suspensionKinematics.FR.camberAngle, ...
                suspensionKinematics.RL.camberAngle, ...
                suspensionKinematics.RR.camberAngle, 0, [], mu, true);
        end
    end
    
    methods (Access = private)
        function [alpha, kappa] = applyRelaxation(obj, cornerState, ssAlpha, ssKappa, longSpeed, dt, relaxationMode)
            % APPLYRELAXATION First-order contact-patch slip lag
            %   sigma * d(alpha)/dt + V * alpha = V * alpha_ss
            % Solved with the exact, unconditionally-stable exponential form
            %   alpha = alpha_ss - (alpha_ss - alpha_prev) * exp(-V_eff*dt/sigma)
            % which is stable for any dt (explicit Euler would be stiff here).
            % Each state has its own physical relaxation length. NaN for the
            % longitudinal value retains the legacy shared-length behavior.
            sigmaAlpha = obj.relaxationLength;
            sigmaKappa = obj.resolvedLongitudinalRelaxationLength();
            clearLongitudinalSlipAtRest = ...
                abs(longSpeed) <= 1e-9 && abs(ssKappa) <= 1e-12;
            if nargin < 7 || isempty(relaxationMode)
                relaxationMode = obj.resolveRelaxationMode(dt, '');
            end
            if strcmp(relaxationMode, 'hold')
                alpha = ssAlpha;
                kappa = ssKappa;
                if sigmaAlpha > 0
                    alpha = cornerState.slipAngle;
                end
                if sigmaKappa > 0
                    kappa = cornerState.slipRatio;
                end
                if clearLongitudinalSlipAtRest
                    kappa = 0;
                end
                return;
            end
            if dt <= 0 || strcmp(relaxationMode, 'steady')
                alpha = ssAlpha;
                kappa = ssKappa;
                if clearLongitudinalSlipAtRest
                    kappa = 0;
                end
                return;
            end

            % Effective rolling speed for the lag time constant. A floor keeps
            % the transient model well-conditioned near standstill.
            V_eff = max(abs(longSpeed), 1.0);
            if sigmaAlpha > 0
                decayAlpha = exp(-V_eff * dt / sigmaAlpha);
                alpha = ssAlpha - ...
                    (ssAlpha - cornerState.slipAngle) * decayAlpha;
            else
                alpha = ssAlpha;
            end
            if sigmaKappa > 0
                decayKappa = exp(-V_eff * dt / sigmaKappa);
                kappa = ssKappa - ...
                    (ssKappa - cornerState.slipRatio) * decayKappa;
            else
                kappa = ssKappa;
            end

            kappa = obj.clampSlipRatio(kappa);
            if clearLongitudinalSlipAtRest
                kappa = 0;
            end
        end

        function sigma = resolvedLongitudinalRelaxationLength(obj)
            sigma = obj.longitudinalRelaxationLength;
            if isempty(sigma) || ~isnumeric(sigma) || ~isscalar(sigma) || ...
                    ~isfinite(sigma) || sigma < 0
                sigma = obj.relaxationLength;
            end
        end

        function sigma = resolvedNormalLoadRelaxationLength(obj)
            % NaN/negative/empty disables the load-response filter (legacy
            % instantaneous-Fz behavior); a finite positive value is used
            % directly. Unlike the longitudinal slip length there is no
            % shared value to inherit: 0 is the explicit "off" default.
            sigma = obj.normalLoadRelaxationLength;
            if isempty(sigma) || ~isnumeric(sigma) || ~isscalar(sigma) || ...
                    ~isfinite(sigma) || sigma < 0
                sigma = 0;
            end
        end

        function FzEval = applyLoadRelaxation(obj, states, Fz, longSpeed, ...
                dt, relaxationMode, holdMode)
            % APPLYLOADRELAXATION Contact-patch load-response lag.
            %   FzEval = applyLoadRelaxation(states, Fz, longSpeed, dt, mode)
            %
            % Exact exponential first-order lag on the normal load the
            % Magic Formula sees, sigma_Fz/V time constant, mirroring
            % applyRelaxation's advance/preview/hold/steady semantics:
            %   advance — lag advanced and committed to the corner state
            %   preview — lag advanced for evaluation only (no commit)
            %   hold    — evaluate at the previously committed lagged load
            %   steady  — evaluate at the instantaneous load, no commit
            % A corner whose true load has reached zero loses its lagged
            % load immediately (contact loss is unambiguous), and the
            % filter seeds from the incoming load on first evaluation so a
            % run does not start with a synthetic load transient.
            Fz = Fz(:);
            n = numel(Fz);
            FzEval = Fz;
            sigmaFz = obj.resolvedNormalLoadRelaxationLength();
            if sigmaFz <= 0
                % Filter disabled: keep the mirrored state coherent, but
                % avoid handle writes in the hot loop once seeded.
                for i = 1:n
                    if ~(states{i}.relaxedNormalLoad == Fz(i))
                        states{i}.relaxedNormalLoad = Fz(i);
                    end
                end
                return;
            end

            previousFz = nan(n, 1);
            for i = 1:n
                previousFz(i) = states{i}.relaxedNormalLoad;
            end
            needsSeed = isnan(previousFz);
            previousFz(needsSeed) = Fz(needsSeed);

            if holdMode
                FzEval = previousFz;
            elseif dt <= 0 || strcmp(relaxationMode, 'steady')
                FzEval = Fz;
            else
                V_eff = max(abs(longSpeed(:)), 1.0);
                decayFz = exp(-V_eff * dt / sigmaFz);
                FzEval = Fz - (Fz - previousFz) .* decayFz;
            end

            % Contact loss clears the lagged load without a decay tail.
            airborne = Fz <= 0;
            FzEval(airborne) = 0;
            FzEval = max(FzEval, 0);

            advanceMode = strcmp(relaxationMode, 'advance');
            for i = 1:n
                if advanceMode && dt > 0
                    states{i}.relaxedNormalLoad = FzEval(i);
                end
            end
        end

        function suspensionKinematics = getSuspensionKinematics(~, vehicleManager, steerInput)
            if ~isempty(vehicleManager.suspension) && ...
                    ismethod(vehicleManager.suspension, 'getCornerKinematics')
                suspensionKinematics = vehicleManager.suspension.getCornerKinematics();
                return;
            end

            suspensionKinematics = struct();
            suspensionKinematics.FL = struct('camberAngle', 0, 'toeAngle', 0, 'steerAngle', steerInput);
            suspensionKinematics.FR = struct('camberAngle', 0, 'toeAngle', 0, 'steerAngle', steerInput);
            suspensionKinematics.RL = struct('camberAngle', 0, 'toeAngle', 0, 'steerAngle', 0);
            suspensionKinematics.RR = struct('camberAngle', 0, 'toeAngle', 0, 'steerAngle', 0);
        end

        function [x, y] = getKinematicPosition(obj, vehicleManager, corner, kin)
            if isfield(kin, 'xPosition') && isfield(kin, 'yPosition')
                x = kin.xPosition;
                y = kin.yPosition;
                return;
            end

            [x, y] = obj.getWheelPosition(vehicleManager, corner);
        end

        function [x, y] = getWheelPosition(~, vehicleManager, corner)
            frontArm = vehicleManager.wheelbase * (1 - vehicleManager.staticFrontWeight);
            rearArm = vehicleManager.wheelbase * vehicleManager.staticFrontWeight;
            halfTrack = vehicleManager.trackWidth / 2;

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

        function alpha = computeCornerSlipAngle(~, vx, vy, yawRate, x, y, kin)
            % Contact patch velocity = CG velocity + yaw-rate cross position.
            % Rotate that velocity into the steered/toed wheel frame; slip
            % angle is positive when the tire must generate force to the left.
            vxCorner = vx - yawRate * y;
            vyCorner = vy + yawRate * x;
            wheelHeading = kin.steerAngle + kin.toeAngle;
            longSpeed = vxCorner * cos(wheelHeading) + vyCorner * sin(wheelHeading);
            latSpeed = -vxCorner * sin(wheelHeading) + vyCorner * cos(wheelHeading);
            alpha = atan2(-latSpeed, max(abs(longSpeed), 0.1));
        end

        function kappa = computeSlipRatioFromOmega(obj, cornerState, omega, longitudinalSpeed)
            % Same slip convention as lts.simulation.Simulator.computeLocalSlipRatio, used by
            % the older single-corner contact solver.
            wheelSpeed = omega * cornerState.wheelRadius;
            kappa = obj.computeKinematicSlipRatio(wheelSpeed, longitudinalSpeed);
        end

        function brakeSign = computeBrakeTorqueSign(~, omega, longitudinalSpeed, driveTorque)
            if abs(omega) > 1e-6
                brakeSign = sign(omega);
            elseif abs(longitudinalSpeed) > 1e-6
                brakeSign = sign(longitudinalSpeed);
            elseif abs(driveTorque) > 1e-6
                brakeSign = sign(driveTorque);
            else
                brakeSign = 0;
            end
        end

        function [Fx, Fy, Mx, My, Mz, peakMu] = evaluateForces(obj, ...
                Fz, alpha, kappa, gamma, surfaceMu, longitudinalSpeed, ...
                computePeakMu, isRight)
            % EVALUATEFORCES Thin wrapper around MFeval's combined-slip mode.
            % Inputs follow MFeval's [Fz, kappa, alpha, gamma, phit, Vx, P]
            % order. The raw .tir file is treated as the dry-reference
            % surface. Legacy surfaceMu is ignored.
            if Fz <= 0
                Fx = 0;
                Fy = 0;
                Mx = 0;
                My = 0;
                Mz = 0;
                peakMu = 0;
                return;
            end
            if nargin < 8 || isempty(computePeakMu)
                computePeakMu = true;
            end
            if nargin < 9 || isempty(isRight)
                isRight = false;
            end

            alpha = obj.evaluationSlipAngle(alpha);
            if isRight
                alpha = -alpha;
            end
            kappa = obj.clampSlipRatio(kappa);
            P = obj.tireConstants.nomPressure;
            params = obj.tireConstants.params;
            Vx = obj.computeMFevalSpeed(longitudinalSpeed);
            inputsMF = [Fz, kappa, alpha, gamma, 0, Vx, P];
            outputs = mfeval(params, inputsMF, 111);

            if computePeakMu
                rawPeakMu = obj.getCachedPeakMu(Fz, gamma, P, params, longitudinalSpeed);
                peakMu = rawPeakMu;
            else
                peakMu = 0;
            end
            Fx = outputs(:,1);
            My = outputs(:,5);
            if isRight
                Fy = outputs(:,2);
                Mx = -outputs(:,4);
                Mz = outputs(:,6);
            else
                Fy = -outputs(:,2);
                Mx = outputs(:,4);
                Mz = -outputs(:,6);
            end
        end

        function netTorque = computeWheelNetTorque(obj, cornerState, ...
                driveTorque, brakeTorque, longitudinalSpeed)
            omega = cornerState.angularVelocity;
            radius = cornerState.wheelRadius;
            brakeSign = obj.computeBrakeTorqueSign( ...
                omega, longitudinalSpeed, driveTorque);
            spinSign = sign(omega);
            if spinSign == 0
                spinSign = brakeSign;
            end
            normalForce = max(cornerState.normalForce, 0);
            resistTorque = spinSign * ( ...
                obj.rollingResistanceCoeff * normalForce * radius + ...
                obj.bearingDragCoeff * abs(omega));
            netTorque = driveTorque - brakeSign * brakeTorque ...
                - cornerState.Fx * radius - resistTorque;
        end

        function tf = isRightCorner(obj, cornerState)
            tf = isequal(cornerState, obj.FR) || isequal(cornerState, obj.RR);
        end

        function [surfaceMu, dt, longSpeed, computePeakMu, relaxationMode] = parseCornerOptionalArgs(obj, varargin)
            surfaceMu = obj.surfaceMuReference;
            dt = 0;
            longSpeed = obj.tireConstants.refVelocity;
            computePeakMu = true;
            relaxationMode = '';

            nArgs = numel(varargin);
            if nArgs == 0
                return;
            elseif nArgs == 1
                dt = varargin{1};
            elseif nArgs == 2
                % Legacy form: updateCorner(..., camber, dt, longSpeed).
                dt = varargin{1};
                longSpeed = varargin{2};
            else
                surfaceMu = varargin{1};
                dt = varargin{2};
                longSpeed = varargin{3};
                if nArgs >= 4 && ~isempty(varargin{4})
                    computePeakMu = logical(varargin{4});
                end
                if nArgs >= 5 && ~isempty(varargin{5})
                    relaxationMode = varargin{5};
                end
            end

            if isempty(surfaceMu)
                surfaceMu = obj.surfaceMuReference;
            end
            if isempty(dt)
                dt = 0;
            end
            if isempty(longSpeed)
                longSpeed = obj.tireConstants.refVelocity;
            end
        end

        function mode = resolveRelaxationMode(~, dt, mode)
            if nargin < 3 || isempty(mode)
                if dt > 0
                    mode = 'advance';
                else
                    mode = 'steady';
                end
                return;
            end

            mode = lower(char(mode));
            validModes = {'advance', 'preview', 'steady', 'hold'};
            if ~any(strcmp(mode, validModes))
                error('PacejkaTire:InvalidRelaxationMode', ...
                    'relaxationMode must be advance, preview, steady, or hold.');
            end
        end

        function alphaEval = evaluationSlipAngle(obj, alpha)
            scale = obj.lateralStiffnessScale;
            if isempty(scale) || ~isfinite(scale) || scale <= 0
                scale = 1.0;
            end
            alphaEval = lts.util.clamp(alpha .* scale, -0.3, 0.3);
        end

        function alphaEval = evaluationSlipAnglesByCorner(obj, alpha)
            alphaEval = obj.evaluationSlipAngle(alpha);
            scale = obj.lateralStiffnessScaleByCorner;
            if isempty(scale) || ~isnumeric(scale) || numel(scale) ~= 4 || ...
                    any(~isfinite(scale)) || any(scale <= 0)
                scale = ones(4, 1);
            else
                scale = scale(:);
            end
            if numel(alphaEval) == 4
                alphaEval = lts.util.clamp(alphaEval(:) .* scale, -0.3, 0.3);
            end
        end

        function kappa = computeKinematicSlipRatio(obj, wheelSpeed, longitudinalSpeed)
            % MFeval reconstructs wheel speed as omega*R=(1+kappa)*Vx,
            % hence road speed (not wheel speed) belongs in the denominator.
            % Regularize only near zero road speed and explicitly clear the
            % indeterminate 0/0 state. A truly locked wheel remains kappa=-1
            % for every nonzero forward road speed.
            restTolerance = 1e-9;
            if abs(wheelSpeed) <= restTolerance && ...
                    abs(longitudinalSpeed) <= restTolerance
                kappa = 0;
                return;
            end

            if isnan(obj.cachedMFevalLowSpeed)
                obj.cachedMFevalLowSpeed = obj.resolveMFevalLowSpeed();
            end
            slipSpeedFloor = obj.cachedMFevalLowSpeed;
            if longitudinalSpeed > restTolerance && ...
                    abs(wheelSpeed) <= restTolerance
                rawKappa = -1;
            else
                rawKappa = (wheelSpeed - longitudinalSpeed) / ...
                    max(abs(longitudinalSpeed), slipSpeedFloor);
            end
            kappa = obj.clampSlipRatio(rawKappa);
        end

        function kappa = clampSlipRatio(obj, kappa)
            % Physical forward braking bottoms out at a locked wheel (-1).
            % Preserve the tire file/MFeval positive-drive range instead of
            % imposing the former artificial +1 ceiling.
            upperLimit = 1.5;
            params = obj.tireConstants.params;
            if isfield(params, 'KPUMAX') && isscalar(params.KPUMAX) && ...
                    isfinite(params.KPUMAX)
                upperLimit = max(double(params.KPUMAX), 0);
            end
            kappa = min(max(kappa, -1), upperLimit);
        end

        function Vx = computeMFevalSpeed(obj, longitudinalSpeed)
            % MFeval has a low-speed singularity/guard; feed it speed magnitude
            % above VXLOW while the simulator's own slip definitions handle
            % sign and near-zero blending.
            if isnan(obj.cachedMFevalLowSpeed)
                obj.cachedMFevalLowSpeed = obj.resolveMFevalLowSpeed();
            end
            Vx = max(abs(longitudinalSpeed), obj.cachedMFevalLowSpeed);
        end

        function lowSpeed = resolveMFevalLowSpeed(obj)
            lowSpeedLimit = 0.1;
            if isfield(obj.tireConstants.params, 'VXLOW')
                lowSpeedLimit = max(lowSpeedLimit, obj.tireConstants.params.VXLOW);
            end
            lowSpeed = lowSpeedLimit + max(1e-3, 1e-6 * lowSpeedLimit);
        end

        function peakMu = getCachedPeakMu(obj, Fz, gamma, P, params, longitudinalSpeed)
            if nargin < 6 || isempty(longitudinalSpeed)
                longitudinalSpeed = obj.tireConstants.refVelocity;
            end
            Vx = obj.computeMFevalSpeed(longitudinalSpeed);
            FzKey = round(Fz / 10) * 10;
            gammaKey = round(gamma * 1000) / 1000;
            VxKey = round(Vx * 10) / 10;
            numericKey = obj.packPeakMuCacheKey(FzKey, gammaKey, P, VxKey);
            if isKey(obj.peakMuNumericCache, numericKey)
                peakMu = obj.peakMuNumericCache(numericKey);
                return;
            end

            peakMu = obj.computePeakMuInternal(Fz, gamma, P, params, Vx);
            obj.peakMuNumericCache(numericKey) = peakMu;
            key = sprintf('%.0f_%.3f_%.0f_%.1f', FzKey, gammaKey, P, VxKey);
            obj.peakMuCache(key) = peakMu;
        end

        function key = packPeakMuCacheKey(~, FzKey, gammaKey, P, VxKey)
            % Pack quantized cache coordinates into a collision-free double
            % for normal tire loads/cambers/speeds. This keeps cache hits out
            % of sprintf/char-map overhead while preserving readable misses.
            fzBin = round(FzKey / 10) + 50000;
            gammaBin = round(gammaKey * 1000) + 50000;
            vxBin = round(VxKey * 10);
            pressureBin = round(P / 1000);
            key = fzBin + 1e5 * gammaBin + 1e10 * vxBin + 1e13 * pressureBin;
        end
        
        function peakMu = computePeakMuInternal(obj, Fz, gamma, P, params, Vx)
            % COMPUTEPEAKMUINTERNAL Scan lateral curve to find peak mu
            %   Vectorized: builds a matrix of 50 input rows, single mfeval call
            
            alphaScan = linspace(-0.21, 0.21, 50);  % ±12 deg in rad
            if nargin < 6 || isempty(Vx)
                Vx = obj.tireConstants.refVelocity;
            end
            nScan = numel(alphaScan);
            
            % Build inputs matrix: each row = [Fz, kappa, alpha, gamma, phit, Vx, P]
            inputsMF = [repmat(Fz, nScan, 1), ...    % Fz
                        zeros(nScan, 1), ...          % kappa = 0 (pure lateral)
                        alphaScan(:), ...             % alpha scan
                        repmat(gamma, nScan, 1), ...  % gamma
                        zeros(nScan, 1), ...          % phit = 0
                        repmat(Vx, nScan, 1), ...     % Vx
                        repmat(P, nScan, 1)];         % P

            % Some tire files emit repeated fit-range warnings while scanning
            % peak mu. The scan is guarded/cached, so suppress only this
            % known diagnostic to avoid console I/O dominating runtime.
            warningState = warning('off', 'all');
            cleanup = onCleanup(@() warning(warningState));
            outputs = mfeval(params, inputsMF, 111);
            peakMu = max(abs(outputs(:,2))) / Fz;
        end

    end
end
