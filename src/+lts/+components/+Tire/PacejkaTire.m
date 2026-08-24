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
    %   tire.updateCorner(tire.FL, Fz, alpha, kappa, gamma)
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

        % Contact-patch load-response length [m]. First-order lag on the
        % normal load the Magic Formula sees (sigma_Fz/V_eff time
        % constant), with identical steady-state forces (no static
        % distortion). Default 0 = off (instantaneous Fz).
        % R25 uses 0.255 m, which serves two regimes:
        %   1. At speed: patch-transport-scale load response; it also broke
        %      the algebraic Fx -> ax -> attitude -> Fz -> Cx/mu*Fz -> Fx
        %      loop that sustained a nonphysical ~10-15 Hz pitch/load
        %      oscillation under heavy longitudinal loading. The one-step
        %      attitude/load stagger that drove that loop is now removed by
        %      the Simulator attitude predictor
        %      (Simulator.useAttitudePredictor; see
        %      SuspensionManager.computeCornerLoadsFromChassis), which
        %      measurably reduces entry transients on top of this filter.
        %   2. Near standstill: the V_eff floor (1 m/s) makes the lag a
        %      ~sigma_Fz-second time constant that damps the launch
        %      wheel-slip/load loop; the predictor cannot cover this
        %      regime, and launch sensitivity shows traction degrades
        %      below ~0.10 m (scripts/dbg_laptime.m).
        % See scripts/audit_stagger_validation.m for the at-speed A/B/C
        % comparison (predictor on/off x this filter on/off).
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

        % Peak-mu scan cache keyed by quantized load/camber/speed. Key
        % format owned by packPeakMuCacheKey; TireState keeps a per-corner
        % last-hit shortcut for the hot loop.
        peakMuNumericCache

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
            %                 against the repository's data/tires/ folder.
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

        function updateCorner(obj, cornerState, normalLoad, slipAngle, ...
                slipRatio, camberAngle, dt, longSpeed, computePeakMu, ...
                relaxationMode)
            % UPDATECORNER Evaluate MFeval for one corner and update its state
            %   updateCorner(cornerState, normalLoad, slipAngle, slipRatio, camberAngle)
            %   updateCorner(cornerState, normalLoad, slipAngle, slipRatio, ...
            %       camberAngle, dt, longSpeed, computePeakMu, relaxationMode)
            %
            %   cornerState   — TireState handle for this corner
            %   normalLoad    — Tire normal force Fz [N]
            %   slipAngle     — Steady-state (kinematic) slip angle alpha [rad]
            %   slipRatio     — Steady-state (kinematic) slip ratio kappa [-1 to KPUMAX]
            %   camberAngle   — Inclination angle gamma [rad], positive = top outward
            %   dt            — Timestep [s] (optional; 0 = steady-state evaluation)
            %   longSpeed     — Contact-patch longitudinal speed [m/s]
            %   computePeakMu — Whether to refresh peakMu (default true)
            %   relaxationMode — 'advance' | 'preview' | 'steady' | 'hold'
            %
            %   Mutates cornerState in-place with computed forces and moments.

            if nargin < 7 || isempty(dt)
                dt = 0;
            end
            if nargin < 8 || isempty(longSpeed)
                longSpeed = obj.tireConstants.refVelocity;
            end
            if nargin < 9 || isempty(computePeakMu)
                computePeakMu = true;
            end
            if nargin < 10 || isempty(relaxationMode)
                relaxationMode = '';
            end
            relaxationMode = obj.resolveRelaxationMode(dt, relaxationMode);

            % Store inputs
            cornerState.normalForce = normalLoad;
            ssAlpha = lts.util.clamp(slipAngle, -0.3, 0.3);   % steady-state (kinematic) target
            ssKappa = obj.clampSlipRatio(slipRatio);

            % Apply first-order contact-patch relaxation to obtain the
            % transient (force-producing) slip. With relaxationLength = 0
            % the transient slip equals the steady-state slip (baseline).
            [alpha, kappa] = obj.applyRelaxationVector(ssAlpha, ssKappa, ...
                cornerState.slipAngle, cornerState.slipRatio, longSpeed, ...
                dt, relaxationMode);
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

            isRight = obj.isRightCorner(cornerState);
            [Fx, Fy, Mx, My, Mz] = obj.evaluateMFevalRows( ...
                FzEval, kappa, obj.evaluationSlipAngle(alpha), ...
                camberAngle, longSpeed, isRight);

            if computePeakMu
                cornerState.peakMu = obj.getCachedPeakMu(FzEval, ...
                    camberAngle, obj.tireConstants.nomPressure, ...
                    obj.tireConstants.params, longSpeed);
            end
            cornerState.Fx = Fx;
            cornerState.Fy = Fy;
            cornerState.Mx = Mx;
            cornerState.My = My;
            cornerState.Mz = Mz;
        end

        %% ---- TireModel interface methods ----

        function Fy = computeLateralForce(obj, normalLoad, slipAngle, ~)
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

        function Fx = computeLongitudinalForce(obj, normalLoad, slipRatio, ~)
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

        %% ---- Slip ratio entry point ----

        function kappa = computeSlipRatioFromKinematics(obj, cornerState, longitudinalSpeed)
            % COMPUTESLIPRATIOFROMKINEMATICS MFeval-consistent local-wheel slip.
            % Shared simulator/contact-solver entry point; preserves the
            % signed wheel-frame road speed (reverse-capable).
            kappa = obj.computeKinematicSlipRatio( ...
                cornerState.angularVelocity * cornerState.wheelRadius, ...
                longitudinalSpeed);
        end

        %% ---- Wheel rotational dynamics ----

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
            %   Uses explicit Euler integration; stability at low speed rests
            %   on the Simulator's outer fixed-point wheel solve.
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

            netTorque = obj.computeWheelNetTorque(cornerState, ...
                driveTorque, brakeTorque, longitudinalSpeed);

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

        %% ---- All-corners batch update ----

        function updateAllCorners(obj, Fz_FL, Fz_FR, Fz_RL, Fz_RR, ...
                slipAngle_FL, slipAngle_FR, slipAngle_RL, slipAngle_RR, ...
                kappa_FL, kappa_FR, kappa_RL, kappa_RR, ...
                camber_FL, camber_FR, camber_RL, camber_RR, dt, longSpeeds, ...
                computePeakMu, relaxationMode)
            % UPDATEALLCORNERS Evaluate and update all four tire states.
            %   Optional trailing args: dt (0 = steady state), longSpeeds
            %   (4-vector), computePeakMu (logical), relaxationMode.

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
            if nargin < 20 || isempty(computePeakMu)
                computePeakMu = true;
            end
            if nargin < 21 || isempty(relaxationMode)
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
                    peakFzKey, peakGammaKey, obj.tireConstants.nomPressure, peakVxKey);
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
            [alpha, kappa] = obj.applyRelaxationVector(ssAlpha, ssKappa, ...
                previousAlpha, previousKappa, longSpeed, dt, relaxationMode);
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
                            FzEval(i), gamma(i), obj.tireConstants.nomPressure, ...
                            obj.tireConstants.params, peakVx(i));
                        obj.peakMuNumericCache(numericKey) = rawPeakMu;
                        states{i}.peakMuCacheKey = numericKey;
                        states{i}.rawPeakMu = rawPeakMu;
                    end
                    states{i}.peakMu = rawPeakMu;
                end
            end
            if any(active)
                isRight = [false; true; false; true];
                alphaEval = obj.evaluationSlipAnglesByCorner(alpha);
                [Fx, Fy, Mx, My, Mz] = obj.evaluateMFevalRows( ...
                    FzEval(active), kappa(active), alphaEval(active), ...
                    gamma(active), longSpeed(active), isRight(active));
                for j = 1:numel(activeIdx)
                    i = activeIdx(j);
                    states{i}.Fx = Fx(j);
                    states{i}.Fy = Fy(j);
                    states{i}.Mx = Mx(j);
                    states{i}.My = My(j);
                    states{i}.Mz = Mz(j);
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
    end

    methods (Access = private)
        function [alpha, kappa] = applyRelaxationVector(obj, ...
                ssAlpha, ssKappa, previousAlpha, previousKappa, ...
                longSpeed, dt, relaxationMode)
            % APPLYRELAXATIONVECTOR First-order contact-patch slip lag
            %   sigma * d(alpha)/dt + V * alpha = V * alpha_ss
            % Solved with the exact, unconditionally-stable exponential form
            %   alpha = alpha_ss - (alpha_ss - alpha_prev) * exp(-V_eff*dt/sigma)
            % which is stable for any dt (explicit Euler would be stiff here).
            % Vectorized over corners; updateCorner passes scalars. Each
            % state has its own physical relaxation length. NaN for the
            % longitudinal value retains the legacy shared-length behavior.
            sigmaAlpha = obj.relaxationLength;
            sigmaKappa = obj.resolvedLongitudinalRelaxationLength();
            % A stopped contact patch has no kinematic longitudinal slip.
            % Never retain a previous braking force after both wheel and
            % road speed have reached zero.
            rest = abs(longSpeed(:)) <= 1e-9 & abs(ssKappa(:)) <= 1e-12;

            if strcmp(relaxationMode, 'hold')
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
                % Effective rolling speed for the lag time constant. A floor
                % keeps the transient model well-conditioned near standstill.
                V_eff = max(abs(longSpeed(:)), 1.0);
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
            kappa(rest) = 0;
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
            % applyRelaxationVector's advance/preview/hold/steady semantics:
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

        function netTorque = computeWheelNetTorque(obj, cornerState, ...
                driveTorque, brakeTorque, longitudinalSpeed)
            % Net wheel torque about the spin axis: drive accelerates,
            % brake and tire Fx decelerate (Fx > 0 = driving force, so its
            % reaction torque opposes wheel spin), plus rolling-resistance
            % and bearing-drag torque opposing spin. Kept separate from the
            % brake path so resistance magnitude does not depend on brakes.
            omega = cornerState.angularVelocity;
            radius = cornerState.wheelRadius;
            brakeSign = obj.computeBrakeTorqueSign( ...
                omega, longitudinalSpeed, driveTorque);
            spinSign = sign(omega);
            if spinSign == 0
                spinSign = brakeSign;  % use the brake/roll direction when omega ~ 0
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

        function [Fx, Fy, Mx, My, Mz] = evaluateMFevalRows(obj, ...
                Fz, kappa, alpha, gamma, longSpeed, isRight)
            % EVALUATEMFEVALROWS MFeval combined-slip evaluation with the
            % corner-mirroring conventions applied. Single source of truth
            % for how a right-side corner maps onto the bundled left-side
            % tire data.
            %
            %   Inputs (column vectors, one row per corner):
            %     Fz        evaluation normal load [N] (already load-relaxed)
            %     kappa     transient slip ratio (already clamped)
            %     alpha     evaluation slip angle (already stiffness-scaled)
            %     gamma     inclination angle, positive = top tilted outward
            %     longSpeed contact-patch longitudinal speed [m/s]
            %     isRight   logical, true for right-side corners
            %
            %   Mirror map (verified empirically, scripts/audit_camber_probe.m):
            %   the .tir data describes one tire; a right corner is its
            %   mirror image about the car centerline (y -> -y), under which
            %   alpha, Fy, Mx, and Mz are odd while kappa, Fx, My, and gamma
            %   are even. Gamma is even because the corner-local convention
            %   "positive = top outward" maps outward to outward under the
            %   mirror — gamma must NOT be negated for right corners. Equal
            %   outward-positive camber on both sides therefore produces
            %   equal and opposite thrusts (thrust toward the lean on each
            %   corner; net zero for a symmetric setup), as measured.
            %
            %   MFeval's Fy/Mz sign convention is opposite the simulator's,
            %   hence the left-corner negation of those outputs.
            n = numel(Fz);
            alphaEval = alpha(:);
            alphaEval(isRight) = -alphaEval(isRight);
            Vx = obj.computeMFevalSpeed(longSpeed);
            inputsMF = [Fz(:), kappa(:), alphaEval, gamma(:), zeros(n, 1), ...
                Vx(:), repmat(obj.tireConstants.nomPressure, n, 1)];
            outputs = mfeval(obj.tireConstants.params, inputsMF, 111);

            Fx = outputs(:,1);
            My = outputs(:,5);
            Fy = -outputs(:,2);
            Mx = outputs(:,4);
            Mz = -outputs(:,6);
            Fy(isRight) = -Fy(isRight);
            Mx(isRight) = -Mx(isRight);
            Mz(isRight) = -Mz(isRight);
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
        end

        function key = packPeakMuCacheKey(~, FzKey, gammaKey, P, VxKey)
            % Pack quantized cache coordinates into a collision-free double
            % for normal tire loads/cambers/speeds.
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
