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

        % Tire relaxation length [m]. Models the first-order contact-patch
        % lag between the kinematic (steady-state) slip and the force-
        % producing (transient) slip:
        %   sigma * d(alpha)/dt + V * alpha = V * alpha_ss
        % Solved in the exact, unconditionally-stable exponential form.
        % 0 disables the transient layer (pure steady-state Magic Formula).
        relaxationLength = 0.30

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

        % Allow wheel angular velocity to go negative (true reverse rotation).
        % Default false: a one-direction clutch clamps omega >= 0, which is
        % stable for forward lap-time simulation. Set true when the powertrain
        % is regen/reverse-capable so coastdown/regen drag can fully spin a
        % wheel down. lts.vehicle.VehicleManager sets this from the powertrain.
        allowReverseRotation = false

        % Cache peak-mu scans by rounded load/camber/speed.
        peakMuCache

        % Track-surface Mu that corresponds to the raw tire file. A surface
        % Mu of 1.2 preserves the current dry-track Pacejka behavior.
        surfaceMuReference = 1.2

        % Resolved MFeval low-speed floor [m/s]. The .tir params struct is
        % immutable, so whether it carries VXLOW (and its value) is a run
        % invariant; cached to avoid an isfield check per corner per step.
        cachedMFevalLowSpeed = NaN
    end
    
    methods
        function obj = PacejkaTire(tirFilePath)
            % PACEJKATIRE Construct from a .tir file, creating 4 corner states
            %   PacejkaTire(tirFilePath)
            %
            %   tirFilePath — path to the .tir file. If relative, resolved
            %                 relative to the +Tire/ folder.
            
            % Load shared tire constants
            obj.tireConstants = lts.components.Tire.TireConstants(tirFilePath);
            obj.cachedMFevalLowSpeed = obj.resolveMFevalLowSpeed();
            
            % Create per-corner state objects
            obj.FL = lts.components.Tire.TireState();
            obj.FR = lts.components.Tire.TireState();
            obj.RL = lts.components.Tire.TireState();
            obj.RR = lts.components.Tire.TireState();
            obj.peakMuCache = containers.Map('KeyType', 'char', 'ValueType', 'double');
            
            fprintf('  PacejkaTire: 4 corner states created (FL, FR, RL, RR)\n');
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
            %   The raw Pacejka tire data is treated as the dry-reference
            %   surface and scaled by surfaceMu/surfaceMuReference.
            %   Mutates cornerState in-place with computed forces and moments.

            [surfaceMu, dt, longSpeed, computePeakMu, relaxationMode] = ...
                obj.parseCornerOptionalArgs(varargin{:});
            relaxationMode = obj.resolveRelaxationMode(dt, relaxationMode);
            
            % Store inputs
            cornerState.normalForce = normalLoad;
            ssAlpha = max(-0.3, min(0.3, slipAngle));   % steady-state (kinematic) target
            ssKappa = max(-1, min(1, slipRatio));

            % Apply first-order contact-patch relaxation to obtain the
            % transient (force-producing) slip. With relaxationLength = 0
            % the transient slip equals the steady-state slip (baseline).
            [alpha, kappa] = obj.applyRelaxation( ...
                cornerState, ssAlpha, ssKappa, longSpeed, dt, relaxationMode);

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

            if normalLoad <= 0
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
            Fz    = normalLoad;
            gamma = camberAngle;
            V     = obj.computeMFevalSpeed(longSpeed);
            P     = obj.tireConstants.nomPressure;
            params = obj.tireConstants.params;

            % Build MFeval inputs row: [Fz, kappa, alpha, gamma, phit, Vx, P]
            inputsMF = [Fz, kappa, alpha, gamma, 0, V, P];

            % Evaluate Pacejka Magic Formula via MFeval (useMode=111: combined)
            outputs = mfeval(params, inputsMF, 111);

            surfaceScale = obj.computeSurfaceScale(surfaceMu);
            if computePeakMu
                rawPeakMu = obj.getCachedPeakMu(Fz, gamma, P, params, longSpeed);
                cornerState.peakMu = rawPeakMu * surfaceScale;
            end

            % Store the Magic-Formula forces scaled from the dry reference
            % surface to the local track Mu.
            cornerState.Fy = -outputs(:,2) * surfaceScale;
            cornerState.Fx = outputs(:,1) * surfaceScale;
            cornerState.Mx = outputs(:,4);
            cornerState.My = outputs(:,5);
            cornerState.Mz = outputs(:,6) * surfaceScale;
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
            if nargin < 4 || isempty(mu)
                mu = obj.surfaceMuReference;
            end
            
            inputsMF = [normalLoad, 0, slipAngle, 0, 0, ...
                obj.tireConstants.refVelocity, obj.tireConstants.nomPressure];
            outputs = mfeval(obj.tireConstants.params, inputsMF, 111);
            
            Fy = -outputs(:,2) * obj.computeSurfaceScale(mu);
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
            if nargin < 4 || isempty(mu)
                mu = obj.surfaceMuReference;
            end
            
            inputsMF = [normalLoad, slipRatio, 0, 0, 0, ...
                obj.tireConstants.refVelocity, obj.tireConstants.nomPressure];
            outputs = mfeval(obj.tireConstants.params, inputsMF, 111);
            
            Fx = outputs(:,1) * obj.computeSurfaceScale(mu);
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
            
            omega = cornerState.angularVelocity;
            R     = cornerState.wheelRadius;
            V     = max(vehicleSpeed, 0);   % no reverse
            
            wheelSpeed = omega * R;
            denom = max(abs(wheelSpeed), abs(V));

            slipSpeedFloor = 1.0;
            rawKappa = (wheelSpeed - V) / max(denom, slipSpeedFloor);
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
            
            % Clamp to [-1, 1]
            kappa = max(-1, min(1, kappa));
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
            %                   Defaults to obj.wheelInertia. The lts.simulation.Simulator
            %                   passes wheel+tire+reflected-rotor inertia on
            %                   the driven axle so the motor mass is felt at
            %                   the contact patch.

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

            % Prevent wheel from spinning backwards (one-direction clutch),
            % unless reverse rotation is explicitly enabled (regen/reverse-
            % capable powertrain). The clamp keeps forward lap-time sim stable;
            % when lifted, sign(omega) terms above handle negative rotation.
            if ~obj.allowReverseRotation && omega_new < 0
                omega_new = 0;
            end

            cornerState.angularVelocity = omega_new;
        end

        function solveWheelContact(obj, cornerState, normalLoad, slipAngle, ...
                camberAngle, mu, longitudinalSpeed, driveTorque, brakeTorque, dt)
            % SOLVEWHEELCONTACT Semi-implicitly couple wheel speed and tire Fx.
            %   I*domega/dt = T_drive - T_brake - Fx(kappa(omega))*R

            if nargin < 10 || isempty(dt)
                dt = 0.001;
            end

            omegaOld = max(cornerState.angularVelocity, 0);
            omegaNew = omegaOld;
            R = max(cornerState.wheelRadius, eps);
            I = max(obj.wheelInertia, eps);
            dt = max(dt, 0);
            slipAngle = max(-0.3, min(0.3, slipAngle));

            finalFx = 0;
            finalFy = 0;
            finalMx = 0;
            finalMy = 0;
            finalMz = 0;
            finalPeakMu = 0;
            finalKappa = cornerState.slipRatio;

            for iter = 1:5 %#ok<NASGU>
                finalKappa = obj.computeSlipRatioFromOmega( ...
                    cornerState, omegaNew, longitudinalSpeed);
                [finalFx, finalFy, finalMx, finalMy, finalMz, finalPeakMu] = ...
                    obj.evaluateForces(normalLoad, slipAngle, finalKappa, ...
                    camberAngle, mu, longitudinalSpeed, false);

                brakeSign = obj.computeBrakeTorqueSign( ...
                    omegaNew, longitudinalSpeed, driveTorque);
                netTorque = driveTorque - brakeSign * brakeTorque - finalFx * R;
                omegaCandidate = max(0, omegaOld + (netTorque / I) * dt);

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
                camberAngle, mu, longitudinalSpeed, true);

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
            %   The raw Pacejka tire data is treated as the dry-reference
            %   surface and scaled by surfaceMu/surfaceMuReference.

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
            ssAlpha = max(-0.3, min(0.3, ...
                [slipAngle_FL; slipAngle_FR; slipAngle_RL; slipAngle_RR]));
            ssKappa = max(-1, min(1, [kappa_FL; kappa_FR; kappa_RL; kappa_RR]));
            gamma = [camber_FL; camber_FR; camber_RL; camber_RR];
            longSpeed = longSpeeds(:);
            surfaceScale = obj.expandSurfaceScale(surfaceMu, 4);
            states = {obj.FL, obj.FR, obj.RL, obj.RR};

            % Apply per-corner relaxation to obtain the transient (force-
            % producing) slip. The lagged slip stored on each corner only
            % advances when dt > 0 (i.e. the final call of a step); during
            % intermediate wheel-solve iterations (dt = 0) the force is
            % evaluated at the steady-state kinematic slip while the lagged
            % state is preserved for the next physics step.
            alpha = zeros(4, 1);
            kappa = zeros(4, 1);
            for i = 1:4
                states{i}.normalForce = Fz(i);
                states{i}.ssSlipAngle = ssAlpha(i);
                states{i}.ssSlipRatio = ssKappa(i);
                states{i}.camberAngle = gamma(i);
                [alpha(i), kappa(i)] = obj.applyRelaxation( ...
                    states{i}, ssAlpha(i), ssKappa(i), longSpeeds(i), dt, ...
                    relaxationMode);
                if strcmp(relaxationMode, 'advance') && dt > 0
                    % Commit the advanced lagged state for next step.
                    states{i}.slipAngle = alpha(i);
                    states{i}.slipRatio = kappa(i);
                end
            end

            active = Fz > 0;
            if any(active)
                P = obj.tireConstants.nomPressure;
                params = obj.tireConstants.params;
                nActive = nnz(active);
                Vx = obj.computeMFevalSpeed(longSpeed(active));
                inputsMF = [Fz(active), kappa(active), alpha(active), ...
                    gamma(active), zeros(nActive, 1), ...
                    Vx, repmat(P, nActive, 1)];
                outputs = mfeval(params, inputsMF, 111);

                activeIdx = find(active);
                for j = 1:numel(activeIdx)
                    i = activeIdx(j);
                    if computePeakMu
                        rawPeakMu = obj.getCachedPeakMu( ...
                            Fz(i), gamma(i), P, params, longSpeed(i));
                        states{i}.peakMu = rawPeakMu * surfaceScale(i);
                    end
                    states{i}.Fx = outputs(j,1) * surfaceScale(i);
                    states{i}.Fy = -outputs(j,2) * surfaceScale(i);
                    states{i}.Mx = outputs(j,4);
                    states{i}.My = outputs(j,5);
                    states{i}.Mz = outputs(j,6) * surfaceScale(i);
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
            % With relaxationLength = 0 the transient slip equals ss (baseline).
            sigma = obj.relaxationLength;
            if nargin < 7 || isempty(relaxationMode)
                relaxationMode = obj.resolveRelaxationMode(dt, '');
            end
            if sigma <= 0 || dt <= 0
                if strcmp(relaxationMode, 'hold') && sigma > 0
                    alpha = cornerState.slipAngle;
                    kappa = cornerState.slipRatio;
                    return;
                end
                alpha = ssAlpha;
                kappa = ssKappa;
                return;
            end
            if strcmp(relaxationMode, 'steady')
                alpha = ssAlpha;
                kappa = ssKappa;
                return;
            elseif strcmp(relaxationMode, 'hold')
                alpha = cornerState.slipAngle;
                kappa = cornerState.slipRatio;
                return;
            end

            % Effective rolling speed for the lag time constant. A floor
            % matches the slip-ratio low-speed blend so the lag does not
            % blow up at standstill (at low speed the patch builds force
            % slowly, which is physical).
            V_eff = max(abs(longSpeed), 1.0);
            decay = exp(-V_eff * dt / sigma);

            alpha = ssAlpha - (ssAlpha - cornerState.slipAngle) * decay;
            kappa = ssKappa - (ssKappa - cornerState.slipRatio) * decay;

            alpha = max(-0.3, min(0.3, alpha));
            kappa = max(-1, min(1, kappa));
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

        function kappa = computeSlipRatioFromOmega(~, cornerState, omega, longitudinalSpeed)
            % Same slip convention as lts.simulation.Simulator.computeLocalSlipRatio, used by
            % the older single-corner contact solver. The low-speed blend
            % keeps launch/brake-to-zero behavior continuous.
            wheelSpeed = omega * cornerState.wheelRadius;
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
                Fz, alpha, kappa, gamma, surfaceMu, longitudinalSpeed, computePeakMu)
            % EVALUATEFORCES Thin wrapper around MFeval's combined-slip mode.
            % Inputs follow MFeval's [Fz, kappa, alpha, gamma, phit, Vx, P]
            % order. The raw .tir file is treated as the dry-reference
            % surface; surfaceMu scales forces linearly for alternate surfaces.
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

            alpha = max(-0.3, min(0.3, alpha));
            kappa = max(-1, min(1, kappa));
            P = obj.tireConstants.nomPressure;
            params = obj.tireConstants.params;
            Vx = obj.computeMFevalSpeed(longitudinalSpeed);
            inputsMF = [Fz, kappa, alpha, gamma, 0, Vx, P];
            outputs = mfeval(params, inputsMF, 111);

            surfaceScale = obj.computeSurfaceScale(surfaceMu);
            if computePeakMu
                rawPeakMu = obj.getCachedPeakMu(Fz, gamma, P, params, longitudinalSpeed);
                peakMu = rawPeakMu * surfaceScale;
            else
                peakMu = 0;
            end
            Fx = outputs(:,1) * surfaceScale;
            Fy = -outputs(:,2) * surfaceScale;
            Mx = outputs(:,4);
            My = outputs(:,5);
            Mz = outputs(:,6) * surfaceScale;
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
            validModes = {'advance', 'steady', 'hold'};
            if ~any(strcmp(mode, validModes))
                error('PacejkaTire:InvalidRelaxationMode', ...
                    'relaxationMode must be advance, steady, or hold.');
            end
        end

        function scale = computeSurfaceScale(obj, surfaceMu)
            % Track mu is a multiplier relative to the tire file's reference
            % surface, not an extra friction cap. This keeps the Pacejka load
            % sensitivity intact while allowing lower-grip tracks.
            if nargin < 2 || isempty(surfaceMu) || ~isfinite(surfaceMu)
                surfaceMu = obj.surfaceMuReference;
            end
            scale = max(surfaceMu, 0) / max(obj.surfaceMuReference, eps);
        end

        function scale = expandSurfaceScale(obj, surfaceMu, n)
            if nargin < 3
                n = 1;
            end
            if isempty(surfaceMu)
                surfaceMu = obj.surfaceMuReference;
            end
            if isscalar(surfaceMu)
                surfaceMu = repmat(surfaceMu, n, 1);
            else
                surfaceMu = surfaceMu(:);
                if numel(surfaceMu) ~= n
                    error('PacejkaTire:InvalidSurfaceMu', ...
                        'surfaceMu must be scalar or have one value per corner.');
                end
            end
            surfaceMu(~isfinite(surfaceMu)) = obj.surfaceMuReference;
            scale = max(surfaceMu, 0) ./ max(obj.surfaceMuReference, eps);
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
            key = sprintf('%.0f_%.3f_%.0f_%.1f', FzKey, gammaKey, P, VxKey);
            if isKey(obj.peakMuCache, key)
                peakMu = obj.peakMuCache(key);
                return;
            end

            peakMu = obj.computePeakMuInternal(Fz, gamma, P, params, Vx);
            obj.peakMuCache(key) = peakMu;
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
            
            outputs = mfeval(params, inputsMF, 111);
            peakMu = max(abs(outputs(:,2))) / Fz;
        end
    end
end
