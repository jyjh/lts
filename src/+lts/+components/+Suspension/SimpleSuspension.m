classdef SimpleSuspension
    % SIMPLESUSPENSION Per-corner quarter-car suspension model
    % Models each corner independently with:
    %   - Sprung mass vertical motion
    %   - Unsprung mass vertical motion
    %   - Suspension spring/damper/bump stop between the masses
    %   - Tire spring to flat road
    %
    % Vehicle-level geometry (trackWidth, wheelbase, cgHeight)
    % is retrieved from lts.vehicle.VehicleManager at construction time.
    %
    % This is one suspension unit for a SINGLE corner. The SuspensionManager
    % creates four instances (FL, FR, RL, RR), where front corners share
    % parameters and rear corners share parameters.
    %
    % Transient state is stored in a SuspensionState object that persists
    % across timesteps and is mutated in-place.

    properties
        % --- Vehicle geometry (from lts.vehicle.VehicleManager, stored at construction) ---
        trackWidth                   % Track width [m]
        wheelbase                    % Wheelbase [m]
        cgHeight                     % Center of gravity height [m]

        % --- Per-corner spring-damper ---
        springRate                   % Heave spring rate [N/m]
        dampingCoeff                 % Low-speed compression slope [N*s/m]
        reboundCoeff                 % Low-speed rebound slope [N*s/m]
        % Digressive-damper knee. Wheel-domain velocity [m/s] up to which the
        % low-speed slope holds; above it the slope drops to
        % dampingHighSpeedRatio x the low-speed slope. Inf => linear damper.
        dampingKneeSpeed = Inf
        % High-speed damper slope as a fraction of the low-speed slope [-].
        % 1.0 => linear (no digression). Typical racing dampers ~0.2-0.3.
        dampingHighSpeedRatio = 1.0
        motionRatio                  % Damper travel / wheel travel [-]
        bumpStopLength               % Bump stop engagement length [m]
        bumpStopRate                 % Bump stop stiffness [N/m]

        % --- Tire spring ---
        tireSpringRate               % Vertical tire stiffness [N/m]

        % --- Corner masses ---
        sprungMass                   % Per-corner sprung mass [kg]
        unsprungMass                 % Per-corner unsprung mass [kg]

        % --- Transient state ---
        state                        % SuspensionState handle object

        % Internal integration cap for stiff tire/suspension vertical modes.
        maxIntegrationStep = 0.001
    end

    methods
        function obj = SimpleSuspension(vehicleManager, ...
                springRate, dampingCoeff, reboundCoeff, ...
                motionRatio, bumpStopLength, bumpStopRate, ...
                tireSpringRate, unsprungMass, sprungMass, ...
                dampingKneeSpeed, dampingHighSpeedRatio)
            % SIMPLESUSPENSION Construct a per-corner suspension unit
            %   SimpleSuspension(vehicleManager, ...
            %       springRate, dampingCoeff, reboundCoeff, ...
            %       motionRatio, bumpStopLength, bumpStopRate, ...
            %       tireSpringRate, unsprungMass, sprungMass)
            %   SimpleSuspension(..., dampingKneeSpeed, dampingHighSpeedRatio)
            %
            %   vehicleManager  - lts.vehicle.VehicleManager handle (geometry pulled at construction)
            %   springRate      - Heave spring rate [N/m]
            %   dampingCoeff    - Low-speed compression slope [N*s/m]
            %   reboundCoeff    - Low-speed rebound slope [N*s/m]
            %   motionRatio     - Damper travel / wheel travel
            %   bumpStopLength  - Bump stop travel before engagement [m]
            %   bumpStopRate    - Bump stop stiffness [N/m]
            %   tireSpringRate  - Vertical tire stiffness [N/m]
            %   unsprungMass    - Per-corner unsprung mass [kg]
            %   sprungMass      - Per-corner sprung mass [kg]
            %   dampingKneeSpeed      - Optional wheel-domain velocity [m/s] at which
            %                           the damper slope breaks to high-speed. Inf
            %                           (default) keeps a linear damper.
            %   dampingHighSpeedRatio - Optional high-speed slope / low-speed slope.
            %                           1.0 (default) keeps a linear damper.

            % Pull vehicle-level geometry from lts.vehicle.VehicleManager
            obj.trackWidth   = vehicleManager.trackWidth;
            obj.wheelbase    = vehicleManager.wheelbase;
            obj.cgHeight     = vehicleManager.cgHeight;

            % Store suspension-specific parameters
            obj.springRate        = springRate;
            obj.dampingCoeff      = dampingCoeff;
            obj.reboundCoeff      = reboundCoeff;
            obj.motionRatio       = motionRatio;
            obj.bumpStopLength    = bumpStopLength;
            obj.bumpStopRate      = bumpStopRate;
            obj.tireSpringRate    = tireSpringRate;
            obj.unsprungMass      = unsprungMass;
            if nargin >= 11 && ~isempty(dampingKneeSpeed) && isnumeric(dampingKneeSpeed) ...
                    && (isinf(dampingKneeSpeed) || (isscalar(dampingKneeSpeed) && ...
                        isfinite(dampingKneeSpeed) && dampingKneeSpeed > 0))
                obj.dampingKneeSpeed = dampingKneeSpeed;
            end
            if nargin >= 12 && ~isempty(dampingHighSpeedRatio) && isnumeric(dampingHighSpeedRatio) ...
                    && isscalar(dampingHighSpeedRatio) && isfinite(dampingHighSpeedRatio) ...
                    && dampingHighSpeedRatio > 0
                obj.dampingHighSpeedRatio = dampingHighSpeedRatio;
            end
            if nargin >= 10 && ~isempty(sprungMass)
                obj.sprungMass = sprungMass;
            else
                obj.sprungMass = max(vehicleManager.totalMass / 4 - unsprungMass, eps);
            end

            % Initialize transient state
            obj.state = lts.components.Suspension.SuspensionState();
            obj.state.motionRatioEffective = obj.motionRatio;
        end

        function initializeStaticLoad(obj, cornerState, staticLoad)
            % INITIALIZESTATICLOAD Set deterministic static equilibrium.
            % Dynamic displacement states are measured from this equilibrium.
            staticLoad = max(staticLoad, 0);
            K_eff = obj.springRate * obj.getEffectiveMotionRatio(cornerState)^2;

            cornerState.staticLoad = staticLoad;
            cornerState.staticSuspensionCompression = ...
                obj.computeStaticSuspensionCompression(staticLoad, K_eff);
            cornerState.staticTireDeflection = staticLoad / max(obj.tireSpringRate, eps);

            cornerState.sprungPosition = 0;
            cornerState.sprungVelocity = 0;
            cornerState.unsprungPosition = 0;
            cornerState.unsprungVelocity = 0;

            cornerState.damperPosition = 0;
            cornerState.damperVelocity = 0;
            cornerState.tireDeflection = cornerState.staticTireDeflection;
            cornerState.tireNormalForce = staticLoad;
            cornerState.suspensionForce = staticLoad;
            cornerState.demandedLoad = staticLoad;
        end

        function updateCornerFromChassis(obj, cornerState, sprungPosition, ...
                sprungVelocity, dt, antiRollBarForce)
            % UPDATECORNERFROMCHASSIS Update unsprung/tire load from chassis motion.
            % Sprung motion is imposed by the chassis heave/pitch/roll model;
            % this corner only advances the unsprung mass against suspension
            % and tire springs (the chassis has already solved the sprung DOF).

            if nargin < 6 || isempty(antiRollBarForce)
                antiRollBarForce = 0;
            end
            if ~isfinite(antiRollBarForce)
                antiRollBarForce = 0;
            end

            if cornerState.staticLoad <= 0 && cornerState.tireNormalForce <= 0
                obj.initializeStaticLoad(cornerState, 0);
            end
            cornerState.antiRollBarForce = antiRollBarForce;

            nSubsteps = obj.integrationSubsteps(dt);
            if nSubsteps > 1
                subDt = dt / nSubsteps;
                startSprungPosition = cornerState.sprungPosition;
                startSprungVelocity = cornerState.sprungVelocity;
                for idx = 1:nSubsteps
                    blend = idx / nSubsteps;
                    subSprungPosition = startSprungPosition + ...
                        blend * (sprungPosition - startSprungPosition);
                    subSprungVelocity = startSprungVelocity + ...
                        blend * (sprungVelocity - startSprungVelocity);
                    obj.updateCornerFromChassis( ...
                        cornerState, subSprungPosition, subSprungVelocity, ...
                        subDt, antiRollBarForce);
                end
                return;
            end

            z_u_prev = cornerState.unsprungPosition;
            v_u_prev = cornerState.unsprungVelocity;

            MR_eff = obj.getEffectiveMotionRatio(cornerState);
            K_eff = obj.springRate * MR_eff^2;

            suspensionDeflection = sprungPosition - z_u_prev;
            suspensionVelocity = sprungVelocity - v_u_prev;
            [F_suspension, ~, ~, ~] = obj.computeSuspensionForce( ...
                cornerState, suspensionDeflection, suspensionVelocity, K_eff, MR_eff);
            F_suspension = F_suspension + antiRollBarForce;
            F_tire = obj.computeTireNormalForce(cornerState, z_u_prev);

            z_u_ddot = (F_suspension - F_tire) / max(obj.unsprungMass, eps);
            v_u_new = v_u_prev + z_u_ddot * dt;
            z_u_new = z_u_prev + v_u_new * dt;

            suspensionDeflection = sprungPosition - z_u_new;
            suspensionVelocity = sprungVelocity - v_u_new;
            [F_suspension, ~, ~, ~] = obj.computeSuspensionForce( ...
                cornerState, suspensionDeflection, suspensionVelocity, K_eff, MR_eff);
            F_suspension = F_suspension + antiRollBarForce;
            F_tire = obj.computeTireNormalForce(cornerState, z_u_new);

            cornerState.sprungPosition = sprungPosition;
            cornerState.sprungVelocity = sprungVelocity;
            cornerState.unsprungPosition = z_u_new;
            cornerState.unsprungVelocity = v_u_new;
            cornerState.damperPosition = suspensionDeflection;
            cornerState.damperVelocity = suspensionVelocity;
            cornerState.tireDeflection = max( ...
                cornerState.staticTireDeflection + z_u_new, 0);
            cornerState.tireNormalForce = F_tire;
            cornerState.suspensionForce = F_suspension;
            cornerState.demandedLoad = F_suspension;
        end

        function initializeCornerFromChassis(obj, cornerState, sprungPosition, ...
                sprungVelocity, antiRollBarForce)
            % INITIALIZECORNERFROMCHASSIS Set the unsprung state to static
            % equilibrium for an imposed chassis position. This is used by
            % correlation warm starts, where integrating from the static-ride
            % wheel position would inject an avoidable wheel-hop transient.
            if nargin < 5 || isempty(antiRollBarForce) || ...
                    ~isfinite(antiRollBarForce)
                antiRollBarForce = 0;
            end
            if nargin < 4 || isempty(sprungVelocity) || ...
                    ~isfinite(sprungVelocity)
                sprungVelocity = 0;
            end

            MR_eff = obj.getEffectiveMotionRatio(cornerState);
            K_eff = obj.springRate * MR_eff^2;
            lower = -0.25;
            upper = 0.25;
            fLower = obj.chassisEquilibriumResidual( ...
                cornerState, sprungPosition, sprungVelocity, lower, ...
                antiRollBarForce, K_eff, MR_eff);
            fUpper = obj.chassisEquilibriumResidual( ...
                cornerState, sprungPosition, sprungVelocity, upper, ...
                antiRollBarForce, K_eff, MR_eff);

            if fLower * fUpper <= 0
                for idx = 1:60
                    mid = 0.5 * (lower + upper);
                    fMid = obj.chassisEquilibriumResidual( ...
                        cornerState, sprungPosition, sprungVelocity, mid, ...
                        antiRollBarForce, K_eff, MR_eff);
                    if fLower * fMid <= 0
                        upper = mid;
                        fUpper = fMid; %#ok<NASGU>
                    else
                        lower = mid;
                        fLower = fMid;
                    end
                end
                unsprungPosition = 0.5 * (lower + upper);
            else
                % The normal operating range is bracketed above. Retain a
                % finite, bounded state for pathological configurations.
                if abs(fLower) < abs(fUpper)
                    unsprungPosition = lower;
                else
                    unsprungPosition = upper;
                end
            end

            suspensionDeflection = sprungPosition - unsprungPosition;
            [F_suspension, ~, ~, ~] = obj.computeSuspensionForce( ...
                cornerState, suspensionDeflection, sprungVelocity, K_eff, MR_eff);
            F_suspension = F_suspension + antiRollBarForce;
            F_tire = obj.computeTireNormalForce(cornerState, unsprungPosition);

            cornerState.sprungPosition = sprungPosition;
            cornerState.sprungVelocity = sprungVelocity;
            cornerState.unsprungPosition = unsprungPosition;
            cornerState.unsprungVelocity = 0;
            cornerState.damperPosition = suspensionDeflection;
            cornerState.damperVelocity = sprungVelocity;
            cornerState.tireDeflection = max( ...
                cornerState.staticTireDeflection + unsprungPosition, 0);
            cornerState.tireNormalForce = F_tire;
            cornerState.suspensionForce = F_suspension;
            cornerState.antiRollBarForce = antiRollBarForce;
            cornerState.demandedLoad = F_suspension;
        end

        function wheelRate = getEffectiveWheelRate(obj, cornerState)
            % GETEFFECTIVEWHEELRATE Small-signal wheel rate about static ride.
            % Includes bump-stop tangent stiffness when the current dynamic
            % wheel-domain travel has reached the configured stop.
            % damperPosition and bumpStopLength are both measured from
            % static ride height in that same wheel domain.
            if nargin < 2 || isempty(cornerState)
                cornerState = obj.state;
            end

            MR_eff = obj.getEffectiveMotionRatio(cornerState);
            wheelRate = obj.springRate * MR_eff^2;

            if obj.bumpStopRate > 0 && ...
                    cornerState.damperPosition >= max(obj.bumpStopLength, 0) - 1e-12
                wheelRate = wheelRate + obj.bumpStopRate;
            end
        end
    end

    methods (Access = private)
        function residual = chassisEquilibriumResidual(obj, cornerState, ...
                sprungPosition, sprungVelocity, unsprungPosition, ...
                antiRollBarForce, K_eff, MR_eff)
            suspensionDeflection = sprungPosition - unsprungPosition;
            [F_suspension, ~, ~, ~] = obj.computeSuspensionForce( ...
                cornerState, suspensionDeflection, sprungVelocity, K_eff, MR_eff);
            F_suspension = F_suspension + antiRollBarForce;
            F_tire = obj.computeTireNormalForce(cornerState, unsprungPosition);
            residual = F_suspension - F_tire;
        end

        function MR_eff = getEffectiveMotionRatio(obj, cornerState)
            MR_eff = obj.motionRatio;
            if cornerState.motionRatioEffective > 0
                MR_eff = cornerState.motionRatioEffective;
            end
            MR_eff = max(MR_eff, eps);
        end

        function [F_suspension, F_spring, F_damper, F_bumpstop] = ...
                computeSuspensionForce(obj, cornerState, suspensionDeflection, ...
                suspensionVelocity, K_eff, MR_eff)
            % Suspension force is measured relative to static equilibrium:
            % staticLoad carries the steady car weight, while spring/damper
            % and bump-stop deltas add transient load from chassis motion.
            F_spring = K_eff * suspensionDeflection;
            F_damper = obj.computeDamperForce(suspensionVelocity, MR_eff);

            % Dynamic states are zeroed at static ride height, and the
            % configured bump-stop length is free jounce travel from that
            % datum. Do not add static spring compression here; doing so
            % makes a 25.4 mm jounce stop engage after roughly 13 mm on R25.
            F_bumpstop = obj.computeBumpStopForce(suspensionDeflection);

            F_suspension = cornerState.staticLoad + F_spring + ...
                F_damper + F_bumpstop;
        end

        function F = computeDamperForce(obj, velocity, MR_eff)
            % COMPUTEDAMPERFORCE Digressive damper force in the wheel domain.
            %   The low-speed slope (compressionCoeff for v >= 0, reboundCoeff
            %   for v < 0, each * MR_eff^2) holds up to dampingKneeSpeed; above
            %   it the slope drops to lowSpeed * dampingHighSpeedRatio.
            %   dampingHighSpeedRatio = 1 or dampingKneeSpeed = Inf reproduces
            %   a linear damper exactly, so the default config is unchanged.
            MR2 = MR_eff * MR_eff;
            if velocity >= 0
                cLow = obj.dampingCoeff * MR2;
            else
                cLow = obj.reboundCoeff * MR2;
            end
            % Unified piecewise magnitude: low-speed up to the knee, then the
            % reduced high-speed slope. Equals |velocity| when ratio == 1 or
            % knee == Inf, giving the legacy linear law with no branch.
            av = abs(velocity);
            beyond = max(av - obj.dampingKneeSpeed, 0);
            mag = min(av, obj.dampingKneeSpeed) + obj.dampingHighSpeedRatio * beyond;
            F = cLow * mag * sign(velocity);
        end

        function F_tire = computeTireNormalForce(obj, cornerState, unsprungPosition)
            % The road is flat and fixed in z; unsprung downward motion adds
            % tire compression to the static tire deflection. Negative normal
            % force is clipped to zero to represent loss of contact.
            tireDeflection = cornerState.staticTireDeflection + unsprungPosition;
            F_tire = max(obj.tireSpringRate * tireDeflection, 0);
        end

        function compression = computeStaticSuspensionCompression(~, staticLoad, K_eff)
            K_eff = max(K_eff, eps);
            % This value is the physical spring compression at static load.
            % Bump-stop travel is defined relative to that equilibrium, so
            % the stop cannot contribute to the static-load calculation.
            compression = staticLoad / K_eff;
        end

        function force = computeBumpStopForce(obj, compression)
            force = 0;
            if compression > obj.bumpStopLength
                force = obj.bumpStopRate * (compression - obj.bumpStopLength);
            end
        end

        function n = integrationSubsteps(obj, dt)
            maxStep = obj.maxIntegrationStep;
            if ~isfinite(maxStep) || maxStep <= 0
                n = 1;
            else
                n = max(1, ceil(dt / maxStep));
            end
        end
    end
end
