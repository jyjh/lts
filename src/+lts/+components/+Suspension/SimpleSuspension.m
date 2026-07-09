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

        % --- Suspension tuning ---
        % DEPRECATED: the front/rear elastic load-transfer split is now
        % derived from springs + anti-roll bars (see
        % SuspensionManager.deriveFrontRollStiffnessFraction). Kept only to
        % preserve the constructor signature; no longer read for the split.
        rollStiffDist                % Legacy roll stiffness distribution [0-1]

        % --- Per-corner spring-damper ---
        springRate                   % Heave spring rate [N/m]
        dampingCoeff                 % Compression damping coefficient [N*s/m]
        reboundCoeff                 % Rebound damping coefficient [N*s/m]
        motionRatio                  % Installation motion ratio [dimensionless]
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
        function obj = SimpleSuspension(vehicleManager, rollStiffDist, ...
                springRate, dampingCoeff, reboundCoeff, ...
                motionRatio, bumpStopLength, bumpStopRate, ...
                tireSpringRate, unsprungMass, sprungMass)
            % SIMPLESUSPENSION Construct a per-corner suspension unit
            %   SimpleSuspension(vehicleManager, rollStiffDist, ...
            %       springRate, dampingCoeff, reboundCoeff, ...
            %       motionRatio, bumpStopLength, bumpStopRate, ...
            %       tireSpringRate, unsprungMass, sprungMass)
            %
            %   vehicleManager  - lts.vehicle.VehicleManager handle (geometry pulled at construction)
            %   rollStiffDist   - Roll stiffness distribution for this end [0-1]
            %   springRate      - Heave spring rate [N/m]
            %   dampingCoeff    - Compression damping [N*s/m]
            %   reboundCoeff    - Rebound damping [N*s/m]
            %   motionRatio     - Installation motion ratio
            %   bumpStopLength  - Bump stop travel before engagement [m]
            %   bumpStopRate    - Bump stop stiffness [N/m]
            %   tireSpringRate  - Vertical tire stiffness [N/m]
            %   unsprungMass    - Per-corner unsprung mass [kg]
            %   sprungMass      - Per-corner sprung mass [kg]

            % Pull vehicle-level geometry from lts.vehicle.VehicleManager
            obj.trackWidth   = vehicleManager.trackWidth;
            obj.wheelbase    = vehicleManager.wheelbase;
            obj.cgHeight     = vehicleManager.cgHeight;

            % Store suspension-specific parameters
            obj.rollStiffDist     = rollStiffDist;
            obj.springRate        = springRate;
            obj.dampingCoeff      = dampingCoeff;
            obj.reboundCoeff      = reboundCoeff;
            obj.motionRatio       = motionRatio;
            obj.bumpStopLength    = bumpStopLength;
            obj.bumpStopRate      = bumpStopRate;
            obj.tireSpringRate    = tireSpringRate;
            obj.unsprungMass      = unsprungMass;
            if nargin >= 11 && ~isempty(sprungMass)
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

        function updateCorner(obj, cornerState, demandedLoad, dt, antiRollBarForce)
            % UPDATECORNER Update one corner's transient state
            %   updateCorner(cornerState, demandedLoad, dt, antiRollBarForce)
            %
            %   cornerState  - SuspensionState handle for this corner
            %   demandedLoad - Total static + aero + load-transfer force [N]
            %   dt           - Timestep [s]
            %   antiRollBarForce - Optional axle coupling force [N]
            %
            %   Mutates cornerState in-place, updating:
            %     .damperPosition, .damperVelocity, .tireDeflection,
            %     .tireNormalForce, .suspensionForce, .demandedLoad
            %
            % Physics model:
            %   sprung mass:   m_s * z_s_ddot = demandedLoad - F_suspension
            %   unsprung mass: m_u * z_u_ddot = F_suspension - F_tire
            % where positive z is downward/compression-producing. Spring,
            % damper, bump-stop, and anti-roll-bar forces act through
            % F_suspension; the tire is a vertical spring to the road.

            if nargin < 5 || isempty(antiRollBarForce)
                antiRollBarForce = 0;
            end
            if ~isfinite(antiRollBarForce)
                antiRollBarForce = 0;
            end

            cornerState.demandedLoad = demandedLoad;
            cornerState.antiRollBarForce = antiRollBarForce;
            if cornerState.staticLoad <= 0 && cornerState.tireNormalForce <= 0
                obj.initializeStaticLoad(cornerState, max(demandedLoad, 0));
                cornerState.antiRollBarForce = antiRollBarForce;
            end

            nSubsteps = obj.integrationSubsteps(dt);
            if nSubsteps > 1
                subDt = dt / nSubsteps;
                for idx = 1:nSubsteps
                    obj.updateCorner(cornerState, demandedLoad, subDt, antiRollBarForce);
                end
                return;
            end

            z_s_prev = cornerState.sprungPosition;
            v_s_prev = cornerState.sprungVelocity;
            z_u_prev = cornerState.unsprungPosition;
            v_u_prev = cornerState.unsprungVelocity;

            MR_eff = obj.getEffectiveMotionRatio(cornerState);
            K_eff = obj.springRate * MR_eff^2;

            suspensionDeflection = z_s_prev - z_u_prev;
            suspensionVelocity = v_s_prev - v_u_prev;
            [F_suspension, ~, ~, ~] = obj.computeSuspensionForce( ...
                cornerState, suspensionDeflection, suspensionVelocity, K_eff, MR_eff);
            F_suspension = F_suspension + antiRollBarForce;
            F_tire = obj.computeTireNormalForce(cornerState, z_u_prev);

            z_s_ddot = (demandedLoad - F_suspension) / max(obj.sprungMass, eps);
            z_u_ddot = (F_suspension - F_tire) / max(obj.unsprungMass, eps);

            % Semi-implicit Euler integration
            v_s_new = v_s_prev + z_s_ddot * dt;
            v_u_new = v_u_prev + z_u_ddot * dt;
            z_s_new = z_s_prev + v_s_new * dt;
            z_u_new = z_u_prev + v_u_new * dt;

            suspensionDeflection = z_s_new - z_u_new;
            suspensionVelocity = v_s_new - v_u_new;
            [F_suspension, ~, ~, ~] = obj.computeSuspensionForce( ...
                cornerState, suspensionDeflection, suspensionVelocity, K_eff, MR_eff);
            F_suspension = F_suspension + antiRollBarForce;
            F_tire = obj.computeTireNormalForce(cornerState, z_u_new);

            % --- Update state ---
            cornerState.sprungPosition = z_s_new;
            cornerState.sprungVelocity = v_s_new;
            cornerState.unsprungPosition = z_u_new;
            cornerState.unsprungVelocity = v_u_new;
            cornerState.damperPosition = suspensionDeflection;
            cornerState.damperVelocity = suspensionVelocity;
            cornerState.tireDeflection = max( ...
                cornerState.staticTireDeflection + z_u_new, 0);
            cornerState.tireNormalForce = F_tire;
            cornerState.suspensionForce = F_suspension;
        end

        function updateCornerFromChassis(obj, cornerState, sprungPosition, ...
                sprungVelocity, dt, antiRollBarForce)
            % UPDATECORNERFROMCHASSIS Update unsprung/tire load from chassis motion.
            % Sprung motion is imposed by the chassis heave/pitch/roll model.
            %
            % Compared with updateCorner(), the sprung DOF is not integrated
            % here. The chassis has already solved it; this corner only
            % advances the unsprung mass against suspension and tire springs.

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

        function wheelRate = getEffectiveWheelRate(obj, cornerState)
            % GETEFFECTIVEWHEELRATE Small-signal wheel rate about static ride.
            % Includes bump-stop tangent stiffness when the corner is already
            % sitting on the stop at static equilibrium.
            if nargin < 2 || isempty(cornerState)
                cornerState = obj.state;
            end

            MR_eff = obj.getEffectiveMotionRatio(cornerState);
            wheelRate = obj.springRate * MR_eff^2;

            staticCompression = cornerState.staticSuspensionCompression;
            if ~isfinite(staticCompression)
                staticCompression = 0;
            end
            if obj.bumpStopRate > 0 && ...
                    staticCompression >= max(obj.bumpStopLength, 0) - 1e-12
                wheelRate = wheelRate + obj.bumpStopRate;
            end
        end
    end

    methods (Access = private)
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
            if suspensionVelocity >= 0
                C_eff = obj.dampingCoeff * MR_eff^2;
            else
                C_eff = obj.reboundCoeff * MR_eff^2;
            end
            F_damper = C_eff * suspensionVelocity;

            staticBump = obj.computeBumpStopForce( ...
                cornerState.staticSuspensionCompression);
            totalCompression = cornerState.staticSuspensionCompression + ...
                suspensionDeflection;
            F_bumpstop = obj.computeBumpStopForce(totalCompression) - staticBump;

            F_suspension = cornerState.staticLoad + F_spring + ...
                F_damper + F_bumpstop;
        end

        function F_tire = computeTireNormalForce(obj, cornerState, unsprungPosition)
            % The road is flat and fixed in z; unsprung downward motion adds
            % tire compression to the static tire deflection. Negative normal
            % force is clipped to zero to represent loss of contact.
            tireDeflection = cornerState.staticTireDeflection + unsprungPosition;
            F_tire = max(obj.tireSpringRate * tireDeflection, 0);
        end

        function compression = computeStaticSuspensionCompression(obj, staticLoad, K_eff)
            K_eff = max(K_eff, eps);
            bumpStopLength = max(obj.bumpStopLength, 0);
            if obj.bumpStopRate <= 0 || staticLoad <= K_eff * bumpStopLength
                compression = staticLoad / K_eff;
            else
                compression = (staticLoad + obj.bumpStopRate * bumpStopLength) / ...
                    (K_eff + obj.bumpStopRate);
            end
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
