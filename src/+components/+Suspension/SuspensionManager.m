classdef SuspensionManager < components.Suspension.SuspensionComponent
    % SUSPENSIONMANAGER Manages four corner suspension units
    % Creates and coordinates per-corner SimpleSuspension instances with
    % associated SuspensionState objects.
    %
    % Front corners (FL, FR) share identical suspension parameters.
    % Rear corners (RL, RR) share identical suspension parameters.
    % Each corner has its own independent SuspensionState for transient tracking.
    %
    % Usage:
    %   mgr = SuspensionManager(vehicleManager, ...)
    %   loads = mgr.computeCornerLoads(state, Fz_aero_front, Fz_aero_rear, totalMass, dt)
    %     loads.FL, loads.FR, loads.RL, loads.RR  - per-corner tire normal force [N]
    
    properties
        % Per-corner suspension units
        frontLeft      % SimpleSuspension (front params)
        frontRight     % SimpleSuspension (front params)
        rearLeft       % SimpleSuspension (rear params)
        rearRight      % SimpleSuspension (rear params)
        
        % Static front weight distribution (from VehicleManager)
        staticFrontWeight = 0.48  % [0-1]

        % Target front share of elastic roll stiffness [0-1].
        % Springs provide a natural front/rear split; anti-roll bars can only
        % add stiffness, so the manager adds the smallest positive bar rate
        % needed to move the natural split toward this target.
        frontRollStiffDist = 0.55

        % Cached static-equilibrium warmup data. Warmup used to integrate the
        % corner model for thousands of small steps; static support is an
        % algebraic spring/bump-stop balance, so we can solve and reuse it.
        warmupCacheKey = []
        warmupCache = []
    end
    
    methods
        function obj = SuspensionManager(vehicleManager, ...
                frontRollStiffDist, ...
                frontSpringRate, frontDampingCoeff, frontReboundCoeff, ...
                rearSpringRate, rearDampingCoeff, rearReboundCoeff, ...
                motionRatio, bumpStopLength, bumpStopRate, ...
                tireSpringRate, unsprungMass)
            % SUSPENSIONMANAGER Construct with front/rear suspension parameters
            %   SuspensionManager(vehicleManager, ...
            %       frontRollStiffDist, ...
            %       frontSpringRate, frontDampingCoeff, frontReboundCoeff, ...
            %       rearSpringRate, rearDampingCoeff, rearReboundCoeff, ...
            %       motionRatio, bumpStopLength, bumpStopRate, ...
            %       tireSpringRate, unsprungMass)
            %
            %   vehicleManager      - VehicleManager handle (geometry pulled by SimpleSuspension)
            %   frontRollStiffDist  - Front roll stiffness distribution [0-1]
            %   frontSpringRate     - Front heave spring rate [N/m]
            %   frontDampingCoeff   - Front compression damping [N·s/m]
            %   frontReboundCoeff   - Front rebound damping [N·s/m]
            %   rearSpringRate      - Rear heave spring rate [N/m]
            %   rearDampingCoeff    - Rear compression damping [N·s/m]
            %   rearReboundCoeff    - Rear rebound damping [N·s/m]
            %   motionRatio         - Installation motion ratio (shared)
            %   bumpStopLength      - Bump stop travel [m] (shared)
            %   bumpStopRate        - Bump stop stiffness [N/m] (shared)
            %   tireSpringRate      - Tire vertical stiffness [N/m] (shared)
            %   unsprungMass        - Per-corner unsprung mass [kg] (shared)
            
	            % Pull static weight distribution from VehicleManager
	            obj.staticFrontWeight = vehicleManager.staticFrontWeight;
	            obj.frontRollStiffDist = frontRollStiffDist;
            
	            % Create front corners (share front parameters, each has own state)
	            obj.frontLeft = components.Suspension.SimpleSuspension( ...
	                vehicleManager, obj.frontRollStiffDist, ...
	                frontSpringRate, frontDampingCoeff, frontReboundCoeff, ...
	                motionRatio, bumpStopLength, bumpStopRate, ...
	                tireSpringRate, unsprungMass);

	            obj.frontRight = components.Suspension.SimpleSuspension( ...
	                vehicleManager, obj.frontRollStiffDist, ...
	                frontSpringRate, frontDampingCoeff, frontReboundCoeff, ...
	                motionRatio, bumpStopLength, bumpStopRate, ...
	                tireSpringRate, unsprungMass);
            
            % Create rear corners (share rear parameters, each has own state)
	            % Rear roll stiffness distribution = 1 - front
	            rearRollStiffDist = 1 - obj.frontRollStiffDist;
            obj.rearLeft = components.Suspension.SimpleSuspension( ...
                vehicleManager, rearRollStiffDist, ...
                rearSpringRate, rearDampingCoeff, rearReboundCoeff, ...
                motionRatio, bumpStopLength, bumpStopRate, ...
                tireSpringRate, unsprungMass);
            
	            obj.rearRight = components.Suspension.SimpleSuspension( ...
	                vehicleManager, rearRollStiffDist, ...
	                rearSpringRate, rearDampingCoeff, rearReboundCoeff, ...
	                motionRatio, bumpStopLength, bumpStopRate, ...
	                tireSpringRate, unsprungMass);
	        end

	        function set.staticFrontWeight(obj, value)
	            obj.staticFrontWeight = utils.unitScalarOrDefault(value, 0.48);
	        end

	        function set.frontRollStiffDist(obj, value)
	            obj.frontRollStiffDist = utils.unitScalarOrDefault(value, 0.55);
	        end

        function syncVehicleGeometry(obj, vehicleManager)
            % SYNCVEHICLEGEOMETRY Refresh copied vehicle geometry.
            %
            % SuspensionManager and its four corner units cache geometry for
            % speed and clarity. VehicleManager remains the source of truth for
            % setup values, so synchronize before computing tire normal loads.
            if nargin < 2 || isempty(vehicleManager)
                return;
            end

            obj.staticFrontWeight = vehicleManager.staticFrontWeight;
            if ismethod(obj.frontLeft, 'syncVehicleGeometry')
                obj.frontLeft.syncVehicleGeometry(vehicleManager);
                obj.frontRight.syncVehicleGeometry(vehicleManager);
                obj.rearLeft.syncVehicleGeometry(vehicleManager);
                obj.rearRight.syncVehicleGeometry(vehicleManager);
            end
        end

        %% ---- Warmup: settle suspension to static equilibrium ----
        
	        function warmup(obj, totalMass, dt)
            % WARMUP Settle suspension state to static equilibrium
            %   warmup(totalMass, dt)
            %
            %   Static warmup has no aero, no load transfer, and zero damper
            %   velocity. That means the equilibrium corner position is found
            %   by solving:
            %       springForce(x) + bumpStopForce(x) = staticCornerLoad
            %   directly, instead of time-marching until velocity happens to
            %   decay. The precomputed snapshot is cached and then copied into
            %   each mutable SuspensionState.
            %
            %   totalMass - Total vehicle mass [kg]
            %   dt        - Kept for interface compatibility; not needed by
            %               the closed-form static solve.

	            if nargin < 3
	                dt = 0.001; %#ok<NASGU>
	            end
	            totalMass = utils.nonnegativeScalarOrDefault(totalMass, 280);

	            snapshot = obj.precomputeStaticWarmup(totalMass);
            obj.applyWarmupCorner(obj.frontLeft, snapshot.FL);
            obj.applyWarmupCorner(obj.frontRight, snapshot.FR);
            obj.applyWarmupCorner(obj.rearLeft, snapshot.RL);
            obj.applyWarmupCorner(obj.rearRight, snapshot.RR);
        end
        
        %% ---- Per-corner transient computation ----
        
	        function loads = computeCornerLoads(obj, state, Fz_aero_front, Fz_aero_rear, totalMass, dt, chassisKinematics)
            % COMPUTECORNERLOADS Compute demanded loads and update all four corners
            %   loads = computeCornerLoads(state, Fz_aero_front, Fz_aero_rear, totalMass, dt)
            %
            %   state          - VehicleState with ax, ay, speed, etc.
            %   Fz_aero_front  - Aero downforce on front axle [N]
            %   Fz_aero_rear   - Aero downforce on rear axle [N]
            %   totalMass      - Total vehicle mass [kg]
            %   dt             - Timestep [s]
            %   chassisKinematics - optional struct with per-corner
            %                      .displacement and .velocity fields
            %
            %   Returns struct with per-corner tire normal forces:
            %     loads.FL, loads.FR, loads.RL, loads.RR  [N]
            
	            Fz_aero_front = utils.scalarOrDefault(Fz_aero_front, 0);
	            Fz_aero_rear = utils.scalarOrDefault(Fz_aero_rear, 0);
	            totalMass = utils.nonnegativeScalarOrDefault(totalMass, 0);
	            dt = utils.nonnegativeScalarOrDefault(dt, 0);
	            W = totalMass * 9.81;
	            ax = utils.scalarOrDefault(state.ax, 0);
	            ay = utils.scalarOrDefault(state.ay, 0);

	            % Geometry is synchronized from VehicleManager before loads are computed.
	            tw = obj.frontLeft.trackWidth;
	            wb = obj.frontLeft.wheelbase;
	            cgH = obj.frontLeft.cgHeight;
	            frontWeightFrac = obj.staticFrontWeight;
	            rollStiffDist = obj.frontRollStiffDist;
            
            % --- Static weight per corner ---
            Fz_static_front = W * frontWeightFrac;
            Fz_static_rear  = W * (1 - frontWeightFrac);
            Fz_static_FL = Fz_static_front / 2;
            Fz_static_FR = Fz_static_front / 2;
            Fz_static_RL = Fz_static_rear  / 2;
            Fz_static_RR = Fz_static_rear  / 2;

	            if nargin >= 7 && ~isempty(chassisKinematics)
	                if isstruct(chassisKinematics) ...
	                        && isfield(chassisKinematics, 'displacement')
	                    disp = utils.cornerStructOrDefault( ...
	                        chassisKinematics.displacement);
	                else
	                    disp = utils.cornerStructOrDefault([]);
	                end
	                if isstruct(chassisKinematics) ...
	                        && isfield(chassisKinematics, 'velocity')
	                    vel = utils.cornerStructOrDefault( ...
	                        chassisKinematics.velocity);
	                else
	                    vel = utils.cornerStructOrDefault([]);
	                end

                % In chassis-coupled mode the sprung mass owns transient load
                % transfer. Static weight is the baseline load; aero and
                % acceleration move the chassis, and the corner spring/damper
                % forces below determine the distribution/moments. Any remaining
                % vertical-load residual is applied using the signed aero axle
                % split so the no-vertical-acceleration tire model preserves the
                % aero pitch moment without double-counting settled chassis pitch.
                externalVerticalLoad = W + Fz_aero_front + Fz_aero_rear;
                obj.frontLeft.updateCornerFromKinematics( ...
                    obj.frontLeft.state, Fz_static_FL, disp.FL, vel.FL);
                obj.frontRight.updateCornerFromKinematics( ...
                    obj.frontRight.state, Fz_static_FR, disp.FR, vel.FR);
                obj.rearLeft.updateCornerFromKinematics( ...
                    obj.rearLeft.state, Fz_static_RL, disp.RL, vel.RL);
                obj.rearRight.updateCornerFromKinematics( ...
                    obj.rearRight.state, Fz_static_RR, disp.RR, vel.RR);

                obj.applyAntiRollLoads(disp);
                obj.applyAeroLoadResidual( ...
                    Fz_aero_front, Fz_aero_rear, externalVerticalLoad);
                obj.redistributeVehicleNormalLoads(externalVerticalLoad);

                loads.FL = obj.frontLeft.state.tireNormalForce;
                loads.FR = obj.frontRight.state.tireNormalForce;
                loads.RL = obj.rearLeft.state.tireNormalForce;
                loads.RR = obj.rearRight.state.tireNormalForce;
                return;
            end

            % --- Aero downforce per corner (split evenly per axle) ---
            Fz_aero_FL = Fz_aero_front / 2;
            Fz_aero_FR = Fz_aero_front / 2;
            Fz_aero_RL = Fz_aero_rear  / 2;
            Fz_aero_RR = Fz_aero_rear  / 2;
            
            % --- Lateral load transfer ---
            % positive ay = left turn → load transfers to right side
            totalLatTransfer = totalMass * abs(ay) * cgH / tw;
            frontLatTransfer = totalLatTransfer * rollStiffDist;
            rearLatTransfer  = totalLatTransfer * (1 - rollStiffDist);
            
            sign_ay = sign(ay);
            Fz_lat_FL = -sign_ay * frontLatTransfer / 2;
            Fz_lat_FR =  sign_ay * frontLatTransfer / 2;
            Fz_lat_RL = -sign_ay * rearLatTransfer / 2;
            Fz_lat_RR =  sign_ay * rearLatTransfer / 2;
            
            % --- Longitudinal load transfer ---
            % positive ax (acceleration) → load transfers to rear
            totalLongTransfer = totalMass * ax * cgH / wb;
            Fz_long_FL = -totalLongTransfer / 2;
            Fz_long_FR = -totalLongTransfer / 2;
            Fz_long_RL =  totalLongTransfer / 2;
            Fz_long_RR =  totalLongTransfer / 2;
            
            % --- Total demanded load per corner ---
            demanded_FL = Fz_static_FL + Fz_aero_FL + Fz_lat_FL + Fz_long_FL;
            demanded_FR = Fz_static_FR + Fz_aero_FR + Fz_lat_FR + Fz_long_FR;
            demanded_RL = Fz_static_RL + Fz_aero_RL + Fz_lat_RL + Fz_long_RL;
            demanded_RR = Fz_static_RR + Fz_aero_RR + Fz_lat_RR + Fz_long_RR;
            
            % --- Update each corner's transient state ---
            obj.frontLeft.updateCorner( obj.frontLeft.state,  demanded_FL, dt);
            obj.frontRight.updateCorner(obj.frontRight.state, demanded_FR, dt);
            obj.rearLeft.updateCorner(  obj.rearLeft.state,   demanded_RL, dt);
            obj.rearRight.updateCorner( obj.rearRight.state,  demanded_RR, dt);
            obj.redistributeVehicleNormalLoads();

            % --- Return per-corner tire normal forces ---
            loads.FL = obj.frontLeft.state.tireNormalForce;
            loads.FR = obj.frontRight.state.tireNormalForce;
            loads.RL = obj.rearLeft.state.tireNormalForce;
            loads.RR = obj.rearRight.state.tireNormalForce;
        end
        
	        function pitchAngle = computePitchAngle(obj)
            % COMPUTEPITCHANGLE Compute pitch angle from suspension compression
            %   pitchAngle = computePitchAngle()
            %
            %   Uses the average front and rear damper positions (compression
            %   from static equilibrium) to determine the body pitch angle.
            %
            %   Positive pitch = nose up (e.g. rear compresses more under
            %   acceleration squat).
            %   Negative pitch = nose down (e.g. front compresses more under
            %   braking dive).
            %
            %   Geometry is trivialized to:
            %     pitchAngle = atan2(avgRearCompress - avgFrontCompress, wheelbase)
            
	            avgFrontCompress = (obj.frontLeft.state.damperPosition + ...
	                                obj.frontRight.state.damperPosition) / 2;
	            avgRearCompress  = (obj.rearLeft.state.damperPosition + ...
	                                obj.rearRight.state.damperPosition) / 2;

	            wheelbase = obj.frontLeft.wheelbase;
	            pitchAngle = atan2(avgRearCompress - avgFrontCompress, wheelbase);
	        end

        function [frontRollStiffness, rearRollStiffness, ...
                frontAntiRollStiffness, rearAntiRollStiffness, ...
                naturalFrontRollStiffness, naturalRearRollStiffness] = ...
                computeRollStiffnessContributions(obj)
            % COMPUTEROLLSTIFFNESSCONTRIBUTIONS Split roll stiffness by axle.
            %
            % The chassis roll equation needs total elastic roll stiffness, and
            % the corner-load path needs matching front/rear vertical forces.
            % Wheel springs give a natural distribution. If the configured
            % frontRollStiffDist asks for a different distribution, we add the
            % minimum positive anti-roll-bar stiffness to one axle. We never
            % subtract spring stiffness, because a real anti-roll bar cannot
            % make an axle less stiff than its springs already are.
	            halfTrack = obj.frontLeft.trackWidth / 2;
	            K_FL = obj.computeCornerWheelRate(obj.frontLeft);
            K_FR = obj.computeCornerWheelRate(obj.frontRight);
            K_RL = obj.computeCornerWheelRate(obj.rearLeft);
            K_RR = obj.computeCornerWheelRate(obj.rearRight);

            naturalFrontRollStiffness = (K_FL + K_FR) * halfTrack^2;
            naturalRearRollStiffness = (K_RL + K_RR) * halfTrack^2;
            naturalTotal = naturalFrontRollStiffness + naturalRearRollStiffness;

	            targetFront = utils.unitScalarOrDefault( ...
	                obj.frontRollStiffDist, 0.55);
	            targetFront = min(max(targetFront, 1e-3), 1 - 1e-3);
            frontAntiRollStiffness = 0;
            rearAntiRollStiffness = 0;
            if naturalTotal > eps
                naturalFrontShare = naturalFrontRollStiffness / naturalTotal;
                if targetFront > naturalFrontShare
                    frontAntiRollStiffness = ...
                        (targetFront * naturalTotal - naturalFrontRollStiffness) ...
                        / max(1 - targetFront, eps);
                elseif targetFront < naturalFrontShare
                    rearAntiRollStiffness = ...
                        ((1 - targetFront) * naturalTotal - naturalRearRollStiffness) ...
                        / max(targetFront, eps);
                end
            end

            frontAntiRollStiffness = max(0, frontAntiRollStiffness);
            rearAntiRollStiffness = max(0, rearAntiRollStiffness);
            frontRollStiffness = naturalFrontRollStiffness + frontAntiRollStiffness;
            rearRollStiffness = naturalRearRollStiffness + rearAntiRollStiffness;
        end

        function applyAntiRollLoads(obj, cornerDisplacement)
            % APPLYANTIROLLLOADS Add anti-roll-bar vertical load per axle.
            %
            % Positive chassis roll means right-side-down. An anti-roll bar
            % produces equal/opposite vertical forces on the two tires of its
            % axle: it adds load to the compressed side and removes it from the
            % extended side. The pair has zero net vertical force, so total
            % vehicle weight/downforce is conserved while roll moment is
            % resisted.
            [~, ~, frontAntiRollStiffness, rearAntiRollStiffness] = ...
                obj.computeRollStiffnessContributions();
	            track = max(obj.frontLeft.trackWidth, eps);

            frontRollAngle = (cornerDisplacement.FR - cornerDisplacement.FL) / track;
            rearRollAngle = (cornerDisplacement.RR - cornerDisplacement.RL) / track;

            frontCornerForce = frontAntiRollStiffness * frontRollAngle / track;
            rearCornerForce = rearAntiRollStiffness * rearRollAngle / track;
            frontCornerForce = obj.limitAntiRollTransfer( ...
                frontCornerForce, obj.frontLeft.state.tireNormalForce, ...
                obj.frontRight.state.tireNormalForce);
            rearCornerForce = obj.limitAntiRollTransfer( ...
                rearCornerForce, obj.rearLeft.state.tireNormalForce, ...
                obj.rearRight.state.tireNormalForce);

            obj.applyCornerAntiRollForce(obj.frontLeft, -frontCornerForce);
            obj.applyCornerAntiRollForce(obj.frontRight, frontCornerForce);
            obj.applyCornerAntiRollForce(obj.rearLeft, -rearCornerForce);
            obj.applyCornerAntiRollForce(obj.rearRight, rearCornerForce);
        end
    end

    methods (Access = private)
        function snapshot = precomputeStaticWarmup(obj, totalMass)
            % PRECOMPUTESTATICWARMUP Build or reuse static corner snapshot.
            %
            % The cache key contains every setup value that changes the static
            % equilibrium. If a setup script retunes spring rate, motion ratio,
            % bump stop, tire rate, total mass, or static weight distribution,
            % the key changes and the snapshot is recomputed automatically.
            key = obj.buildWarmupCacheKey(totalMass);
            if ~isempty(obj.warmupCache) && isequaln(obj.warmupCacheKey, key)
                snapshot = obj.warmupCache;
                return;
            end

	            totalMass = utils.nonnegativeScalarOrDefault(totalMass, 280);
	            frontWeight = obj.staticFrontWeight;
	            W = max(totalMass, 0) * 9.81;
	            frontLoad = W * frontWeight / 2;
	            rearLoad = W * (1 - frontWeight) / 2;

            snapshot.FL = obj.computeCornerStaticEquilibrium( ...
                obj.frontLeft, frontLoad);
            snapshot.FR = obj.computeCornerStaticEquilibrium( ...
                obj.frontRight, frontLoad);
            snapshot.RL = obj.computeCornerStaticEquilibrium( ...
                obj.rearLeft, rearLoad);
            snapshot.RR = obj.computeCornerStaticEquilibrium( ...
                obj.rearRight, rearLoad);

            obj.warmupCacheKey = key;
            obj.warmupCache = snapshot;
        end

        function key = buildWarmupCacheKey(obj, totalMass)
            % BUILDWARMUPCACHEKEY Numeric signature for static warmup inputs.
            key = [
	                utils.nonnegativeScalarOrDefault(totalMass, 280)
	                obj.staticFrontWeight
                obj.cornerWarmupKey(obj.frontLeft)
                obj.cornerWarmupKey(obj.frontRight)
                obj.cornerWarmupKey(obj.rearLeft)
                obj.cornerWarmupKey(obj.rearRight)
            ];
        end

	        function key = cornerWarmupKey(obj, corner)
            key = [
	                utils.nonnegativeScalarOrDefault(corner.springRate, 25000)
	                utils.nonnegativeScalarOrDefault(corner.motionRatio, 0.95)
	                utils.nonnegativeScalarOrDefault(corner.bumpStopLength, 0.025)
	                utils.nonnegativeScalarOrDefault(corner.bumpStopRate, 200000)
	                utils.positiveScalarOrDefault(corner.tireSpringRate, 200000)
            ];
        end

	        function equilibrium = computeCornerStaticEquilibrium(obj, corner, load)
            % COMPUTECORNERSTATICEQUILIBRIUM Solve static spring support.
            %
            % For x <= bumpStopLength:
            %   F = K*x
            % For x > bumpStopLength:
            %   F = K*x + K_bump*(x - bumpStopLength)
            % This closed form gives the same static endpoint the old warmup
            % loop approached, without the iteration count depending on dt,
            % damping, or numerical decay tolerance.
	            load = utils.nonnegativeScalarOrDefault(load, 0);
	            springRate = utils.nonnegativeScalarOrDefault(corner.springRate, 25000);
	            motionRatio = utils.nonnegativeScalarOrDefault(corner.motionRatio, 0.95);
	            tireSpringRate = utils.positiveScalarOrDefault( ...
	                corner.tireSpringRate, 200000);
	            wheelRate = springRate * motionRatio^2;
	            bumpRate = utils.nonnegativeScalarOrDefault( ...
	                corner.bumpStopRate, 200000);
	            bumpGap = utils.nonnegativeScalarOrDefault( ...
	                corner.bumpStopLength, 0.025);

            if load <= eps
                damperPosition = 0;
            elseif wheelRate <= eps
                if bumpRate > eps
                    damperPosition = bumpGap + load / bumpRate;
                else
                    damperPosition = 0;
                end
            elseif load <= wheelRate * bumpGap || bumpRate <= eps
                damperPosition = load / max(wheelRate, eps);
            else
                damperPosition = (load + bumpRate * bumpGap) ...
                    / max(wheelRate + bumpRate, eps);
            end

            springForce = wheelRate * damperPosition;
            bumpForce = 0;
            if damperPosition > bumpGap
                bumpForce = bumpRate * (damperPosition - bumpGap);
            end

            equilibrium.damperPosition = damperPosition;
            equilibrium.damperVelocity = 0;
            equilibrium.tireNormalForce = load;
	            equilibrium.tireDeflection = load / max(tireSpringRate, eps);
            equilibrium.suspensionForce = springForce + bumpForce;
            equilibrium.antiRollForce = 0;
            equilibrium.demandedLoad = load;
        end

        function applyWarmupCorner(~, corner, equilibrium)
            % APPLYWARMUPCORNER Copy a precomputed snapshot into mutable state.
            state = corner.state;
            state.damperPosition = equilibrium.damperPosition;
            state.damperVelocity = equilibrium.damperVelocity;
            state.tireDeflection = equilibrium.tireDeflection;
            state.tireNormalForce = equilibrium.tireNormalForce;
            state.suspensionForce = equilibrium.suspensionForce;
            state.antiRollForce = equilibrium.antiRollForce;
            state.demandedLoad = equilibrium.demandedLoad;
        end

	        function wheelRate = computeCornerWheelRate(obj, corner)
	            springRate = utils.nonnegativeScalarOrDefault(corner.springRate, 25000);
	            motionRatio = utils.nonnegativeScalarOrDefault(corner.motionRatio, 0.95);
	            wheelRate = springRate * motionRatio^2;
        end

	        function applyCornerAntiRollForce(obj, corner, force)
	            force = utils.scalarOrDefault(force, 0);
	            state = corner.state;
	            state.antiRollForce = force;
	            state.tireNormalForce = max(state.tireNormalForce + force, 0);
	            tireSpringRate = utils.positiveScalarOrDefault( ...
	                corner.tireSpringRate, 200000);
	            state.tireDeflection = state.tireNormalForce / max(tireSpringRate, eps);
	            state.suspensionForce = state.suspensionForce + force;
	            state.demandedLoad = state.demandedLoad + force;
	        end

        function applyAeroLoadResidual(obj, FzAeroFront, FzAeroRear, targetTotalLoad)
            % APPLYAEROLOADRESIDUAL Preserve aero split for the algebraic residual.
            %
            % Chassis displacement already contributes spring/damper tire loads.
            % Only the gap between that support and the simulator's external
            % vertical-load budget is added here. Once the chassis has settled and
            % spring loads carry the aero force, this residual naturally goes to
            % zero instead of adding the same aero load twice.
            aeroTotal = FzAeroFront + FzAeroRear;
            if abs(aeroTotal) <= eps
                return;
            end

            rawTotal = obj.frontLeft.state.demandedLoad ...
                + obj.frontRight.state.demandedLoad ...
                + obj.rearLeft.state.demandedLoad ...
                + obj.rearRight.state.demandedLoad;
            residualTotal = max(targetTotalLoad, 0) - rawTotal;
            if abs(residualTotal) <= 1e-12
                return;
            end

            frontResidual = residualTotal * FzAeroFront / aeroTotal;
            rearResidual = residualTotal * FzAeroRear / aeroTotal;
            obj.applyCornerLoadOffset(obj.frontLeft, frontResidual / 2);
            obj.applyCornerLoadOffset(obj.frontRight, frontResidual / 2);
            obj.applyCornerLoadOffset(obj.rearLeft, rearResidual / 2);
            obj.applyCornerLoadOffset(obj.rearRight, rearResidual / 2);
        end

	        function applyCornerLoadOffset(obj, corner, loadOffset)
	            loadOffset = utils.scalarOrDefault(loadOffset, 0);
	            state = corner.state;
	            state.demandedLoad = state.demandedLoad + loadOffset;
	            state.tireNormalForce = state.tireNormalForce + loadOffset;
	            tireSpringRate = utils.positiveScalarOrDefault( ...
	                corner.tireSpringRate, 200000);
	            state.tireDeflection = max(state.tireNormalForce, 0) ...
	                / max(tireSpringRate, eps);
	        end

        function redistributeVehicleNormalLoads(obj, targetTotalLoad)
            % REDISTRIBUTEVEHICLENORMALLOADS Enforce no-tension tire contact.
            %
            % Suspension and load-transfer equations can request a negative
            % normal load on one corner or even a whole axle during wheel lift.
            % A tire cannot pull on the road, but simply clamping each corner
            % to zero creates vertical force. Preserve either the vehicle's
            % requested raw load sum or, for chassis-coupled kinematics, the
            % explicitly conserved external vertical load. This is a compact
            % unilateral-contact approximation: no tire tension, no artificial
            % gain/loss of total vertical force.
            corners = {obj.frontLeft, obj.frontRight, obj.rearLeft, obj.rearRight};
            rawLoad = [
                obj.frontLeft.state.demandedLoad
                obj.frontRight.state.demandedLoad
                obj.rearLeft.state.demandedLoad
                obj.rearRight.state.demandedLoad
            ];
            if nargin < 2 || isempty(targetTotalLoad)
                totalLoad = sum(rawLoad);
            else
                totalLoad = targetTotalLoad;
            end
            totalLoad = max(totalLoad, 0);

            if totalLoad <= eps
                for i = 1:numel(corners)
                    obj.setCornerNormalLoad(corners{i}, 0);
                end
                return;
            end

            contactLoad = min(max(rawLoad, 0), totalLoad);
            positiveSum = sum(contactLoad);
            if positiveSum <= eps
                contactLoad(:) = totalLoad / numel(contactLoad);
            else
                % If one raw load was negative, scale the remaining positive
                % contact patches so the vehicle still carries exactly its
                % requested vertical load.
                contactLoad = contactLoad * (totalLoad / positiveSum);
            end

            for i = 1:numel(corners)
                obj.setCornerNormalLoad(corners{i}, contactLoad(i));
            end
        end

	        function force = limitAntiRollTransfer(obj, force, leftLoad, rightLoad)
            % LIMITANTIROLLTRANSFER Keep the unloading tire from going negative.
            %
            % Positive anti-roll force adds load to the right tire and removes
            % it from the left. Negative force does the opposite. Limiting the
            % transfer before it is applied keeps the anti-roll bar as a pure
            % equal/opposite load-transfer device with zero net vertical force.
	            force = utils.scalarOrDefault(force, 0);
	            leftLoad = utils.nonnegativeScalarOrDefault(leftLoad, 0);
	            rightLoad = utils.nonnegativeScalarOrDefault(rightLoad, 0);
	            if force >= 0
	                force = min(force, max(leftLoad, 0));
	            else
	                force = -min(abs(force), max(rightLoad, 0));
	            end
	        end

	        function setCornerNormalLoad(obj, corner, normalLoad)
	            normalLoad = utils.nonnegativeScalarOrDefault(normalLoad, 0);
	            tireSpringRate = utils.positiveScalarOrDefault( ...
	                corner.tireSpringRate, 200000);
	            corner.state.tireNormalForce = normalLoad;
	            corner.state.tireDeflection = normalLoad / max(tireSpringRate, eps);
	        end





	    end
	end
