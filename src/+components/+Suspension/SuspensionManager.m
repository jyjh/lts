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
        
        % Static front weight distribution: pulled from VehicleManager at
        % construction (no default — a bare instance is invalid).
        staticFrontWeight  % [0-1]

        % Gravitational acceleration [m/s^2], cached from VehicleManager so
        % the load-transfer algebra reads a single named constant.
        g = 9.80665

        % Roll-center height per axle [m], resolved from geometry at
        % construction. Drives the geometric (instantaneous) component of
        % lateral load transfer; 0 recovers the CG-height-only split.
        frontRollCenterHeight = 0
        rearRollCenterHeight = 0

        % Anti-roll bars per axle, resolved from geometry at construction.
        % Their wheel-rate stiffness is added to each axle's wheel-spring
        % rate to derive the elastic load-transfer split.
        frontAntiRollBar = []
        rearAntiRollBar  = []

        % Optional override for the front elastic load-transfer fraction
        % [0-1]. When set (non-NaN), it is used directly and the spring+ARB
        % derivation is skipped — this reproduces the legacy fixed 0.55
        % split for an exact A/B baseline. Leave NaN to derive from stiffness.
        rollStiffnessOverride = NaN

        % Opt-in coupling between the chassis roll DOFs and the elastic
        % load-transfer split. When true (and a chassis is linked), the
        % elastic transfer is redistributed by each axle's roll-angle
        % contribution, so chassis torsional compliance changes the F/R
        % balance under asymmetric load. Default false: the split is purely
        % stiffness-derived and the chassis roll is telemetry/aero only.
        coupleChassisRollToLoadTransfer = false

        % Linked chassis attitude model for the optional coupling above.
        chassis

        % Suspension and steering kinematic model
        geometry
    end

    properties (Transient = true) %#ok<MCNPC>
        % Lazily-cached run invariant: whether the linked chassis exposes
        % computeCornerKinematics. Empty = uncached.
        cachedChassisHasCornerKinematics
    end
    
    methods
        function obj = SuspensionManager(vehicleManager, ...
                frontRollStiffDist, ...
                frontSpringRate, frontDampingCoeff, frontReboundCoeff, ...
                rearSpringRate, rearDampingCoeff, rearReboundCoeff, ...
                motionRatio, bumpStopLength, bumpStopRate, ...
                tireSpringRate, unsprungMass, geometry, ...
                frontAntiRollBarRate, rearAntiRollBarRate)
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
            %   geometry            - SuspensionGeometry object (optional)
            %   frontAntiRollBarRate - Front anti-roll bar wheel rate [N/m] (optional)
            %   rearAntiRollBarRate  - Rear anti-roll bar wheel rate [N/m] (optional)
            
            if nargin < 14 || isempty(geometry)
                % Fallback when no geometry is supplied: a neutral kinematic
                % model (zero camber/toe gain) whose only non-trivial table is
                % the motion-ratio curve set from the passed motionRatio.
                geometry = components.Suspension.SuspensionGeometry(vehicleManager);
                geometry.frontMotionRatioCurve = motionRatio * [1 1 1];
                geometry.rearMotionRatioCurve = motionRatio * [1 1 1];
            end

            % Pull static weight distribution and gravity from VehicleManager
            obj.staticFrontWeight = vehicleManager.staticFrontWeight;
            if isprop(vehicleManager, 'g')
                obj.g = vehicleManager.g;
            end
            obj.geometry = geometry;

            % Resolve per-axle roll-center heights from the geometry model.
            if isprop(geometry, 'frontRollCenterHeight')
                obj.frontRollCenterHeight = geometry.frontRollCenterHeight;
            end
            if isprop(geometry, 'rearRollCenterHeight')
                obj.rearRollCenterHeight = geometry.rearRollCenterHeight;
            end

            % Resolve per-axle anti-roll bars from the geometry model.
            if isprop(geometry, 'frontAntiRollBar') && ~isempty(geometry.frontAntiRollBar)
                obj.frontAntiRollBar = geometry.frontAntiRollBar;
            end
            if isprop(geometry, 'rearAntiRollBar') && ~isempty(geometry.rearAntiRollBar)
                obj.rearAntiRollBar = geometry.rearAntiRollBar;
            end

            % Backward-compatible scalar ARB wheel rates (legacy call style):
            % when supplied AND the geometry did not already provide a bar,
            % install a unit-ratio AntiRollBar whose wheel-rate stiffness
            % equals the scalar. Geometry-object ARBs take precedence.
            if nargin >= 15 && ~isempty(frontAntiRollBarRate) && ...
                    isempty(obj.frontAntiRollBar)
                obj.frontAntiRollBar = obj.makeWheelRateBar(frontAntiRollBarRate);
            end
            if nargin >= 16 && ~isempty(rearAntiRollBarRate) && ...
                    isempty(obj.rearAntiRollBar)
                obj.rearAntiRollBar = obj.makeWheelRateBar(rearAntiRollBarRate);
            end

            totalSprungMass = max(vehicleManager.totalMass - 4 * unsprungMass, eps);
            frontSprungMass = max(totalSprungMass * obj.staticFrontWeight / 2, eps);
            rearSprungMass = max(totalSprungMass * (1 - obj.staticFrontWeight) / 2, eps);
            
            % Create front corners (share front parameters, each has own state)
            obj.frontLeft = components.Suspension.SimpleSuspension( ...
                vehicleManager, frontRollStiffDist, ...
                frontSpringRate, frontDampingCoeff, frontReboundCoeff, ...
                motionRatio, bumpStopLength, bumpStopRate, ...
                tireSpringRate, unsprungMass, frontSprungMass);
            
            obj.frontRight = components.Suspension.SimpleSuspension( ...
                vehicleManager, frontRollStiffDist, ...
                frontSpringRate, frontDampingCoeff, frontReboundCoeff, ...
                motionRatio, bumpStopLength, bumpStopRate, ...
                tireSpringRate, unsprungMass, frontSprungMass);
            
            % Create rear corners (share rear parameters, each has own state)
            % Rear roll stiffness distribution = 1 - front
            rearRollStiffDist = 1 - frontRollStiffDist;
            obj.rearLeft = components.Suspension.SimpleSuspension( ...
                vehicleManager, rearRollStiffDist, ...
                rearSpringRate, rearDampingCoeff, rearReboundCoeff, ...
                motionRatio, bumpStopLength, bumpStopRate, ...
                tireSpringRate, unsprungMass, rearSprungMass);
            
            obj.rearRight = components.Suspension.SimpleSuspension( ...
                vehicleManager, rearRollStiffDist, ...
                rearSpringRate, rearDampingCoeff, rearReboundCoeff, ...
                motionRatio, bumpStopLength, bumpStopRate, ...
                tireSpringRate, unsprungMass, rearSprungMass);
        end
        
        %% ---- Warmup: settle suspension to static equilibrium ----
        
        function warmup(obj, totalMass, dt)
            % WARMUP Settle suspension state to static equilibrium
            %   warmup(totalMass, dt)
            %
            %   Initializes deterministic per-corner static load and
            %   deflection state. Dynamic displacement states are measured
            %   from this equilibrium.
            %
            %   totalMass - Total vehicle mass [kg]
            %   dt        - Unused, kept for interface compatibility
            
            if nargin < 3
                dt = 0.001;
            end
            %#ok<NASGU>

            W = totalMass * obj.g;
            
            % Static weight per corner (no aero, no load transfer)
            Fz_static_front = W * obj.staticFrontWeight;
            Fz_static_rear  = W * (1 - obj.staticFrontWeight);
            demanded_FL = Fz_static_front / 2;
            demanded_FR = Fz_static_front / 2;
            demanded_RL = Fz_static_rear  / 2;
            demanded_RR = Fz_static_rear  / 2;

            obj.frontLeft.initializeStaticLoad( obj.frontLeft.state,  demanded_FL);
            obj.frontRight.initializeStaticLoad(obj.frontRight.state, demanded_FR);
            obj.rearLeft.initializeStaticLoad(  obj.rearLeft.state,   demanded_RL);
            obj.rearRight.initializeStaticLoad( obj.rearRight.state,  demanded_RR);
            obj.updateGeometry(0);
        end
        
        %% ---- Per-corner transient computation ----
        
        function loads = computeCornerLoads(obj, state, Fz_aero_front, Fz_aero_rear, totalMass, dt)
            % COMPUTECORNERLOADS Compute demanded loads and update all four corners
            %   loads = computeCornerLoads(state, Fz_aero_front, Fz_aero_rear, totalMass, dt)
            %
            %   state          - VehicleState with ax, ay, speed, etc.
            %   Fz_aero_front  - Aero downforce on front axle [N]
            %   Fz_aero_rear   - Aero downforce on rear axle [N]
            %   totalMass      - Total vehicle mass [kg]
            %   dt             - Timestep [s]
            %
            %   Returns struct with per-corner tire normal forces:
            %     loads.FL, loads.FR, loads.RL, loads.RR  [N]
            
            W = totalMass * obj.g;
            ax = state.ax;
            ay = state.ay;
            
            % Geometry (stored in corners from VehicleManager)
            tw = obj.frontLeft.trackWidth;
            wb = obj.frontLeft.wheelbase;
            cgH = obj.frontLeft.cgHeight;
            frontWeightFrac = obj.staticFrontWeight;
            rollStiffDist = obj.deriveFrontRollStiffnessFraction();
            hrcF = obj.frontRollCenterHeight;
            hrcR = obj.rearRollCenterHeight;
            
            % --- Static weight per corner ---
            Fz_static_front = W * frontWeightFrac;
            Fz_static_rear  = W * (1 - frontWeightFrac);
            Fz_static_FL = Fz_static_front / 2;
            Fz_static_FR = Fz_static_front / 2;
            Fz_static_RL = Fz_static_rear  / 2;
            Fz_static_RR = Fz_static_rear  / 2;
            
            % --- Aero downforce per corner (split evenly per axle) ---
            Fz_aero_FL = Fz_aero_front / 2;
            Fz_aero_FR = Fz_aero_front / 2;
            Fz_aero_RL = Fz_aero_rear  / 2;
            Fz_aero_RR = Fz_aero_rear  / 2;
            
            % --- Lateral load transfer ---
            % positive ay = left turn → load transfers to right side.
            % The total transfer is split into a geometric (roll-center)
            % component, which acts instantaneously through the linkage at
            % each axle, and an elastic component, which is distributed by
            % the roll-stiffness distribution. With hrcF = hrcR = 0 the
            % geometric part vanishes and the split collapses to the legacy
            % CG-height-only behavior.
            latForce = totalMass * abs(ay);
            totalLatTransfer = latForce * cgH / tw;
            geoLatFront = latForce * hrcF / tw;
            geoLatRear  = latForce * hrcR / tw;
            elasticLat = max(totalLatTransfer - geoLatFront - geoLatRear, 0);
            frontLatTransfer = geoLatFront + elasticLat * rollStiffDist;
            rearLatTransfer  = geoLatRear  + elasticLat * (1 - rollStiffDist);

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
            
            frontAntiRoll = obj.computeAntiRollBarForces( ...
                obj.frontLeft, obj.frontRight, obj.getAxleBarWheelRate(obj.frontAntiRollBar));
            rearAntiRoll = obj.computeAntiRollBarForces( ...
                obj.rearLeft, obj.rearRight, obj.getAxleBarWheelRate(obj.rearAntiRollBar));

            % --- Update each corner's transient state ---
            obj.frontLeft.updateCorner( ...
                obj.frontLeft.state, demanded_FL, dt, frontAntiRoll.left);
            obj.frontRight.updateCorner( ...
                obj.frontRight.state, demanded_FR, dt, frontAntiRoll.right);
            obj.rearLeft.updateCorner( ...
                obj.rearLeft.state, demanded_RL, dt, rearAntiRoll.left);
            obj.rearRight.updateCorner( ...
                obj.rearRight.state, demanded_RR, dt, rearAntiRoll.right);
            obj.updateGeometry(state.steer);

            % --- Return per-corner tire normal forces ---
            loads.FL = obj.frontLeft.state.tireNormalForce;
            loads.FR = obj.frontRight.state.tireNormalForce;
            loads.RL = obj.rearLeft.state.tireNormalForce;
            loads.RR = obj.rearRight.state.tireNormalForce;
        end

        function updateGeometry(obj, steerInput)
            % UPDATEGEOMETRY Refresh per-corner suspension kinematics.
            obj.updateCornerGeometry(obj.frontLeft,  'FL', steerInput);
            obj.updateCornerGeometry(obj.frontRight, 'FR', steerInput);
            obj.updateCornerGeometry(obj.rearLeft,   'RL', steerInput);
            obj.updateCornerGeometry(obj.rearRight,  'RR', steerInput);
        end

        function forces = getAntiRollBarForces(obj)
            % GETANTIROLLBARFORCES Per-corner ARB coupling forces [N].
            %   forces.FL, .FR, .RL, .RR
            %   Uses each axle bar's wheel-rate stiffness. Shared by the
            %   demanded-load and chassis-driven load paths so both apply the
            %   same axle coupling.
            front = obj.computeAntiRollBarForces( ...
                obj.frontLeft, obj.frontRight, ...
                obj.getAxleBarWheelRate(obj.frontAntiRollBar));
            rear = obj.computeAntiRollBarForces( ...
                obj.rearLeft, obj.rearRight, ...
                obj.getAxleBarWheelRate(obj.rearAntiRollBar));
            forces.FL = front.left;
            forces.FR = front.right;
            forces.RL = rear.left;
            forces.RR = rear.right;
        end

        function loads = estimateCornerLoads(obj, state, Fz_aero_front, Fz_aero_rear, totalMass)
            % ESTIMATECORNERLOADS Compute load-transfer demands without
            % advancing suspension state.

            W = totalMass * obj.g;
            ax = state.ax;
            ay = state.ay;

            tw = obj.frontLeft.trackWidth;
            wb = obj.frontLeft.wheelbase;
            cgH = obj.frontLeft.cgHeight;
            frontWeightFrac = obj.staticFrontWeight;
            rollStiffDist = obj.deriveFrontRollStiffnessFraction();
            hrcF = obj.frontRollCenterHeight;
            hrcR = obj.rearRollCenterHeight;

            Fz_static_front = W * frontWeightFrac;
            Fz_static_rear  = W * (1 - frontWeightFrac);
            Fz_static_FL = Fz_static_front / 2;
            Fz_static_FR = Fz_static_front / 2;
            Fz_static_RL = Fz_static_rear  / 2;
            Fz_static_RR = Fz_static_rear  / 2;

            Fz_aero_FL = Fz_aero_front / 2;
            Fz_aero_FR = Fz_aero_front / 2;
            Fz_aero_RL = Fz_aero_rear  / 2;
            Fz_aero_RR = Fz_aero_rear  / 2;

            latForce = totalMass * abs(ay);
            totalLatTransfer = latForce * cgH / tw;
            geoLatFront = latForce * hrcF / tw;
            geoLatRear  = latForce * hrcR / tw;
            elasticLat = max(totalLatTransfer - geoLatFront - geoLatRear, 0);
            frontLatTransfer = geoLatFront + elasticLat * rollStiffDist;
            rearLatTransfer  = geoLatRear  + elasticLat * (1 - rollStiffDist);

            sign_ay = sign(ay);
            Fz_lat_FL = -sign_ay * frontLatTransfer / 2;
            Fz_lat_FR =  sign_ay * frontLatTransfer / 2;
            Fz_lat_RL = -sign_ay * rearLatTransfer / 2;
            Fz_lat_RR =  sign_ay * rearLatTransfer / 2;

            totalLongTransfer = totalMass * ax * cgH / wb;
            Fz_long_FL = -totalLongTransfer / 2;
            Fz_long_FR = -totalLongTransfer / 2;
            Fz_long_RL =  totalLongTransfer / 2;
            Fz_long_RR =  totalLongTransfer / 2;

            loads.FL = max(Fz_static_FL + Fz_aero_FL + Fz_lat_FL + Fz_long_FL, 0);
            loads.FR = max(Fz_static_FR + Fz_aero_FR + Fz_lat_FR + Fz_long_FR, 0);
            loads.RL = max(Fz_static_RL + Fz_aero_RL + Fz_lat_RL + Fz_long_RL, 0);
            loads.RR = max(Fz_static_RR + Fz_aero_RR + Fz_lat_RR + Fz_long_RR, 0);
        end

        function loads = computeCornerLoadsFromChassis(obj, chassis, steer, dt)
            % COMPUTECORNERLOADSFROMCHASSIS Chassis-driven per-corner loads.
            %   loads = computeCornerLoadsFromChassis(chassis, steer, dt)
            %
            %   Reads the sprung-mass motion (heave/pitch/roll) the chassis
            %   has resolved at each suspension pickup and drives each
            %   corner's unsprung/tire state through it, returning the four
            %   tire normal forces. This is the chassis-coupled counterpart
            %   of computeCornerLoads: there the corners integrate a
            %   *demanded* load, here the sprung motion is *imposed* by the
            %   chassis so the attitude and load-transfer models share one
            %   sprung-mass motion.
            %
            %   chassis - linked SimpleChassis (or compatible) whose state
            %             already reflects the current heave/pitch/roll
            %   steer   - steering input [-1,1] (for kinematic refresh only)
            %   dt      - timestep [s]
            if nargin < 3 || isempty(steer)
                steer = 0;
            end
            if nargin < 4 || isempty(dt)
                dt = 0.001;
            end

            % Refresh per-corner sprung displacement/velocity from the chassis
            % attitude (positive = compression-producing). The chassis
            % capability is a run invariant; resolve it once.
            if isempty(obj.cachedChassisHasCornerKinematics)
                obj.cachedChassisHasCornerKinematics = ...
                    ismethod(chassis, 'computeCornerKinematics');
            end
            if obj.cachedChassisHasCornerKinematics
                chassis.computeCornerKinematics();
            end
            disp = chassis.state.cornerDisplacement;
            vel  = chassis.state.cornerVelocity;

            % ARB coupling forces per axle (wheel-rate stiffness from the bar
            % objects, mirroring computeCornerLoads).
            arb = obj.getAntiRollBarForces();

            % Drive each corner from the chassis-imposed sprung motion. The
            % suspension advances its unsprung mass against the tire spring.
            obj.frontLeft.updateCornerFromChassis( ...
                obj.frontLeft.state, disp.FL, vel.FL, dt, arb.FL);
            obj.frontRight.updateCornerFromChassis( ...
                obj.frontRight.state, disp.FR, vel.FR, dt, arb.FR);
            obj.rearLeft.updateCornerFromChassis( ...
                obj.rearLeft.state, disp.RL, vel.RL, dt, arb.RL);
            obj.rearRight.updateCornerFromChassis( ...
                obj.rearRight.state, disp.RR, vel.RR, dt, arb.RR);
            obj.updateGeometry(steer);

            loads.FL = obj.frontLeft.state.tireNormalForce;
            loads.FR = obj.frontRight.state.tireNormalForce;
            loads.RL = obj.rearLeft.state.tireNormalForce;
            loads.RR = obj.rearRight.state.tireNormalForce;
        end

        function frac = deriveFrontRollStiffnessFraction(obj)
            % DERIVEFRONTROLLSTIFFNESSFRACTION Front share [0-1] of the
            % elastic lateral load transfer, derived from actual per-axle
            % roll stiffness (wheel springs + anti-roll bars) rather than a
            % fixed magic scalar.
            %
            %   KwF = frontSpringRate * frontMotionRatio^2 + frontAntiRollBar.Kw_bar
            %   KwR = rearSpringRate  * rearMotionRatio^2  + rearAntiRollBar.Kw_bar
            %   frac = KwF / (KwF + KwR)
            %
            % If rollStiffnessOverride is set (non-NaN) it is returned
            % directly, reproducing the legacy fixed split for an exact A/B
            % baseline.

            if ~isnan(obj.rollStiffnessOverride)
                frac = min(1, max(0, obj.rollStiffnessOverride));
                return;
            end

            [KwF, KwR] = obj.getAxleRollStiffness();
            totalKw = KwF + KwR;
            if totalKw <= eps
                frac = 0.5;
            else
                frac = KwF / totalKw;
            end

            % Optional chassis-roll coupling: with a torsionally compliant
            % chassis the axle carrying more roll angle physically takes a
            % larger share of the elastic transfer. Redistribute by each
            % axle's stiffness*roll-angle product. Disabled by default.
            if obj.coupleChassisRollToLoadTransfer && ~isempty(obj.chassis) && ...
                    isa(obj.chassis, 'components.Chassis.ChassisComponent') && ...
                    ismethod(obj.chassis, 'getFrontRollAngle')
                phiF = obj.chassis.getFrontRollAngle();
                phiR = obj.chassis.getRearRollAngle();
                % Axle elastic force ~ K_roll * |phi|. Compare magnitudes.
                fF = abs(KwF * phiF);
                fR = abs(KwR * phiR);
                tot = fF + fR;
                if tot > eps
                    frac = fF / tot;
                end
            end
        end

        function [KwF, KwR] = getAxleRollStiffness(obj)
            % GETAXLEROLLSTIFFNESS Per-axle wheel-rate roll stiffness [N/m].
            % Wheel springs + anti-roll bar, referenced to the wheel. Shared
            % with the chassis roll model so the load-transfer split and the
            % chassis roll stiffness use the same numbers.
            KwSpringF = obj.frontLeft.springRate * obj.frontLeft.motionRatio^2;
            KwSpringR = obj.rearLeft.springRate  * obj.rearLeft.motionRatio^2;
            KwF = KwSpringF + obj.getAxleBarWheelRate(obj.frontAntiRollBar);
            KwR = KwSpringR + obj.getAxleBarWheelRate(obj.rearAntiRollBar);
        end

        function kw = getAxleBarWheelRate(~, antiRollBar)
            % GETAXLEBARWHEELRATE Wheel-rate roll stiffness of an axle's ARB.
            kw = 0;
            if ~isempty(antiRollBar) && isa(antiRollBar, 'components.Suspension.AntiRollBar')
                kw = antiRollBar.getWheelRateStiffness();
            end
        end

        function updateCornerGeometry(obj, cornerUnit, cornerName, steerInput)
            cornerState = cornerUnit.state;
            wheelTravel = cornerState.damperPosition / max(cornerUnit.motionRatio, eps);
            kin = obj.geometry.computeCornerKinematics(cornerName, wheelTravel, steerInput);

            cornerState.wheelTravel = kin.wheelTravel;
            cornerState.camberAngle = kin.camberAngle;
            cornerState.toeAngle = kin.toeAngle;
            cornerState.steerAngle = kin.steerAngle;
            cornerState.motionRatioEffective = max(kin.motionRatio, eps);
        end

        function cornerKinematics = getCornerKinematics(obj)
            % GETCORNERKINEMATICS Return tire-facing geometry for all corners.
            cornerKinematics.FL = obj.stateToKinematics(obj.frontLeft.state);
            cornerKinematics.FR = obj.stateToKinematics(obj.frontRight.state);
            cornerKinematics.RL = obj.stateToKinematics(obj.rearLeft.state);
            cornerKinematics.RR = obj.stateToKinematics(obj.rearRight.state);
        end
        
        function pitchAngle = computePitchAngle(obj)
            % COMPUTEPITCHANGLE Compute dynamic body pitch from sprung motion
            %   pitchAngle = computePitchAngle()
            %
            %   Uses average front and rear sprung-mass positions measured
            %   from static equilibrium. Static rake or undertray ride-height
            %   offsets are treated as the zero-pitch reference after warmup.
            %
            %   Positive pitch = nose up (e.g. rear compresses more under
            %   acceleration squat).
            %   Negative pitch = nose down (e.g. front compresses more under
            %   braking dive).
            %
            %   Geometry is simplified to:
            %     pitchAngle = atan2(avgRearSprungDown - avgFrontSprungDown, wheelbase)
            
            avgFrontSprungDown = (obj.frontLeft.state.sprungPosition + ...
                                  obj.frontRight.state.sprungPosition) / 2;
            avgRearSprungDown  = (obj.rearLeft.state.sprungPosition + ...
                                  obj.rearRight.state.sprungPosition) / 2;
            
            pitchAngle = atan2(avgRearSprungDown - avgFrontSprungDown, ...
                obj.frontLeft.wheelbase);
        end

        function setAntiRollBarRates(obj, frontRate, rearRate)
            % SETANTIROLLBARRATES Configure front/rear anti-roll bars from a
            %   wheel-rate stiffness [N/m].
            %   The rates are interpreted as the bar's wheel-rate roll
            %   stiffness (the same quantity getWheelRateStiffness returns),
            %   so an AntiRollBar is built whose wheel rate equals the input.
            if nargin >= 2 && ~isempty(frontRate)
                obj.frontAntiRollBar = obj.makeWheelRateBar(max(frontRate, 0));
            end
            if nargin >= 3 && ~isempty(rearRate)
                obj.rearAntiRollBar = obj.makeWheelRateBar(max(rearRate, 0));
            end
        end
    end

    methods (Static, Access = private)
        function kin = stateToKinematics(state)
            kin.wheelTravel = state.wheelTravel;
            kin.camberAngle = state.camberAngle;
            kin.toeAngle = state.toeAngle;
            kin.steerAngle = state.steerAngle;
            kin.motionRatio = state.motionRatioEffective;
        end
    end

    methods (Access = private)
        function bar = makeWheelRateBar(~, wheelRate)
            % MAKEWHEELRATEBAR Build a unit-ratio ARB whose wheel-rate roll
            % stiffness equals the requested value [N/m]. Used by
            % setAntiRollBarRates so the input is interpreted as Kw_bar.
            wheelRate = max(0, wheelRate);
            if wheelRate <= 0
                bar = components.Suspension.AntiRollBar();
            else
                bar = components.Suspension.AntiRollBar(wheelRate, 1, 1, true);
            end
        end

        function forces = computeAntiRollBarForces(obj, leftUnit, rightUnit, rate)
            forces = struct('left', 0, 'right', 0);
            rate = max(rate, 0);
            if rate <= 0
                return;
            end

            leftTravel = obj.computeAntiRollBarTravel(leftUnit);
            rightTravel = obj.computeAntiRollBarTravel(rightUnit);
            force = rate * (rightTravel - leftTravel);
            if ~isfinite(force)
                force = 0;
            end

            forces.left = -force;
            forces.right = force;
        end

        function wheelTravel = computeAntiRollBarTravel(~, cornerUnit)
            cornerState = cornerUnit.state;
            motionRatio = cornerUnit.motionRatio;
            if isprop(cornerState, 'motionRatioEffective') && ...
                    cornerState.motionRatioEffective > 0
                motionRatio = cornerState.motionRatioEffective;
            end
            wheelTravel = cornerState.damperPosition / max(motionRatio, eps);
            if ~isfinite(wheelTravel)
                wheelTravel = 0;
            end
        end
    end
end
