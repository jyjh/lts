classdef SuspensionManager < lts.components.Suspension.SuspensionComponent
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
    %   loads = mgr.computeCornerLoadsFromChassis(chassis, steer, dt)
    %     loads.FL, loads.FR, loads.RL, loads.RR  - per-corner tire normal force [N]
    
    properties
        % Per-corner suspension units
        frontLeft      % SimpleSuspension (front params)
        frontRight     % SimpleSuspension (front params)
        rearLeft       % SimpleSuspension (rear params)
        rearRight      % SimpleSuspension (rear params)
        
        % Static front weight distribution: pulled from lts.vehicle.VehicleManager at
        % construction (no default — a bare instance is invalid).
        staticFrontWeight  % [0-1]

        % Gravitational acceleration [m/s^2], cached from lts.vehicle.VehicleManager so
        % the load-transfer algebra reads a single named constant.
        g = 9.80665

        % Roll-center position per axle [m], resolved from geometry at
        % construction. Height drives the geometric (instantaneous) component
        % of lateral load transfer. Lateral is the signed +1g kinematic datum
        % scaled by current axle lateral acceleration.
        frontRollCenterHeight = 0
        rearRollCenterHeight = 0
        frontRollCenterLateral = 0
        rearRollCenterLateral = 0

        % Longitudinal anti-geometry fractions [0-1], resolved from geometry
        % at construction. Anti-dive: share of the sprung braking transfer
        % reacted by the front links (no spring pitch). Anti-squat: share of
        % the acceleration transfer reacted by the rear links. 0 = legacy
        % spring-only pitch path. Read by SimpleChassis.updateFromAccelerations.
        frontAntiDiveFraction = 0
        rearAntiSquatFraction = 0

        % Anti-roll bars per axle, resolved from geometry at construction.
        % Their differential wheel-coupling rate is converted to an
        % equivalent independent-corner rate (2*B_bar) when deriving axle
        % roll stiffness and elastic load-transfer split.
        frontAntiRollBar = []
        rearAntiRollBar  = []
        frontAntiRollBarWheelRate = 0
        rearAntiRollBarWheelRate = 0

        % Shared digressive-damper parameters forwarded to every corner's
        % SimpleSuspension. Defaults reproduce the legacy linear damper.
        dampingKneeSpeed = Inf       % wheel-domain low->high speed knee [m/s]
        dampingHighSpeedRatio = 1.0  % high-speed slope / low-speed slope [-]
        % Rebound-side knee override [m/s]; NaN shares dampingKneeSpeed.
        dampingReboundKneeSpeed = NaN

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

    properties (Transient = true)
        % Empty until the chassis API is first queried.
        cachedChassisHasCornerKinematics
    end
    
    methods
        function obj = SuspensionManager(vehicleManager, ...
                frontSpringRate, frontDampingCoeff, frontReboundCoeff, ...
                rearSpringRate, rearDampingCoeff, rearReboundCoeff, ...
                motionRatio, bumpStopLength, bumpStopRate, ...
                tireSpringRate, unsprungMass, geometry, ...
                frontAntiRollBarRate, rearAntiRollBarRate, ...
                dampingKneeSpeed, dampingHighSpeedRatio, dampingReboundKneeSpeed)
            % SUSPENSIONMANAGER Construct with front/rear suspension parameters
            %   SuspensionManager(vehicleManager, ...
            %       frontSpringRate, frontDampingCoeff, frontReboundCoeff, ...
            %       rearSpringRate, rearDampingCoeff, rearReboundCoeff, ...
            %       motionRatio, bumpStopLength, bumpStopRate, ...
            %       tireSpringRate, unsprungMass, geometry)
            %
            %   vehicleManager      - lts.vehicle.VehicleManager handle (geometry pulled by SimpleSuspension)
            %   frontSpringRate     - Front heave spring rate [N/m]
            %   frontDampingCoeff   - Front low-speed compression slope [N·s/m]
            %   frontReboundCoeff   - Front low-speed rebound slope [N·s/m]
            %   rearSpringRate      - Rear heave spring rate [N/m]
            %   rearDampingCoeff    - Rear low-speed compression slope [N·s/m]
            %   rearReboundCoeff    - Rear low-speed rebound slope [N·s/m]
            %   motionRatio         - Installation motion ratio (shared)
            %   bumpStopLength      - Bump stop travel [m] (shared)
            %   bumpStopRate        - Bump stop stiffness [N/m] (shared)
            %   tireSpringRate      - Tire vertical stiffness [N/m] (shared)
            %   unsprungMass        - Per-corner unsprung mass [kg] (shared)
            %   geometry            - SuspensionGeometry object (optional)
            %   frontAntiRollBarRate - Front ARB coupling rate B [N/m] (optional)
            %   rearAntiRollBarRate  - Rear ARB coupling rate B [N/m] (optional)
            %   dampingKneeSpeed      - Shared digressive-damper knee [m/s] (optional, Inf=linear)
            %   dampingHighSpeedRatio - Shared high-speed slope ratio [-] (optional, 1.0=linear)
            %   dampingReboundKneeSpeed - Rebound knee override [m/s] (optional, NaN=shared)
            
            if nargin < 13 || isempty(geometry)
                % Fallback when no geometry is supplied: a neutral kinematic
                % model (zero camber/toe gain) whose only non-trivial table is
                % the motion-ratio curve set from the passed motionRatio.
                geometry = lts.components.Suspension.SuspensionGeometry(vehicleManager);
                geometry.frontMotionRatioCurve = motionRatio * [1 1 1];
                geometry.rearMotionRatioCurve = motionRatio * [1 1 1];
            end

            % Pull static weight distribution and gravity from lts.vehicle.VehicleManager
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
            if isprop(geometry, 'frontRollCenterLateral')
                obj.frontRollCenterLateral = geometry.frontRollCenterLateral;
            end
            if isprop(geometry, 'rearRollCenterLateral')
                obj.rearRollCenterLateral = geometry.rearRollCenterLateral;
            end
            if isprop(geometry, 'frontAntiDiveFraction')
                obj.frontAntiDiveFraction = geometry.frontAntiDiveFraction;
            end
            if isprop(geometry, 'rearAntiSquatFraction')
                obj.rearAntiSquatFraction = geometry.rearAntiSquatFraction;
            end

            % Resolve per-axle anti-roll bars from the geometry model.
            if isprop(geometry, 'frontAntiRollBar') && ~isempty(geometry.frontAntiRollBar)
                obj.frontAntiRollBar = geometry.frontAntiRollBar;
            end
            if isprop(geometry, 'rearAntiRollBar') && ~isempty(geometry.rearAntiRollBar)
                obj.rearAntiRollBar = geometry.rearAntiRollBar;
            end

            % Backward-compatible scalar ARB coupling rates (legacy call style):
            % when supplied AND the geometry did not already provide a bar,
            % install a unit-ratio AntiRollBar whose wheel-rate stiffness
            % equals the scalar. Geometry-object ARBs take precedence.
            if nargin >= 14 && ~isempty(frontAntiRollBarRate) && ...
                    isempty(obj.frontAntiRollBar)
                obj.frontAntiRollBar = obj.makeWheelRateBar(frontAntiRollBarRate);
            end
            if nargin >= 15 && ~isempty(rearAntiRollBarRate) && ...
                    isempty(obj.rearAntiRollBar)
                obj.rearAntiRollBar = obj.makeWheelRateBar(rearAntiRollBarRate);
            end
            obj.frontAntiRollBarWheelRate = obj.getAxleBarWheelRate(obj.frontAntiRollBar);
            obj.rearAntiRollBarWheelRate = obj.getAxleBarWheelRate(obj.rearAntiRollBar);

            % Shared digressive-damper knee/ratio forwarded to every corner.
            % Defaults (Inf, 1.0) reproduce the legacy linear damper.
            if nargin >= 16 && ~isempty(dampingKneeSpeed) && isnumeric(dampingKneeSpeed) ...
                    && (isinf(dampingKneeSpeed) || (isscalar(dampingKneeSpeed) && ...
                        isfinite(dampingKneeSpeed) && dampingKneeSpeed > 0))
                obj.dampingKneeSpeed = dampingKneeSpeed;
            end
            if nargin >= 17 && ~isempty(dampingHighSpeedRatio) && isnumeric(dampingHighSpeedRatio) ...
                    && isscalar(dampingHighSpeedRatio) && isfinite(dampingHighSpeedRatio) ...
                    && dampingHighSpeedRatio > 0
                obj.dampingHighSpeedRatio = dampingHighSpeedRatio;
            end
            if nargin >= 18 && ~isempty(dampingReboundKneeSpeed) && ...
                    isnumeric(dampingReboundKneeSpeed) && isscalar(dampingReboundKneeSpeed) && ...
                    (isinf(dampingReboundKneeSpeed) || ...
                    (isfinite(dampingReboundKneeSpeed) && dampingReboundKneeSpeed > 0))
                obj.dampingReboundKneeSpeed = dampingReboundKneeSpeed;
            end

            totalSprungMass = max(vehicleManager.totalMass - 4 * unsprungMass, eps);
            frontSprungMass = max(totalSprungMass * obj.staticFrontWeight / 2, eps);
            rearSprungMass = max(totalSprungMass * (1 - obj.staticFrontWeight) / 2, eps);

            % Create the four corner units. Front corners share front
            % parameters, rear corners share rear parameters; each corner
            % owns an independent transient state.
            axleParams = { ...
                frontSpringRate, frontDampingCoeff, frontReboundCoeff, frontSprungMass; ...
                rearSpringRate, rearDampingCoeff, rearReboundCoeff, rearSprungMass};
            cornerNames = {'frontLeft', 'frontRight', 'rearLeft', 'rearRight'};
            for idx = 1:numel(cornerNames)
                axle = axleParams(1 + (idx > 2), :);
                obj.(cornerNames{idx}) = lts.components.Suspension.SimpleSuspension( ...
                    vehicleManager, ...
                    axle{1}, axle{2}, axle{3}, ...
                    motionRatio, bumpStopLength, bumpStopRate, ...
                    tireSpringRate, unsprungMass, axle{4}, ...
                    obj.dampingKneeSpeed, obj.dampingHighSpeedRatio, ...
                    obj.dampingReboundKneeSpeed);
            end
        end
        
        %% ---- Warmup: settle suspension to static equilibrium ----
        
        function warmup(obj, totalMass, ~)
            % Initialize per-corner static equilibrium.
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
            %   Uses each axle bar's wheel-rate stiffness.
            front = obj.computeAntiRollBarForces( ...
                obj.frontLeft, obj.frontRight, ...
                obj.frontAntiRollBarWheelRate);
            rear = obj.computeAntiRollBarForces( ...
                obj.rearLeft, obj.rearRight, ...
                obj.rearAntiRollBarWheelRate);
            forces.FL = front.left;
            forces.FR = front.right;
            forces.RL = rear.left;
            forces.RR = rear.right;
        end

        function loads = computeCornerLoadsFromChassis(obj, chassis, steer, dt, predictDt)
            % COMPUTECORNERLOADSFROMCHASSIS Chassis-driven per-corner loads.
            %   loads = computeCornerLoadsFromChassis(chassis, steer, dt)
            %   loads = computeCornerLoadsFromChassis(chassis, steer, dt, predictDt)
            %
            %   Reads the sprung-mass motion (heave/pitch/roll) the chassis
            %   has resolved at each suspension pickup and drives each
            %   corner's unsprung/tire state through it, returning the four
            %   tire normal forces. The attitude and load-transfer models
            %   share one sprung-mass motion: the chassis resolves it, the
            %   suspension reacts to it, and elastic load transfer emerges
            %   from the corner spring/ARB/damper forces.
            %
            %   predictDt > 0 extrapolates the chassis attitude forward by
            %   that horizon (corner displacement + corner velocity *
            %   predictDt, first order) before imposing it on the suspension.
            %   This removes the one-step stagger in which the step's tire
            %   loads were generated by the previous step's attitude — the
            %   structural source of the nonphysical ~10-15 Hz Fz
            %   oscillation under heavy longitudinal loading. predictDt = 0
            %   (default) evaluates at the current attitude (legacy).
            %
            %   chassis - linked SimpleChassis (or compatible) whose state
            %             already reflects the current heave/pitch/roll
            %   steer   - steering input [-1,1] (for kinematic refresh only)
            %   dt      - timestep [s]
            %   predictDt - attitude extrapolation horizon [s] (optional)
            if nargin < 3 || isempty(steer)
                steer = 0;
            end
            if nargin < 4 || isempty(dt)
                dt = 0.001;
            end
            if nargin < 5 || isempty(predictDt) || ~isfinite(predictDt) || predictDt <= 0
                predictDt = 0;
            end

            % Refresh per-corner sprung displacement/velocity from the chassis
            % attitude (positive = compression-producing). ChassisComponent
            % declares this method, so avoid probing for it in the hot loop.
            chassis.computeCornerKinematics();
            disp = chassis.state.cornerDisplacement;
            vel  = chassis.state.cornerVelocity;
            if predictDt > 0
                disp = struct( ...
                    'FL', disp.FL + vel.FL * predictDt, ...
                    'FR', disp.FR + vel.FR * predictDt, ...
                    'RL', disp.RL + vel.RL * predictDt, ...
                    'RR', disp.RR + vel.RR * predictDt);
            end

            % ARB coupling forces per axle (differential rate from the bar
            % objects).
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
            loads = obj.applyChassisGeometricTransfer(loads, chassis);
        end

        function loads = initializeCornerLoadsFromChassis(obj, chassis, steer)
            % INITIALIZECORNERLOADSFROMCHASSIS Put each unsprung mass and tire
            % spring in equilibrium with an already-seeded chassis attitude.
            if nargin < 3 || isempty(steer)
                steer = 0;
            end

            chassis.computeCornerKinematics();
            disp = chassis.state.cornerDisplacement;
            vel = chassis.state.cornerVelocity;
            arb = obj.getAntiRollBarForces();

            obj.frontLeft.initializeCornerFromChassis( ...
                obj.frontLeft.state, disp.FL, vel.FL, arb.FL);
            obj.frontRight.initializeCornerFromChassis( ...
                obj.frontRight.state, disp.FR, vel.FR, arb.FR);
            obj.rearLeft.initializeCornerFromChassis( ...
                obj.rearLeft.state, disp.RL, vel.RL, arb.RL);
            obj.rearRight.initializeCornerFromChassis( ...
                obj.rearRight.state, disp.RR, vel.RR, arb.RR);
            obj.updateGeometry(steer);

            loads.FL = obj.frontLeft.state.tireNormalForce;
            loads.FR = obj.frontRight.state.tireNormalForce;
            loads.RL = obj.rearLeft.state.tireNormalForce;
            loads.RR = obj.rearRight.state.tireNormalForce;
            loads = obj.applyChassisGeometricTransfer(loads, chassis);
        end

        function frac = deriveFrontRollStiffnessFraction(obj)
            % DERIVEFRONTROLLSTIFFNESSFRACTION Front share [0-1] of the
            % elastic lateral load transfer, derived from actual per-axle
            % roll stiffness (wheel springs + anti-roll bars) rather than a
            % fixed magic scalar.
            %
            %   KwF = mean front wheel rate + 2*frontAntiRollBar.B_bar
            %   KwR = mean rear wheel rate  + 2*rearAntiRollBar.B_bar
            %   frac = KwF / (KwF + KwR)
            %
            % If rollStiffnessOverride is set (non-NaN) it is returned
            % directly, reproducing the legacy fixed split for an exact A/B
            % baseline.

            if ~isnan(obj.rollStiffnessOverride)
                frac = lts.util.saturate(obj.rollStiffnessOverride);
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
            % Structural method probes (no isa on ChassisComponent) keep the
            % Suspension package usable without the Chassis package.
            if obj.coupleChassisRollToLoadTransfer && ~isempty(obj.chassis) && ...
                    ismethod(obj.chassis, 'getFrontRollAngle') && ...
                    ismethod(obj.chassis, 'getRearRollAngle')
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
            % GETAXLEROLLSTIFFNESS Equivalent per-corner axle rate [N/m].
            % The spring term is the mean of the current left/right tangent
            % rates. An ARB rate B applies +/-B*(zR-zL), so it contributes
            % 2*B to this equivalent rate. The chassis conversion Kw*t^2/2
            % then exactly matches the corner-force roll moment B*t^2.
            KwSpringF = 0.5 * ( ...
                obj.fastEffectiveWheelRate(obj.frontLeft, obj.frontLeft.state) + ...
                obj.fastEffectiveWheelRate(obj.frontRight, obj.frontRight.state));
            KwSpringR = 0.5 * ( ...
                obj.fastEffectiveWheelRate(obj.rearLeft, obj.rearLeft.state) + ...
                obj.fastEffectiveWheelRate(obj.rearRight, obj.rearRight.state));
            KwF = KwSpringF + 2 * obj.frontAntiRollBarWheelRate;
            KwR = KwSpringR + 2 * obj.rearAntiRollBarWheelRate;
        end

        function kw = getAxleBarWheelRate(~, antiRollBar)
            % GETAXLEBARWHEELRATE Differential wheel-coupling rate of an ARB.
            kw = 0;
            if ~isempty(antiRollBar) && isa(antiRollBar, 'lts.components.Suspension.AntiRollBar')
                kw = antiRollBar.getWheelRateStiffness();
            end
        end

        function updateCornerGeometry(obj, cornerUnit, cornerName, steerInput)
            cornerState = cornerUnit.state;
            % sprungPosition and unsprungPosition are wheel-center-domain
            % coordinates. Their difference is therefore already physical
            % wheel travel; motion ratio has already entered the force law as
            % MR^2 and must not be applied a second time here.
            wheelTravel = cornerState.damperPosition;
            kin = obj.geometry.computeCornerKinematics(cornerName, wheelTravel, steerInput);

            % Geometry tables return camber relative to the chassis. Rotate
            % the wheel-top vector into the road frame so chassis roll is not
            % silently omitted from the Magic Formula inclination input.
            rollAngle = obj.readChassisRollAngle(cornerName);
            wheelHeading = kin.steerAngle + kin.toeAngle;
            kin.camberAngle = obj.applyChassisRollCamber( ...
                kin.camberAngle, wheelHeading, cornerName, rollAngle);

            cornerState.wheelTravel = kin.wheelTravel;
            cornerState.camberAngle = kin.camberAngle;
            cornerState.toeAngle = kin.toeAngle;
            cornerState.steerAngle = kin.steerAngle;
            cornerState.motionRatioEffective = max(kin.motionRatio, eps);
            cornerState.xPosition = kin.xPosition;
            cornerState.yPosition = kin.yPosition;
            cornerState.wheelCenterXPosition = kin.wheelCenterXPosition;
            cornerState.wheelCenterYPosition = kin.wheelCenterYPosition;
            cornerState.kingpinXPosition = kin.kingpinXPosition;
            cornerState.kingpinYPosition = kin.kingpinYPosition;
            cornerState.casterAngle = kin.casterAngle;
            cornerState.kingpinInclination = kin.kingpinInclination;
            cornerState.mechanicalTrail = kin.mechanicalTrail;
            cornerState.scrubRadius = kin.scrubRadius;
            cornerState.kingpinOffset = kin.kingpinOffset;
            cornerState.rollCenterHeight = kin.rollCenterHeight;
            cornerState.rollCenterLateral = kin.rollCenterLateral;
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

    end

    methods (Static, Access = private)
        function wheelRate = fastEffectiveWheelRate(cornerUnit, cornerState)
            motionRatio = cornerUnit.motionRatio;
            if cornerState.motionRatioEffective > 0
                motionRatio = cornerState.motionRatioEffective;
            end
            motionRatio = max(motionRatio, eps);
            wheelRate = cornerUnit.springRate * motionRatio^2;

            if cornerUnit.bumpStopRate > 0 && ...
                    cornerState.damperPosition >= ...
                    max(cornerUnit.bumpStopLength, 0) - 1e-12
                wheelRate = wheelRate + cornerUnit.bumpStopRate;
            end
        end

        function kin = stateToKinematics(state)
            kin.wheelTravel = state.wheelTravel;
            kin.camberAngle = state.camberAngle;
            kin.toeAngle = state.toeAngle;
            kin.steerAngle = state.steerAngle;
            kin.motionRatio = state.motionRatioEffective;
            kin.xPosition = state.xPosition;
            kin.yPosition = state.yPosition;
            kin.wheelCenterXPosition = state.wheelCenterXPosition;
            kin.wheelCenterYPosition = state.wheelCenterYPosition;
            kin.kingpinXPosition = state.kingpinXPosition;
            kin.kingpinYPosition = state.kingpinYPosition;
            kin.casterAngle = state.casterAngle;
            kin.kingpinInclination = state.kingpinInclination;
            kin.mechanicalTrail = state.mechanicalTrail;
            kin.scrubRadius = state.scrubRadius;
            kin.kingpinOffset = state.kingpinOffset;
            kin.rollCenterHeight = state.rollCenterHeight;
            kin.rollCenterLateral = state.rollCenterLateral;
        end
    end

    methods (Access = private)
        function loads = applyChassisGeometricTransfer(obj, loads, chassis)
            geometricTransfer = obj.readChassisGeometricTransfer(chassis);
            additionalTransfer = obj.readChassisAdditionalTransfer(chassis);

            % Geometric transfer is an internal left/right redistribution.
            % Bound it by the unloading wheel's available load so wheel lift
            % cannot create net vertical force at an axle.
            [loads.FL, loads.FR] = obj.transferAxleLoad( ...
                loads.FL, loads.FR, geometricTransfer.front);
            [loads.RL, loads.RR] = obj.transferAxleLoad( ...
                loads.RL, loads.RR, geometricTransfer.rear);
            [loads.FL, loads.FR] = obj.transferAxleLoad( ...
                loads.FL, loads.FR, additionalTransfer.frontLateral);
            [loads.RL, loads.RR] = obj.transferAxleLoad( ...
                loads.RL, loads.RR, additionalTransfer.rearLateral);

            % Positive longitudinal transfer unloads the front and loads the
            % rear. Apply half at each side while preserving nonnegative loads.
            [loads.FL, loads.RL] = obj.transferLongitudinalLoad( ...
                loads.FL, loads.RL, additionalTransfer.longitudinal / 2);
            [loads.FR, loads.RR] = obj.transferLongitudinalLoad( ...
                loads.FR, loads.RR, additionalTransfer.longitudinal / 2);

            obj.frontLeft.state.tireNormalForce = loads.FL;
            obj.frontRight.state.tireNormalForce = loads.FR;
            obj.rearLeft.state.tireNormalForce = loads.RL;
            obj.rearRight.state.tireNormalForce = loads.RR;
        end

        function [leftLoad, rightLoad] = transferAxleLoad(~, leftLoad, rightLoad, requestedTransfer)
            leftLoad = max(leftLoad, 0);
            rightLoad = max(rightLoad, 0);
            actualTransfer = lts.util.clamp( ...
                requestedTransfer, -rightLoad, leftLoad);
            leftLoad = leftLoad - actualTransfer;
            rightLoad = rightLoad + actualTransfer;
        end

        function [frontLoad, rearLoad] = transferLongitudinalLoad(~, ...
                frontLoad, rearLoad, requestedTransfer)
            frontLoad = max(frontLoad, 0);
            rearLoad = max(rearLoad, 0);
            actualTransfer = lts.util.clamp( ...
                requestedTransfer, -rearLoad, frontLoad);
            frontLoad = frontLoad - actualTransfer;
            rearLoad = rearLoad + actualTransfer;
        end

        function transfer = readChassisGeometricTransfer(~, chassis)
            transfer = struct('front', 0, 'rear', 0);
            if isempty(chassis) || ~isprop(chassis, 'state') || isempty(chassis.state)
                return;
            end

            state = chassis.state;
            if isprop(state, 'frontGeometricLateralLoadTransfer')
                transfer.front = state.frontGeometricLateralLoadTransfer;
            end
            if isprop(state, 'rearGeometricLateralLoadTransfer')
                transfer.rear = state.rearGeometricLateralLoadTransfer;
            end
            if ~isfinite(transfer.front)
                transfer.front = 0;
            end
            if ~isfinite(transfer.rear)
                transfer.rear = 0;
            end
        end

        function transfer = readChassisAdditionalTransfer(~, chassis)
            transfer = struct('longitudinal', 0, ...
                'frontLateral', 0, 'rearLateral', 0);
            if isempty(chassis) || ~isprop(chassis, 'state') || isempty(chassis.state)
                return;
            end

            state = chassis.state;
            if isprop(state, 'additionalLongitudinalLoadTransfer')
                transfer.longitudinal = state.additionalLongitudinalLoadTransfer;
            end
            if isprop(state, 'frontAdditionalLateralLoadTransfer')
                transfer.frontLateral = state.frontAdditionalLateralLoadTransfer;
            end
            if isprop(state, 'rearAdditionalLateralLoadTransfer')
                transfer.rearLateral = state.rearAdditionalLateralLoadTransfer;
            end
            names = fieldnames(transfer);
            for idx = 1:numel(names)
                if ~isfinite(transfer.(names{idx}))
                    transfer.(names{idx}) = 0;
                end
            end
        end

        function bar = makeWheelRateBar(~, wheelRate)
            % MAKEWHEELRATEBAR Build a unit-ratio ARB whose differential
            % wheel-coupling rate equals the requested value [N/m].
            wheelRate = max(0, wheelRate);
            if wheelRate <= 0
                bar = lts.components.Suspension.AntiRollBar();
            else
                bar = lts.components.Suspension.AntiRollBar(wheelRate, 1, 1, true);
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
            % damperPosition is the historical telemetry name for the
            % wheel-domain relative suspension displacement. The ARB coupling
            % rate is already referenced to wheel travel, so no motion-ratio
            % conversion belongs here.
            wheelTravel = cornerState.damperPosition;
            if ~isfinite(wheelTravel)
                wheelTravel = 0;
            end
        end

        function rollAngle = readChassisRollAngle(obj, cornerName)
            rollAngle = 0;
            if isempty(obj.chassis) || ~isprop(obj.chassis, 'state') || ...
                    isempty(obj.chassis.state)
                return;
            end

            state = obj.chassis.state;
            hasSplitRoll = isprop(state, 'frontRollAngle') && ...
                isprop(state, 'rearRollAngle');
            if hasSplitRoll
                if startsWith(upper(cornerName), 'F')
                    rollAngle = state.frontRollAngle;
                else
                    rollAngle = state.rearRollAngle;
                end

                % Preserve compatibility with initializers that populate
                % only the legacy whole-car roll state.
                if state.frontRollAngle == 0 && state.rearRollAngle == 0 && ...
                        isprop(state, 'rollAngle') && state.rollAngle ~= 0
                    rollAngle = state.rollAngle;
                end
            elseif isprop(state, 'rollAngle')
                rollAngle = state.rollAngle;
            end

            if ~isfinite(rollAngle)
                rollAngle = 0;
            end
        end

        function camber = applyChassisRollCamber(~, camber, wheelHeading, ...
                cornerName, rollAngle)
            if rollAngle == 0 || ~isfinite(camber) || ~isfinite(wheelHeading)
                return;
            end

            if endsWith(upper(cornerName), 'L')
                side = 1;
            else
                side = -1;
            end

            % Reconstruct the wheel-top vector from tire-facing camber,
            % rotate it by positive right-side-down chassis roll about +x,
            % then measure its outward lean against the road vertical.
            outward = side * [-sin(wheelHeading), cos(wheelHeading), 0];
            top = outward * sin(camber) + [0, 0, cos(camber)];
            c = cos(rollAngle);
            s = sin(rollAngle);
            topRoad = [top(1), c * top(2) - s * top(3), ...
                s * top(2) + c * top(3)];
            camber = atan2(dot(topRoad, outward), topRoad(3));
        end
    end
end
