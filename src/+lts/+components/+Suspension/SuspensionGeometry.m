classdef SuspensionGeometry
    % SUSPENSIONGEOMETRY Table-based suspension and steering kinematics
    %
    % This model maps wheel travel and steering input to the per-corner
    % geometry consumed by the tire model. Curves are intentionally simple
    % tables so different suspension concepts can be compared without a
    % full hardpoint solver.

    properties
        % Vehicle geometry: pulled from lts.vehicle.VehicleManager at construction
        % (no defaults — a bare instance must not be used for kinematics).
        wheelbase
        trackWidth
        staticFrontWeight

        % Front axle geometry curves, indexed by wheel travel [m].
        % Positive wheel travel is bump/compression.
        frontTravelGrid = [-0.05 0 0.05]
        frontCamberCurve = [0 0 0]      % [rad], positive top-out
        frontToeCurve = [0 0 0]         % [rad], positive toe-left
        frontMotionRatioCurve = [1 1 1]

        % Rear axle geometry curves, indexed by wheel travel [m].
        rearTravelGrid = [-0.05 0 0.05]
        rearCamberCurve = [0 0 0]       % [rad], positive top-out
        rearToeCurve = [0 0 0]          % [rad], positive toe-left
        rearMotionRatioCurve = [1 1 1]

        % Steering-axis geometry per axle. Caster is positive when the top
        % of the steering axis leans rearward. Kingpin inclination is
        % positive when the top of the axis leans inward. Mechanical trail
        % is positive when the contact patch sits behind the kingpin ground
        % intercept; scrub radius is positive outboard of that intercept.
        frontCasterAngle = 0             % [rad]
        frontMechanicalTrail = 0         % [m]
        frontScrubRadius = 0             % [m]
        frontKingpinInclination = 0      % [rad]
        frontKingpinOffset = 0           % [m], alias/fallback for scrub radius

        rearCasterAngle = 0              % [rad]
        rearMechanicalTrail = 0          % [m]
        rearScrubRadius = 0              % [m]
        rearKingpinInclination = 0       % [rad]
        rearKingpinOffset = 0            % [m], alias/fallback for scrub radius

        % Steering model. steerInput is treated as road-wheel angle by
        % default to preserve current lts.driver.DriverModel behavior.
        steeringRatio = 1.0
        ackermann = 0.0                 % 0 = parallel, 1 = ideal, >1 = over-Ackermann
        maxWheelSteerAngle = 0.6        % [rad]
        rearSteerRatio = 0.0

        % Roll-center position per axle. Height [m] above the ground plane
        % drives geometric lateral load transfer. Lateral [m] is retained
        % from +1g kinematic outputs for inspection and dynamics inputs.
        % A zero height collapses the lateral-transfer split to the legacy
        % CG-height-only behavior.
        frontRollCenterHeight = 0
        rearRollCenterHeight = 0
        frontRollCenterLateral = 0
        rearRollCenterLateral = 0

        % Anti-roll bars per axle. Empty/disabled => the axle's roll
        % stiffness is its wheel springs only.
        frontAntiRollBar = []
        rearAntiRollBar  = []
    end

    methods
        function obj = SuspensionGeometry(vehicleManager)
            if nargin >= 1 && ~isempty(vehicleManager)
                obj.wheelbase = vehicleManager.wheelbase;
                obj.trackWidth = vehicleManager.trackWidth;
                obj.staticFrontWeight = vehicleManager.staticFrontWeight;
            end
        end

        function kin = computeCornerKinematics(obj, corner, wheelTravel, steerInput)
            % COMPUTECORNERKINEMATICS Return tire-facing geometry for a corner.
            %
            % Functionality: interpolate suspension tables at the current
            % wheel travel, resolve the road-wheel steer angle, and return
            % camber/toe/motion ratio/contact-patch position for the tire.
            %
            % Physics: camber and toe change the tire's local force axes and
            % Magic Formula inputs; trail/scrub move the contact patch relative
            % to the wheel center, changing the local velocity and yaw moment.
            cornerKey = upper(char(corner));
            isFront = cornerKey(1) == 'F';
            isLeft = cornerKey(end) == 'L';
            if isLeft
                side = 1;
            else
                side = -1;
            end

            if isFront
                travelGrid = obj.frontTravelGrid;
                camberCurve = obj.frontCamberCurve;
                toeCurve = obj.frontToeCurve;
                motionRatioCurve = obj.frontMotionRatioCurve;
                caster = obj.frontCasterAngle;
                trail = obj.frontMechanicalTrail;
                scrub = obj.frontScrubRadius;
                kpi = obj.frontKingpinInclination;
                kingpinOffset = obj.frontKingpinOffset;
                rollCenterHeight = obj.frontRollCenterHeight;
                rollCenterLateral = obj.frontRollCenterLateral;
            else
                travelGrid = obj.rearTravelGrid;
                camberCurve = obj.rearCamberCurve;
                toeCurve = obj.rearToeCurve;
                motionRatioCurve = obj.rearMotionRatioCurve;
                caster = obj.rearCasterAngle;
                trail = obj.rearMechanicalTrail;
                scrub = obj.rearScrubRadius;
                kpi = obj.rearKingpinInclination;
                kingpinOffset = obj.rearKingpinOffset;
                rollCenterHeight = obj.rearRollCenterHeight;
                rollCenterLateral = obj.rearRollCenterLateral;
            end

            wheelSteer = obj.computeWheelSteerFast(isFront, isLeft, steerInput);
            baseCamber = obj.interpolateCurve(travelGrid, camberCurve, wheelTravel);
            toeAngle = side * obj.interpolateCurve(travelGrid, toeCurve, wheelTravel);
            wheelHeading = wheelSteer + toeAngle;
            axis = [-sin(caster), -side * sin(kpi), cos(caster) * cos(kpi)];
            normAxis = norm(axis);
            if normAxis <= eps
                axis = [0, 0, 1];
            else
                axis = axis ./ normAxis;
            end

            if abs(scrub) > eps || abs(kingpinOffset) <= eps
                effectiveScrub = scrub;
            else
                effectiveScrub = kingpinOffset;
            end

            if isFront
                wheelCenterX = obj.wheelbase * (1 - obj.staticFrontWeight);
            else
                wheelCenterX = -obj.wheelbase * obj.staticFrontWeight;
            end
            wheelCenterY = side * obj.trackWidth / 2;

            [contactX, contactY, kingpinX, kingpinY] = ...
                obj.computeContactPatchPositionFast( ...
                    trail, effectiveScrub, side, wheelHeading, ...
                    wheelCenterX, wheelCenterY);

            kin.wheelTravel = wheelTravel;
            kin.baseCamberAngle = baseCamber;
            kin.camberAngle = obj.applySteeringAxisCamberFast( ...
                baseCamber, wheelHeading, axis, side);
            kin.toeAngle = toeAngle;
            kin.steerAngle = wheelSteer;
            kin.motionRatio = obj.interpolateCurve(travelGrid, motionRatioCurve, wheelTravel);
            kin.wheelCenterXPosition = wheelCenterX;
            kin.wheelCenterYPosition = wheelCenterY;
            kin.xPosition = contactX;
            kin.yPosition = contactY;
            kin.kingpinXPosition = kingpinX;
            kin.kingpinYPosition = kingpinY;
            kin.casterAngle = caster;
            kin.mechanicalTrail = trail;
            kin.scrubRadius = effectiveScrub;
            kin.kingpinInclination = kpi;
            kin.kingpinOffset = kingpinOffset;
            kin.rollCenterHeight = rollCenterHeight;
            kin.rollCenterLateral = rollCenterLateral;
        end
    end

    methods (Access = private)
        function wheelSteer = computeWheelSteer(obj, corner, steerInput)
            % COMPUTEWHEELSTEER Convert driver road-wheel command to corner angle.
            % Front steering blends parallel steer with ideal Ackermann:
            % the inside wheel steers more than the outside wheel so both
            % wheels point toward a common low-speed turn center.
            axle = lts.components.Suspension.SuspensionGeometry.getAxle(corner);
            if strcmp(axle, 'rear')
                wheelSteer = obj.rearSteerRatio * steerInput;
                wheelSteer = lts.util.clamp(wheelSteer, -obj.maxWheelSteerAngle, obj.maxWheelSteerAngle);
                return;
            end

            meanSteer = steerInput / max(obj.steeringRatio, eps);
            meanSteer = lts.util.clamp(meanSteer, -obj.maxWheelSteerAngle, obj.maxWheelSteerAngle);
            if abs(meanSteer) < eps || obj.ackermann <= 0
                wheelSteer = meanSteer;
                return;
            end

            turnSign = sign(meanSteer);
            absSteer = abs(meanSteer);
            turnRadius = obj.wheelbase / max(tan(absSteer), eps);
            halfTrack = obj.trackWidth / 2;

            idealInner = atan(obj.wheelbase / max(turnRadius - halfTrack, eps));
            idealOuter = atan(obj.wheelbase / (turnRadius + halfTrack));
            ackermannBlend = max(obj.ackermann, 0);
            if ~isfinite(ackermannBlend)
                ackermannBlend = 0;
            end

            isLeftSide = strcmp(upper(corner), 'FL');
            isInside = (turnSign > 0 && isLeftSide) || (turnSign < 0 && ~isLeftSide);
            if isInside
                target = idealInner;
            else
                target = idealOuter;
            end

            wheelSteer = turnSign * (absSteer + ackermannBlend * (target - absSteer));
            wheelSteer = lts.util.clamp(wheelSteer, -obj.maxWheelSteerAngle, obj.maxWheelSteerAngle);
        end

        function wheelSteer = computeWheelSteerFast(obj, isFront, isLeft, steerInput)
            if ~isFront
                wheelSteer = obj.rearSteerRatio * steerInput;
                wheelSteer = lts.util.clamp(wheelSteer, ...
                    -obj.maxWheelSteerAngle, obj.maxWheelSteerAngle);
                return;
            end

            meanSteer = steerInput / max(obj.steeringRatio, eps);
            meanSteer = lts.util.clamp(meanSteer, ...
                -obj.maxWheelSteerAngle, obj.maxWheelSteerAngle);
            if abs(meanSteer) < eps || obj.ackermann <= 0
                wheelSteer = meanSteer;
                return;
            end

            turnSign = sign(meanSteer);
            absSteer = abs(meanSteer);
            turnRadius = obj.wheelbase / max(tan(absSteer), eps);
            halfTrack = obj.trackWidth / 2;

            idealInner = atan(obj.wheelbase / max(turnRadius - halfTrack, eps));
            idealOuter = atan(obj.wheelbase / (turnRadius + halfTrack));
            ackermannBlend = max(obj.ackermann, 0);
            if ~isfinite(ackermannBlend)
                ackermannBlend = 0;
            end

            isInside = (turnSign > 0 && isLeft) || (turnSign < 0 && ~isLeft);
            if isInside
                target = idealInner;
            else
                target = idealOuter;
            end

            wheelSteer = turnSign * (absSteer + ackermannBlend * (target - absSteer));
            wheelSteer = lts.util.clamp(wheelSteer, ...
                -obj.maxWheelSteerAngle, obj.maxWheelSteerAngle);
        end

        function camber = applySteeringAxisCamberFast(~, baseCamber, wheelHeading, axis, side)
            topVector = [0, side * sin(baseCamber), cos(baseCamber)];
            c = cos(wheelHeading);
            s = sin(wheelHeading);
            crossAxisTop = [ ...
                axis(2) * topVector(3) - axis(3) * topVector(2), ...
                axis(3) * topVector(1) - axis(1) * topVector(3), ...
                axis(1) * topVector(2) - axis(2) * topVector(1)];
            axisDotTop = axis(1) * topVector(1) + ...
                axis(2) * topVector(2) + axis(3) * topVector(3);
            topVector = topVector * c + crossAxisTop * s + ...
                axis * axisDotTop * (1 - c);

            outward = side * [-sin(wheelHeading), cos(wheelHeading), 0];
            camber = atan2( ...
                topVector(1) * outward(1) + topVector(2) * outward(2), ...
                topVector(3));
        end

        function [x, y, kingpinX, kingpinY] = computeContactPatchPositionFast( ...
                ~, trail, scrub, side, wheelHeading, baseX, baseY)
            offset0 = [-trail, side * scrub];
            forward = [cos(wheelHeading), sin(wheelHeading)];
            left = [-sin(wheelHeading), cos(wheelHeading)];
            offset = -trail * forward + side * scrub * left;

            kingpinX = baseX - offset0(1);
            kingpinY = baseY - offset0(2);
            x = kingpinX + offset(1);
            y = kingpinY + offset(2);
        end
    end

    methods (Static)
        function obj = fromConfig(geometryCfg, vehicleManager)
            % FROMCONFIG Build a SuspensionGeometry from a config struct.
            %   SuspensionGeometry.fromConfig(geometryCfg, vehicleManager)
            %
            %   geometryCfg    - suspension geometry config struct (see
            %                    lts.vehicle.VehicleConfig.suspension.geometry) with
            %                    nested .front, .rear, and .steering fields.
            %   vehicleManager - lts.vehicle.VehicleManager (geometry pulled at construction)
            obj = lts.components.Suspension.SuspensionGeometry(vehicleManager);

            f = geometryCfg.front;
            obj.frontTravelGrid       = f.travelGrid;
            obj.frontCamberCurve      = f.camberCurve;
            obj.frontToeCurve         = f.toeCurve;
            obj.frontMotionRatioCurve = f.motionRatioCurve;
            obj.frontRollCenterHeight = f.rollCenterHeight;
            obj.frontRollCenterLateral = lts.components.Suspension.SuspensionGeometry.readConfigField( ...
                f, {'rollCenterLateral', 'rollCenterY'}, 0);
            obj.frontCasterAngle = lts.components.Suspension.SuspensionGeometry.readConfigField( ...
                f, {'casterAngle', 'caster'}, 0);
            obj.frontMechanicalTrail = lts.components.Suspension.SuspensionGeometry.readConfigField( ...
                f, {'mechanicalTrail', 'trail'}, 0);
            obj.frontScrubRadius = lts.components.Suspension.SuspensionGeometry.readConfigField( ...
                f, {'scrubRadius', 'scrub'}, 0);
            obj.frontKingpinInclination = lts.components.Suspension.SuspensionGeometry.readConfigField( ...
                f, {'kingpinInclination', 'kingpinInclinationAngle', 'kpi'}, 0);
            obj.frontKingpinOffset = lts.components.Suspension.SuspensionGeometry.readConfigField( ...
                f, {'kingpinOffset'}, obj.frontScrubRadius);

            r = geometryCfg.rear;
            obj.rearTravelGrid       = r.travelGrid;
            obj.rearCamberCurve      = r.camberCurve;
            obj.rearToeCurve         = r.toeCurve;
            obj.rearMotionRatioCurve = r.motionRatioCurve;
            obj.rearRollCenterHeight = r.rollCenterHeight;
            obj.rearRollCenterLateral = lts.components.Suspension.SuspensionGeometry.readConfigField( ...
                r, {'rollCenterLateral', 'rollCenterY'}, 0);
            obj.rearCasterAngle = lts.components.Suspension.SuspensionGeometry.readConfigField( ...
                r, {'casterAngle', 'caster'}, 0);
            obj.rearMechanicalTrail = lts.components.Suspension.SuspensionGeometry.readConfigField( ...
                r, {'mechanicalTrail', 'trail'}, 0);
            obj.rearScrubRadius = lts.components.Suspension.SuspensionGeometry.readConfigField( ...
                r, {'scrubRadius', 'scrub'}, 0);
            obj.rearKingpinInclination = lts.components.Suspension.SuspensionGeometry.readConfigField( ...
                r, {'kingpinInclination', 'kingpinInclinationAngle', 'kpi'}, 0);
            obj.rearKingpinOffset = lts.components.Suspension.SuspensionGeometry.readConfigField( ...
                r, {'kingpinOffset'}, obj.rearScrubRadius);

            s = geometryCfg.steering;
            obj.steeringRatio      = s.steeringRatio;
            obj.ackermann          = s.ackermann;
            obj.maxWheelSteerAngle = s.maxWheelSteerAngle;
            obj.rearSteerRatio     = s.rearSteerRatio;
        end

        function value = readConfigField(s, names, defaultValue)
            value = defaultValue;
            for i = 1:numel(names)
                fieldName = names{i};
                if isfield(s, fieldName) && ~isempty(s.(fieldName))
                    value = s.(fieldName);
                    return;
                end
            end
        end

        function value = interpolateCurve(grid, curve, query)
            if isempty(grid) || isempty(curve)
                value = 0;
                return;
            end
            grid = grid(:);
            curve = curve(:);
            n = min(numel(grid), numel(curve));
            if n <= 0
                value = 0;
                return;
            elseif n == 1
                value = curve(1);
                return;
            end

            % Scalar linear interpolation with extrapolation. This is called
            % for every corner every step, and avoids interp1's setup cost.
            grid = grid(1:n);
            curve = curve(1:n);
            if query <= grid(1)
                idx0 = 1;
            elseif query >= grid(end)
                idx0 = n - 1;
            else
                idx0 = find(grid <= query, 1, 'last');
                idx0 = min(max(idx0, 1), n - 1);
            end

            dx = grid(idx0 + 1) - grid(idx0);
            if abs(dx) <= eps || ~isfinite(dx)
                value = curve(idx0);
                return;
            end
            frac = (query - grid(idx0)) / dx;
            value = curve(idx0) + frac * (curve(idx0 + 1) - curve(idx0));
        end

        function axle = getAxle(corner)
            if startsWith(upper(corner), 'F')
                axle = 'front';
            else
                axle = 'rear';
            end
        end

    end
end
