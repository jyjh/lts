classdef SuspensionGeometry
    % SUSPENSIONGEOMETRY Table-based suspension and steering kinematics
    %
    % This model maps wheel travel and steering input to the per-corner
    % geometry consumed by the tire model. Curves are intentionally simple
    % tables so different suspension concepts can be compared without a
    % full hardpoint solver.

    properties
        % Vehicle geometry: pulled from VehicleManager at construction
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
        % default to preserve current DriverModel behavior.
        steeringRatio = 1.0
        ackermann = 0.0                 % 0 = parallel steer, 1 = ideal Ackermann
        maxWheelSteerAngle = 0.6        % [rad]
        rearSteerRatio = 0.0

        % Roll-center height per axle [m] above the ground plane. The roll
        % center is the point through which the lateral (geometric) load
        % transfer acts. 0 collapses the lateral-transfer split to the
        % legacy CG-height-only behavior.
        frontRollCenterHeight = 0
        rearRollCenterHeight = 0

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
            axle = components.Suspension.SuspensionGeometry.getAxle(corner);
            side = components.Suspension.SuspensionGeometry.getSide(corner);

            if strcmp(axle, 'front')
                travelGrid = obj.frontTravelGrid;
                camberCurve = obj.frontCamberCurve;
                toeCurve = obj.frontToeCurve;
                motionRatioCurve = obj.frontMotionRatioCurve;
            else
                travelGrid = obj.rearTravelGrid;
                camberCurve = obj.rearCamberCurve;
                toeCurve = obj.rearToeCurve;
                motionRatioCurve = obj.rearMotionRatioCurve;
            end

            wheelSteer = obj.computeWheelSteer(corner, steerInput);
            baseCamber = obj.interpolateCurve(travelGrid, camberCurve, wheelTravel);
            toeAngle = side * obj.interpolateCurve(travelGrid, toeCurve, wheelTravel);
            wheelHeading = wheelSteer + toeAngle;
            axis = obj.computeSteeringAxis(corner);

            kin.wheelTravel = wheelTravel;
            kin.baseCamberAngle = baseCamber;
            kin.camberAngle = obj.applySteeringAxisCamber( ...
                baseCamber, wheelHeading, axis, side);
            kin.toeAngle = toeAngle;
            kin.steerAngle = wheelSteer;
            kin.motionRatio = obj.interpolateCurve(travelGrid, motionRatioCurve, wheelTravel);
            [kin.wheelCenterXPosition, kin.wheelCenterYPosition] = ...
                obj.computeWheelPosition(corner);
            [kin.xPosition, kin.yPosition, kin.kingpinXPosition, ...
                kin.kingpinYPosition] = obj.computeContactPatchPosition( ...
                corner, wheelHeading, kin.wheelCenterXPosition, ...
                kin.wheelCenterYPosition);
            kin.casterAngle = obj.getAxleValue(axle, 'CasterAngle');
            kin.mechanicalTrail = obj.getAxleValue(axle, 'MechanicalTrail');
            kin.scrubRadius = obj.getEffectiveScrubRadius(axle);
            kin.kingpinInclination = obj.getAxleValue(axle, 'KingpinInclination');
            kin.kingpinOffset = obj.getAxleValue(axle, 'KingpinOffset');
            if strcmp(axle, 'front')
                kin.rollCenterHeight = obj.frontRollCenterHeight;
            else
                kin.rollCenterHeight = obj.rearRollCenterHeight;
            end
        end

        function steer = computeSteeringAngles(obj, steerInput)
            steer.FL = obj.computeWheelSteer('FL', steerInput);
            steer.FR = obj.computeWheelSteer('FR', steerInput);
            steer.RL = obj.computeWheelSteer('RL', steerInput);
            steer.RR = obj.computeWheelSteer('RR', steerInput);
        end
    end

    methods (Access = private)
        function wheelSteer = computeWheelSteer(obj, corner, steerInput)
            axle = components.Suspension.SuspensionGeometry.getAxle(corner);
            if strcmp(axle, 'rear')
                wheelSteer = obj.rearSteerRatio * steerInput;
                wheelSteer = obj.clamp(wheelSteer, -obj.maxWheelSteerAngle, obj.maxWheelSteerAngle);
                return;
            end

            meanSteer = steerInput / max(obj.steeringRatio, eps);
            meanSteer = obj.clamp(meanSteer, -obj.maxWheelSteerAngle, obj.maxWheelSteerAngle);
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
            ackermannBlend = obj.clamp(obj.ackermann, 0, 1);

            isLeftSide = strcmp(upper(corner), 'FL');
            isInside = (turnSign > 0 && isLeftSide) || (turnSign < 0 && ~isLeftSide);
            if isInside
                target = idealInner;
            else
                target = idealOuter;
            end

            wheelSteer = turnSign * (absSteer + ackermannBlend * (target - absSteer));
            wheelSteer = obj.clamp(wheelSteer, -obj.maxWheelSteerAngle, obj.maxWheelSteerAngle);
        end

        function [x, y] = computeWheelPosition(obj, corner)
            frontArm = obj.wheelbase * (1 - obj.staticFrontWeight);
            rearArm = obj.wheelbase * obj.staticFrontWeight;
            halfTrack = obj.trackWidth / 2;

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

        function axis = computeSteeringAxis(obj, corner)
            axle = components.Suspension.SuspensionGeometry.getAxle(corner);
            side = components.Suspension.SuspensionGeometry.getSide(corner);
            caster = obj.getAxleValue(axle, 'CasterAngle');
            kpi = obj.getAxleValue(axle, 'KingpinInclination');

            axis = [-sin(caster), -side * sin(kpi), ...
                cos(caster) * cos(kpi)];
            normAxis = norm(axis);
            if normAxis <= eps
                axis = [0, 0, 1];
            else
                axis = axis ./ normAxis;
            end
        end

        function camber = applySteeringAxisCamber(~, baseCamber, wheelHeading, axis, side)
            topVector = [0, side * sin(baseCamber), cos(baseCamber)];
            topVector = components.Suspension.SuspensionGeometry.rotateVector( ...
                topVector, axis, wheelHeading);

            outward = side * [-sin(wheelHeading), cos(wheelHeading), 0];
            camber = atan2(dot(topVector, outward), dot(topVector, [0, 0, 1]));
        end

        function [x, y, kingpinX, kingpinY] = computeContactPatchPosition( ...
                obj, corner, wheelHeading, baseX, baseY)
            axle = components.Suspension.SuspensionGeometry.getAxle(corner);
            side = components.Suspension.SuspensionGeometry.getSide(corner);
            trail = obj.getAxleValue(axle, 'MechanicalTrail');
            scrub = obj.getEffectiveScrubRadius(axle);

            offset0 = [-trail, side * scrub];
            forward = [cos(wheelHeading), sin(wheelHeading)];
            left = [-sin(wheelHeading), cos(wheelHeading)];
            offset = -trail * forward + side * scrub * left;

            kingpinX = baseX - offset0(1);
            kingpinY = baseY - offset0(2);
            x = kingpinX + offset(1);
            y = kingpinY + offset(2);
        end

        function value = getEffectiveScrubRadius(obj, axle)
            scrub = obj.getAxleValue(axle, 'ScrubRadius');
            offset = obj.getAxleValue(axle, 'KingpinOffset');
            if abs(scrub) > eps || abs(offset) <= eps
                value = scrub;
            else
                value = offset;
            end
        end

        function value = getAxleValue(obj, axle, suffix)
            fieldName = [axle suffix];
            value = obj.(fieldName);
        end
    end

    methods (Static)
        function obj = fromConfig(geometryCfg, vehicleManager)
            % FROMCONFIG Build a SuspensionGeometry from a config struct.
            %   SuspensionGeometry.fromConfig(geometryCfg, vehicleManager)
            %
            %   geometryCfg    - suspension geometry config struct (see
            %                    VehicleConfig.suspension.geometry) with
            %                    nested .front, .rear, and .steering fields.
            %   vehicleManager - VehicleManager (geometry pulled at construction)
            obj = components.Suspension.SuspensionGeometry(vehicleManager);

            f = geometryCfg.front;
            obj.frontTravelGrid       = f.travelGrid;
            obj.frontCamberCurve      = f.camberCurve;
            obj.frontToeCurve         = f.toeCurve;
            obj.frontMotionRatioCurve = f.motionRatioCurve;
            obj.frontRollCenterHeight = f.rollCenterHeight;
            obj.frontCasterAngle = components.Suspension.SuspensionGeometry.readConfigField( ...
                f, {'casterAngle', 'caster'}, 0);
            obj.frontMechanicalTrail = components.Suspension.SuspensionGeometry.readConfigField( ...
                f, {'mechanicalTrail', 'trail'}, 0);
            obj.frontScrubRadius = components.Suspension.SuspensionGeometry.readConfigField( ...
                f, {'scrubRadius', 'scrub'}, 0);
            obj.frontKingpinInclination = components.Suspension.SuspensionGeometry.readConfigField( ...
                f, {'kingpinInclination', 'kingpinInclinationAngle', 'kpi'}, 0);
            obj.frontKingpinOffset = components.Suspension.SuspensionGeometry.readConfigField( ...
                f, {'kingpinOffset'}, obj.frontScrubRadius);

            r = geometryCfg.rear;
            obj.rearTravelGrid       = r.travelGrid;
            obj.rearCamberCurve      = r.camberCurve;
            obj.rearToeCurve         = r.toeCurve;
            obj.rearMotionRatioCurve = r.motionRatioCurve;
            obj.rearRollCenterHeight = r.rollCenterHeight;
            obj.rearCasterAngle = components.Suspension.SuspensionGeometry.readConfigField( ...
                r, {'casterAngle', 'caster'}, 0);
            obj.rearMechanicalTrail = components.Suspension.SuspensionGeometry.readConfigField( ...
                r, {'mechanicalTrail', 'trail'}, 0);
            obj.rearScrubRadius = components.Suspension.SuspensionGeometry.readConfigField( ...
                r, {'scrubRadius', 'scrub'}, 0);
            obj.rearKingpinInclination = components.Suspension.SuspensionGeometry.readConfigField( ...
                r, {'kingpinInclination', 'kingpinInclinationAngle', 'kpi'}, 0);
            obj.rearKingpinOffset = components.Suspension.SuspensionGeometry.readConfigField( ...
                r, {'kingpinOffset'}, obj.rearScrubRadius);

            s = geometryCfg.steering;
            obj.steeringRatio      = s.steeringRatio;
            obj.ackermann          = s.ackermann;
            obj.maxWheelSteerAngle = s.maxWheelSteerAngle;
            obj.rearSteerRatio     = s.rearSteerRatio;
        end

        function rotated = rotateVector(vector, axis, angle)
            axis = axis ./ max(norm(axis), eps);
            rotated = vector * cos(angle) + cross(axis, vector) * sin(angle) + ...
                axis * dot(axis, vector) * (1 - cos(angle));
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
            value = interp1(grid(:), curve(:), query, 'linear', 'extrap');
        end

        function axle = getAxle(corner)
            if startsWith(upper(corner), 'F')
                axle = 'front';
            else
                axle = 'rear';
            end
        end

        function side = getSide(corner)
            if endsWith(upper(corner), 'L')
                side = 1;
            else
                side = -1;
            end
        end

        function value = clamp(value, lower, upper)
            value = max(lower, min(upper, value));
        end
    end
end
