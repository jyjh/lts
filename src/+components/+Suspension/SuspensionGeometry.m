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
            kin.wheelTravel = wheelTravel;
            kin.camberAngle = obj.interpolateCurve(travelGrid, camberCurve, wheelTravel);
            kin.toeAngle = side * obj.interpolateCurve(travelGrid, toeCurve, wheelTravel);
            kin.steerAngle = wheelSteer;
            kin.motionRatio = obj.interpolateCurve(travelGrid, motionRatioCurve, wheelTravel);
            [kin.xPosition, kin.yPosition] = obj.computeWheelPosition(corner);
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

            r = geometryCfg.rear;
            obj.rearTravelGrid       = r.travelGrid;
            obj.rearCamberCurve      = r.camberCurve;
            obj.rearToeCurve         = r.toeCurve;
            obj.rearMotionRatioCurve = r.motionRatioCurve;
            obj.rearRollCenterHeight = r.rollCenterHeight;

            s = geometryCfg.steering;
            obj.steeringRatio      = s.steeringRatio;
            obj.ackermann          = s.ackermann;
            obj.maxWheelSteerAngle = s.maxWheelSteerAngle;
            obj.rearSteerRatio     = s.rearSteerRatio;
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
