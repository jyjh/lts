classdef VehicleState
    % Vehicle dynamics state and reference telemetry.

    properties
        vehicleManager

        s = 0
        x = NaN
        y = NaN
        yaw = NaN
        speed = 0
        vx = NaN
        vy = 0
        bodySlipAngle = 0

        ax = 0
        ay = 0
        frontAxleAy = NaN
        rearAxleAy = NaN
        heading = 0
        yawRate = 0
        yawAccel = 0

        pitchAngle = 0
        rollAngle = 0
        rollRate = 0
        frontRollAngle = 0
        rearRollAngle = 0
        frontRollRate = 0
        rearRollRate = 0
        twistAngle = 0
        twistRate = 0
        rideHeight = 0

        throttle = 0
        brake = 0
        steer = 0

        curvature = 0
        refS = 0
        refHeading = 0
        refCurvature = 0
        lateralError = 0
        mu = 1
        time = 0
        onTrack = true
    end

    methods
        function obj = VehicleState(varargin)
            specifiedYaw = false;
            specifiedHeading = false;
            for i = 1:2:numel(varargin)
                name = varargin{i};
                if isprop(obj, name)
                    obj.(name) = varargin{i + 1};
                    specifiedYaw = specifiedYaw || strcmp(name, 'yaw');
                    specifiedHeading = specifiedHeading || strcmp(name, 'heading');
                end
            end

            if isnan(obj.vx)
                obj.vx = obj.speed;
            elseif obj.speed <= 0
                obj.speed = hypot(obj.vx, obj.vy);
            end
            if specifiedYaw
                obj.heading = obj.yaw;
            elseif specifiedHeading
                obj.yaw = obj.heading;
            end
            obj.mu = 1;
            obj.bodySlipAngle = atan2(obj.vy, obj.vx);
        end

        function obj = updateFromPlanarDynamics(obj, ax, ay, yawAccel, ...
                vx, vy, yawRate, yaw, x, y, refS, refHeading, refCurvature, ...
                lateralError, dt, frontAxleAy, rearAxleAy)
            if nargin < 16 || isempty(frontAxleAy)
                frontAxleAy = ay;
            end
            if nargin < 17 || isempty(rearAxleAy)
                rearAxleAy = ay;
            end

            obj.ax = ax;
            obj.ay = ay;
            obj.yawAccel = yawAccel;
            obj.frontAxleAy = frontAxleAy;
            obj.rearAxleAy = rearAxleAy;
            obj.vx = vx;
            obj.vy = vy;
            obj.speed = hypot(vx, vy);
            obj.bodySlipAngle = atan2(vy, vx);
            obj.yawRate = yawRate;
            obj.yaw = yaw;
            obj.heading = yaw;
            obj.x = x;
            obj.y = y;
            obj.s = refS;
            obj.refS = refS;
            obj.refHeading = refHeading;
            obj.refCurvature = refCurvature;
            obj.curvature = refCurvature;
            obj.lateralError = lateralError;
            obj.mu = 1;

            obj = obj.updateAttitude();
            obj.time = obj.time + dt;
        end

        function obj = updateAttitude(obj)
            vm = obj.vehicleManager;
            if isempty(vm) || isempty(vm.chassis)
                if ~isempty(vm) && ~isempty(vm.suspension) && ...
                        ismethod(vm.suspension, 'computePitchAngle')
                    obj.pitchAngle = vm.suspension.computePitchAngle();
                end
                return;
            end

            chassis = vm.chassis;
            obj.pitchAngle = chassis.getPitchAngle();
            obj.rollAngle = chassis.getRollAngle();
            obj.rideHeight = -chassis.getHeave();

            if ismethod(chassis, 'getRollRate')
                obj.rollRate = chassis.getRollRate();
                obj.frontRollRate = chassis.getFrontRollRate();
                obj.rearRollRate = chassis.getRearRollRate();
            end
            if ismethod(chassis, 'getFrontRollAngle')
                obj.frontRollAngle = chassis.getFrontRollAngle();
                obj.rearRollAngle = chassis.getRearRollAngle();
            else
                obj.frontRollAngle = obj.rollAngle;
                obj.rearRollAngle = obj.rollAngle;
            end
            if ismethod(chassis, 'getTwistAngle')
                obj.twistAngle = chassis.getTwistAngle();
                obj.twistRate = chassis.getTwistRate();
            else
                obj.twistAngle = obj.frontRollAngle - obj.rearRollAngle;
                obj.twistRate = obj.frontRollRate - obj.rearRollRate;
            end
        end
    end
end
