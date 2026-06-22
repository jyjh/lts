classdef DriverInputPlanner
    % DRIVERINPUTPLANNER Builds open-loop controls from track reference data.
    %
    % The planner treats the track centerline as a reference for estimating
    % throttle, brake, and steering commands. It does not perform path
    % tracking and does not constrain vehicle motion to the centerline.

    properties
        vehicleManager
        driverModel
        maxSteeringAngle = 0.6
        maxDriveAccel = 5.0
        speedFeedbackDeadband = 0.20
        speedFeedbackThrottleBand = 1.0
        speedFeedbackBrakeBand = 1.0
    end

    methods
        function obj = DriverInputPlanner(vehicleManager, driverModelOrMaxSteer)
            obj.vehicleManager = vehicleManager;
            if nargin >= 2
                if isnumeric(driverModelOrMaxSteer)
                    obj.maxSteeringAngle = driverModelOrMaxSteer;
                    obj.driverModel = [];
                else
                    obj.driverModel = driverModelOrMaxSteer;
                    if isprop(driverModelOrMaxSteer, 'maxSteeringAngle')
                        obj.maxSteeringAngle = driverModelOrMaxSteer.maxSteeringAngle;
                    end
                    if isprop(driverModelOrMaxSteer, 'throttleBand')
                        obj.speedFeedbackDeadband = max( ...
                            obj.speedFeedbackDeadband, ...
                            driverModelOrMaxSteer.throttleBand);
                    end
                    if isprop(driverModelOrMaxSteer, 'brakeBlendSpeed')
                        blendSpeed = driverModelOrMaxSteer.brakeBlendSpeed;
                        if isfinite(blendSpeed) && blendSpeed > 0
                            obj.speedFeedbackBrakeBand = blendSpeed;
                        end
                    end
                end
            else
                obj.driverModel = [];
            end
        end

        function profile = buildOpenLoopProfile(obj, initialState, trackData)
            vm = obj.vehicleManager;
            n = trackData.nPts;
            curvature = trackData.curvature(:);
            mu = trackData.mu(:);
            vTarget = vm.maxSpeed * ones(n, 1);

            % Iterating lets speed-dependent aero influence the GGV envelope
            % without turning this into a full trajectory optimization.
            for iter = 1:3 %#ok<NASGU>
                for i = 1:n
                    if abs(curvature(i)) > 1e-6
                        limits = obj.estimateGGVLimits(vTarget(i), mu(i), initialState);
                        vTarget(i) = min(vm.maxSpeed, ...
                            sqrt(max(limits.maxLatAccel, 0.1) / abs(curvature(i))));
                    else
                        vTarget(i) = vm.maxSpeed;
                    end
                end
            end

            maxBrakeAccel = zeros(n, 1);
            for i = 1:n
                limits = obj.estimateGGVLimits(vTarget(i), mu(i), initialState);
                maxBrakeAccel(i) = limits.maxBrakeAccel;
            end

            for i = n-1:-1:1
                ds = max(trackData.arcLen(i+1) - trackData.arcLen(i), 0.001);
                reachableSpeed = sqrt(vTarget(i+1)^2 + 2 * maxBrakeAccel(i+1) * ds);
                vTarget(i) = min(vTarget(i), reachableSpeed);
            end

            speedPlan = vTarget;
            speedPlan(1) = min(max(initialState.speed, 0), vTarget(1));
            maxDriveAccel = obj.maxDriveAccel;
            for i = 1:n-1
                ds = max(trackData.arcLen(i+1) - trackData.arcLen(i), 0.001);
                reachableSpeed = sqrt(speedPlan(i)^2 + 2 * maxDriveAccel * ds);
                speedPlan(i+1) = min(vTarget(i+1), reachableSpeed);
            end

            axRef = zeros(n, 1);
            for i = 1:n-1
                ds = max(trackData.arcLen(i+1) - trackData.arcLen(i), 0.001);
                axRef(i) = (speedPlan(i+1)^2 - speedPlan(i)^2) / (2 * ds);
            end
            axRef(n) = axRef(max(n-1, 1));

            maxSteer = obj.maxSteeringAngle;
            steerRef = atan(vm.wheelbase * curvature);
            steerRef = max(-maxSteer, min(maxSteer, steerRef));

            brakeRef = zeros(n, 1);
            throttleRef = zeros(n, 1);
            for i = 1:n
                if axRef(i) < 0
                    brakeRef(i) = min(1, -axRef(i) / max(maxBrakeAccel(i), eps));
                elseif axRef(i) > 0
                    throttleRef(i) = min(1, axRef(i) / max(maxDriveAccel, eps));
                end
            end

            profile = struct( ...
                's', trackData.arcLen, ...
                'vTarget', speedPlan, ...
                'vLimit', vTarget, ...
                'axRef', axRef, ...
                'throttle', throttleRef, ...
                'brake', brakeRef, ...
                'steer', steerRef);
        end

        function input = sample(obj, profile, idx, actualSpeed)
            idx = max(1, min(idx, numel(profile.throttle)));
            input = struct( ...
                'throttle', profile.throttle(idx), ...
                'brake', profile.brake(idx), ...
                'steer', profile.steer(idx), ...
                'targetSpeed', profile.vTarget(idx), ...
                'axRef', profile.axRef(idx));

            if nargin >= 4 && isfinite(actualSpeed)
                input = obj.applySpeedFeedback(input, actualSpeed);
            end
        end

        function input = sampleAtProgress(obj, profile, s, actualSpeed)
            sProfile = profile.s(:);
            s = max(sProfile(1), min(sProfile(end), s));

            input = struct( ...
                'throttle', interp1(sProfile, profile.throttle(:), s, 'linear'), ...
                'brake', interp1(sProfile, profile.brake(:), s, 'linear'), ...
                'steer', interp1(sProfile, profile.steer(:), s, 'linear'), ...
                'targetSpeed', interp1(sProfile, profile.vTarget(:), s, 'linear'), ...
                'axRef', interp1(sProfile, profile.axRef(:), s, 'linear'));

            input.throttle = max(0, min(1, input.throttle));
            input.brake = max(0, min(1, input.brake));

            if nargin >= 4 && isfinite(actualSpeed)
                input = obj.applySpeedFeedback(input, actualSpeed);
            end
        end
    end

    methods (Access = private)
        function input = applySpeedFeedback(obj, input, actualSpeed)
            if ~isfield(input, 'targetSpeed') || ~isfinite(input.targetSpeed)
                return;
            end
            if ~isfield(input, 'axRef') || ~isfinite(input.axRef)
                input.axRef = 0;
            end

            speedError = actualSpeed - input.targetSpeed;
            deadband = max(0, obj.speedFeedbackDeadband);
            if actualSpeed < 0.5 && speedError <= 0
                input.brake = 0;
                input.throttle = 1;
            elseif speedError < -deadband
                input.brake = 0;
                throttleCorrection = (-speedError - deadband) / ...
                    max(obj.speedFeedbackThrottleBand, eps);
                input.throttle = max(input.throttle, min(1, throttleCorrection));
            elseif speedError > deadband
                input.throttle = 0;
                brakeCorrection = (speedError - deadband) / ...
                    max(obj.speedFeedbackBrakeBand, eps);
                input.brake = max(input.brake, min(1, brakeCorrection));
            else
                if input.axRef >= 0
                    input.brake = 0;
                end
                if input.axRef <= 0
                    input.throttle = 0;
                end
            end

            input.throttle = max(0, min(1, input.throttle));
            input.brake = max(0, min(1, input.brake));
        end

        function limits = estimateGGVLimits(obj, speed, mu, templateState)
            vm = obj.vehicleManager;
            tempState = templateState;
            tempState.vehicleManager = vm;
            tempState.speed = max(speed, 0);
            tempState.vx = tempState.speed;
            tempState.vy = 0;

            aeroForces = vm.aero.computeForces(tempState);
            totalNormalLoad = vm.totalMass * 9.81 + aeroForces.Fz_front + aeroForces.Fz_rear;
            peakMu = vm.tire.getPeakFriction(totalNormalLoad / 4);
            effectiveMu = min(max(peakMu, 0), max(mu, 0));
            tireAccel = effectiveMu * totalNormalLoad / vm.totalMass;

            brakeForce = max(0, vm.brakeForceCoefficient) * totalNormalLoad;
            rollingResistance = 0.015 * totalNormalLoad;
            brakeAccel = (brakeForce + aeroForces.F_drag + rollingResistance) / vm.totalMass;

            corneringUsage = 0.98;
            brakingUsage = 0.98;
            if ~isempty(obj.driverModel)
                if isprop(obj.driverModel, 'corneringUsage')
                    corneringUsage = obj.driverModel.corneringUsage;
                end
                if isprop(obj.driverModel, 'brakingUsage')
                    brakingUsage = obj.driverModel.brakingUsage;
                end
            end
            corneringUsage = max(0, min(1, corneringUsage));
            brakingUsage = max(0, min(1, brakingUsage));

            limits.maxLatAccel = max(0.1, corneringUsage * tireAccel);
            limits.maxBrakeAccel = max(0.1, brakingUsage * min(tireAccel, brakeAccel));
        end
    end
end
