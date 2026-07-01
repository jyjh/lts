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
        % When the feedforward plan calls for coast (both pedals zero) and the
        % speed error stays within this tolerance [m/s], the closed-loop layer
        % holds coast instead of pedaling to correct the small drift. This lets
        % visible lift-off coasting plateaus survive at corner entries/exits.
        % Only applies when the plan is actually slowing (axRef below
        % coastAxRefThreshold), not when it is holding speed against drag.
        coastSpeedTolerance = 0.75
        coastAxRefThreshold = 0.05  % |axRef| above which a coast plan counts as slowing
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
            vTarget = vm.maxSpeed * ones(n, 1);

            % Iterating lets speed-dependent aero influence the GGV envelope
            % without turning this into a full trajectory optimization.
            for iter = 1:3 %#ok<NASGU>
                for i = 1:n
                    if abs(curvature(i)) > 1e-6
                        limits = obj.estimateGGVLimits(vTarget(i), initialState);
                        vTarget(i) = min(vm.maxSpeed, ...
                            sqrt(max(limits.maxLatAccel, 0.1) / abs(curvature(i))));
                    else
                        vTarget(i) = vm.maxSpeed;
                    end
                end
            end

            % Backward (braking) sweep: collect brake capability at the corner-
            % limited envelope speed, then propagate the latest-feasible braking
            % point upstream. Drive capability is not needed here; it is queried
            % at the actual sweep speed during the forward pass below.
            maxBrakeAccel = zeros(n, 1);
            maxDriveAccel = zeros(n, 1);   % filled in during the forward sweep
            F_drive_full = zeros(n, 1);
            F_resistance = zeros(n, 1);
            brakeForceAccel = zeros(n, 1);
            for i = 1:n
                limits = obj.estimateGGVLimits(vTarget(i), initialState);
                maxBrakeAccel(i) = limits.maxBrakeAccel;
            end

            for i = n-1:-1:1
                ds = max(trackData.arcLen(i+1) - trackData.arcLen(i), 0.001);
                reachableSpeed = sqrt(vTarget(i+1)^2 + 2 * maxBrakeAccel(i+1) * ds);
                vTarget(i) = min(vTarget(i), reachableSpeed);
            end

            % Forward (acceleration) sweep. Drive capability is strongly
            % speed-dependent (traction-limited at low speed, power-limited at
            % high speed), so it is queried at the actual sweep speed
            % speedPlan(i) — not the corner-limited vTarget(i) — so corner-exit
            % acceleration is modeled correctly. The capability returned at
            % each point is reused by the pedal map below for consistency.
            speedPlan = vTarget;
            speedPlan(1) = min(max(initialState.speed, 0), vTarget(1));
            for i = 1:n
                limits = obj.estimateGGVLimits(speedPlan(i), initialState);
                maxDriveAccel(i) = limits.maxDriveAccel;
                F_drive_full(i) = limits.F_drive_full;
                F_resistance(i) = limits.F_resistance;
                brakeForceAccel(i) = limits.brakeForceAccel;
                if i < n
                    ds = max(trackData.arcLen(i+1) - trackData.arcLen(i), 0.001);
                    axCap = max(maxDriveAccel(i), 0);
                    reachableSpeed = sqrt(speedPlan(i)^2 + 2 * axCap * ds);
                    speedPlan(i+1) = min(vTarget(i+1), reachableSpeed);
                end
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

            % Physics-based pedal map: each planned accel maps to partial
            % throttle, coast, or gradual brake based on the actual force
            % balance, instead of saturating to {0, WOT, full-brake}.
            brakeRef = zeros(n, 1);
            throttleRef = zeros(n, 1);
            for i = 1:n
                [throttleRef(i), brakeRef(i)] = DriverInputPlanner.computePedals( ...
                    axRef(i), F_drive_full(i), F_resistance(i), ...
                    vm.totalMass, brakeForceAccel(i));
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
            plannedCoast = input.throttle <= 0 && input.brake <= 0;
            % Coast-aware hold: when the feedforward plan calls for coast AND
            % the planned axRef indicates the car is meant to be slowing (drag
            % doing the work, not holding speed), tolerate small speed drift
            % without pedaling so lift-off coast plateaus survive. When axRef is
            % near zero (maintain-speed coast) the feedback still trims, since
            % holding speed against drag needs throttle.
            planningToSlow = input.axRef < -obj.coastAxRefThreshold;
            if plannedCoast && planningToSlow && actualSpeed >= 0.5 && ...
                    abs(speedError) <= obj.coastSpeedTolerance
                input.throttle = 0;
                input.brake = 0;
            elseif actualSpeed < 0.5 && speedError <= 0
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

        function limits = estimateGGVLimits(obj, speed, templateState)
            vm = obj.vehicleManager;
            tempState = templateState;
            tempState.vehicleManager = vm;
            tempState.speed = max(speed, 0);
            tempState.vx = tempState.speed;
            tempState.vy = 0;

            aeroForces = vm.aero.computeForces(tempState);
            F_drag = max(0, aeroForces.F_drag);
            totalNormalLoad = vm.totalMass * vm.g + aeroForces.Fz_front + aeroForces.Fz_rear;
            peakMu = vm.tire.getPeakFriction(totalNormalLoad / 4);
            % Grip comes entirely from the tire model (no surface mu cap);
            % the vehicles run on dry rubber with no friction variability.
            tireAccel = max(peakMu, 0) * totalNormalLoad / vm.totalMass;

            brakeForce = max(0, vm.brakeForceCoefficient) * totalNormalLoad;
            rollingResistance = 0.015 * totalNormalLoad;
            brakeAccel = (brakeForce + F_drag + rollingResistance) / vm.totalMass;

            corneringUsage = 0.98;
            brakingUsage = 0.98;
            driveUsage = 0.98;
            if ~isempty(obj.driverModel)
                if isprop(obj.driverModel, 'corneringUsage')
                    corneringUsage = obj.driverModel.corneringUsage;
                end
                if isprop(obj.driverModel, 'brakingUsage')
                    brakingUsage = obj.driverModel.brakingUsage;
                end
                if isprop(obj.driverModel, 'driveUsage')
                    driveUsage = obj.driverModel.driveUsage;
                end
            end
            corneringUsage = max(0, min(1, corneringUsage));
            brakingUsage = max(0, min(1, brakingUsage));
            driveUsage = max(0, min(1, driveUsage));

            % --- Forward drive capability (speed- and load-dependent) ---
            % Full-throttle wheel force from the powertrain map, capped by the
            % driven (rear) axle's traction limit. Net of drag + rolling
            % resistance so it is the achievable forward accel, not gross.
            F_drive_full = 0;
            if ~isempty(vm.powertrain)
                F_drive_full = max(0, vm.powertrain.computeMaxDriveForce(tempState.speed));
            end
            W = vm.totalMass * vm.g;
            rearNormalLoad = max(W * (1 - vm.staticFrontWeight) + aeroForces.Fz_rear, 0);
            rearMu = max(vm.tire.getPeakFriction(rearNormalLoad / 2), 0);
            F_traction_rear = driveUsage * rearMu * rearNormalLoad;
            F_drive_full = min(F_drive_full, F_traction_rear);

            F_resistance = F_drag + rollingResistance;
            maxDriveAccel = max(0, (F_drive_full - F_resistance) / vm.totalMass);
            % Coasting (lift-off) deceleration comes for free from drag +
            % rolling resistance; no brake needed if the required decel is at
            % or below this. brakeForceAccel is hydraulic-brake decel only
            % (drag adds on top during actual braking), used to map a required
            % decel onto a gradual [0,1] brake command.
            coastDecel = F_resistance / vm.totalMass;
            brakeForceAccel = max(0.1, brakingUsage * brakeForce / vm.totalMass);

            limits.maxLatAccel = max(0.1, corneringUsage * tireAccel);
            limits.maxBrakeAccel = max(0.1, brakingUsage * min(tireAccel, brakeAccel));
            % --- Capability fields consumed by the physics-based pedal map ---
            limits.F_drive_full = F_drive_full;        % Full-throttle wheel force, traction-capped [N]
            limits.F_resistance = F_resistance;        % Drag + rolling resistance at this speed [N]
            limits.maxDriveAccel = maxDriveAccel;      % Net forward accel capability [m/s^2]
            limits.coastDecel = coastDecel;            % Free lift-off decel [m/s^2]
            limits.brakeForceAccel = brakeForceAccel;  % Hydraulic-brake-only decel per unit brake [m/s^2]
        end
    end

    methods (Static)
        function [throttle, brake] = computePedals(axRef, F_drive_full, F_resistance, mass, brakeForceAccel)
            % COMPUTEPEDALS Map a required longitudinal accel onto pedal commands.
            %
            %   [throttle, brake] = DriverInputPlanner.computePedals( ...
            %       axRef, F_drive_full, F_resistance, mass, brakeForceAccel)
            %
            %   Pure (stateless) physics-based pedal map that produces all three
            %   regimes a real driver uses:
            %     - WOT when the required drive force meets/exceeds full-throttle,
            %     - partial throttle to maintain speed against drag/rolling (cruise),
            %     - coast (both pedals zero) when drag alone provides the decel,
            %     - gradual brake proportional to the decel beyond coast.
            %
            %   Inputs:
            %     axRef           - required longitudinal accel [m/s^2] (+ = drive)
            %     F_drive_full    - full-throttle wheel force, traction-capped [N]
            %     F_resistance    - drag + rolling resistance at this speed [N]
            %     mass            - vehicle mass [kg]
            %     brakeForceAccel - decel per unit brake command [m/s^2]
            %
            %   Pedals are mutually exclusive (never both > 0), clamped to [0,1].
            mass = max(mass, eps);
            brakeForceAccel = max(brakeForceAccel, eps);

            % Force the wheels must apply at the contact patch to net axRef,
            % i.e. invert  ax = (F_drive - F_resistance) / mass.
            F_req = axRef * mass + F_resistance;

            % Coast deadband: a real driver lifts rather than holding a few
            % percent throttle or dabbing a few percent brake. When the required
            % force magnitude is below this fraction of the drive/brake scale,
            % snap to coast. The threshold is small enough (a few %) that it
            % only suppresses negligible pedal commands, never genuine cruise
            % throttle (which on this car is ~10%+ to overcome drag).
            coastFraction = 0.03;

            throttle = 0;
            brake = 0;
            if F_req <= 0
                % No drive force needed: the car must hold speed or slow down.
                requiredDecel = max(0, -axRef);
                coastDecel = F_resistance / mass;
                brakeForceTotal = brakeForceAccel * mass;
                if requiredDecel <= coastDecel
                    % Drag/rolling resistance alone covers the decel -> coast.
                    throttle = 0;
                    brake = 0;
                elseif brakeForceTotal > 0 && ...
                        (requiredDecel - coastDecel) < coastFraction * brakeForceAccel
                    % Required brake is negligible -> coast.
                    throttle = 0;
                    brake = 0;
                else
                    % Hydraulic brake fills the gap beyond coast, gradually.
                    brake = (requiredDecel - coastDecel) / brakeForceAccel;
                    brake = max(0, min(1, brake));
                end
            elseif F_drive_full <= 0
                % No tractive capability recorded (e.g. at/over rev limit) but
                % drive was requested: ask for WOT and let the powertrain
                % return whatever it can.
                throttle = 1;
            else
                throttle = F_req / F_drive_full;
                throttle = max(0, min(1, throttle));
                % Negligible throttle (below a few % of full) -> coast.
                if throttle < coastFraction
                    throttle = 0;
                end
            end
        end
    end
end
