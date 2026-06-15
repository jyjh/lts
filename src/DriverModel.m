classdef DriverModel
    % DRIVERMODEL Decides throttle and brake inputs from a racing speed profile
    %
    % The driver builds a cached speed envelope from upcoming curvature,
    % surface grip, and available braking. It stays at full throttle until the
    % envelope or path-recovery logic asks for braking, then uses a high brake
    % command. The only intentional coast state is near a detected corner apex.
    % Steering combines local curvature preview with yaw, heading, lateral
    % error, and sideslip feedback.

    properties
        % Reference to VehicleManager for component access
        vehicleManager

        % Tuneable driver parameters
        brakingLookahead = 2.5    % Multiplier on calculated braking distance
        lookaheadTime    = 3.0    % Minimum seconds ahead to inspect
        minLookaheadDist = 35     % Minimum lookahead distance [m]
        hysteresis       = 0.005  % Speed tolerance as a fraction of target speed
        corneringUsage   = 0.45   % Fraction of lateral grip used for speed targets
        brakingUsage     = 0.60   % Fraction of braking capability used in planning
        minBrakeCommand  = 0.85   % Minimum brake command once braking is required
        brakeBlendSpeed  = 1.0    % Speed error [m/s] that ramps brake to 100%
        throttleBand     = 0.15   % Speed band [m/s] around target before switching
        apexDistanceTol  = 0.75   % Distance around an apex allowed to coast [m]
        curvatureTol     = 1e-6   % Curvature below this is treated as straight
        steeringUsage    = 1.0    % Fraction of path curvature converted to steer
        maxSteeringAngle = 0.6    % Steering angle limit [rad]
        minLongitudinalCommandScale = 0.15 % Longitudinal command left at peak steer
        apexPhase        = 0.5    % Corner apex location as fraction from entry to exit
        steeringLookaheadDistance = 3.0 % Curvature preview distance for steer feedforward [m]
        yawRateFeedbackGain = 0.15 % Steering correction per yaw-rate error
        headingErrorFeedbackGain = 0.10 % Steering correction per heading error
        lateralErrorFeedbackGain = 0.04 % Steering correction per lateral path error angle
        straightHeadingRecoveryGain = 0.00 % Extra heading recovery on straights
        straightLateralRecoveryGain = 0.00 % Extra lateral recovery on straights
        edgeRecoveryStartFraction = 0.25 % Start stronger path recovery at this fraction of half-width
        edgeHeadingRecoveryGain = 0.15 % Extra heading correction near track edge
        edgeLateralRecoveryGain = 0.45 % Extra lateral correction near track edge
        edgeSpeedPenalty = 0.55 % Target-speed reduction at full edge recovery
        headingRecoveryStart = 0.12 % Heading error [rad] where recovery begins
        headingRecoveryFull = 0.40 % Heading error [rad] for full recovery
        headingMisalignmentRecoveryGain = 0.30 % Extra heading correction for large heading error
        sideslipFeedbackGain = 0.10 % Steering correction per sideslip angle

        % Cached track geometry
        trackArcLen      = []
        trackCurvature   = []
        trackMu          = []
        trackSpeedProfileS = []
        trackSpeedProfile = []
        trackBrakeAccelProfile = []
    end

    methods
        function obj = DriverModel(vehicleManager)
            % DRIVERMODEL Construct with a VehicleManager reference
            obj.vehicleManager = vehicleManager;
            obj = obj.sanitizeDriverSetup();
            obj.sanitizeVehicleSetup();
            obj = obj.cacheTrackGeometry();
        end

        function obj = refreshTrackGeometry(obj)
            % REFRESHTRACKGEOMETRY Rebuild cached preview samples from vehicle.track.
            %
            % DriverModel is a value object and caches track curvature/mu for
            % fast preview queries. If VehicleManager.track changes after the
            % driver is constructed, call this so driver decisions and the
            % simulator's current track sample come from the same centerline.
            obj = obj.sanitizeDriverSetup();
            obj.sanitizeVehicleSetup();
            obj = obj.cacheTrackGeometry();
        end

        function [throttle, brake, steer] = computeInputs(obj, state)
            % COMPUTEINPUTS Decide throttle and brake for the current state
            %
            % The command policy is deliberately close to bang-bang:
            %   - brake hard if the car is above the latest-braking envelope
            %   - coast only at the local apex
            %   - otherwise use full throttle
            % Steering uses a local curvature preview plus recovery feedback
            % from the path-frame heading/lateral errors.

            obj = obj.sanitizeDriverSetup();
            obj.sanitizeVehicleSetup();
            speed = max(state.speed, 0);
            s = state.s;
            [arcLen, curvature, trackMu] = obj.getTrackGeometry();
            nPts = numel(curvature);

            idx = obj.findTrackIndexAtS(s, arcLen, nPts);

            currentMu = obj.interpolateTrackScalar(s, arcLen, trackMu);
            currentMu = obj.getStateSurfaceMu(state, currentMu);
            currentBrakeAccel = obj.getCachedBrakeAccel(s);
            if isempty(currentBrakeAccel)
                [~, currentBrakeAccel] = obj.estimateAvailableAcceleration( ...
                    state, currentMu, speed);
            end
            lookAheadDist = obj.computeLookaheadDistance(speed, currentBrakeAccel);
            profileS = obj.buildPreviewArc(s, s + lookAheadDist, arcLen);
            profileSpeed = obj.interpolateCachedSpeedProfile(profileS);
            if isempty(profileSpeed)
                profileCurvature = obj.interpolateTrackScalar(profileS, arcLen, curvature);
                profileMu = obj.interpolateTrackScalar(profileS, arcLen, trackMu);
                profileSpeed = obj.computeBackwardSpeedProfile( ...
                    state, profileS, profileCurvature, profileMu);
            end

            targetSpeed = profileSpeed(1);
            nextTargetSpeed = profileSpeed(min(2, numel(profileSpeed)));
            edgeBlend = obj.computeEdgeRecoveryBlend(state.lateralError);
            headingBlend = obj.computeHeadingRecoveryBlend(state.headingError);
            recoveryBlend = max(edgeBlend, headingBlend);
            recoverySpeedScale = max(0.35, 1 - obj.edgeSpeedPenalty * recoveryBlend);
            targetSpeed = targetSpeed * recoverySpeedScale;
            nextTargetSpeed = nextTargetSpeed * recoverySpeedScale;
            speedTolerance = max(obj.throttleBand, obj.hysteresis * max(targetSpeed, 1));
            speedError = speed - targetSpeed;
            recoveryActive = recoveryBlend > 0.10;

            [apexDistance, atApex, inActiveCorner, afterApex] = obj.distanceToRelevantApex(idx, s);
            [steer, steeringUsageFrac] = obj.computeSteeringCommand(idx, s, state);
            longitudinalCommandScale = obj.computeLongitudinalCommandScale(steeringUsageFrac);

            throttle = 0;
            brake = 0;

            if speedError > speedTolerance
                brake = obj.computeBrakeCommand(speedError);
            elseif recoveryActive
                throttle = 0;
                brake = 0;
            elseif atApex
                throttle = 0;
                brake = 0;
            elseif inActiveCorner && afterApex
                throttle = 1.0;
            elseif nextTargetSpeed < targetSpeed - speedTolerance && ...
                    speed >= targetSpeed - speedTolerance
                brake = obj.minBrakeCommand;
            else
                throttle = 1.0;
            end

            % Do not coast just because the speed error is tiny; outside the
            % apex zone, choose either throttle or brake.
            if throttle == 0 && brake == 0 && abs(apexDistance) > obj.apexDistanceTol ...
                    && ~recoveryActive
                throttle = 1.0;
            end

            throttle = throttle * longitudinalCommandScale;
            % Braking is already limited by tire residual grip in Simulator.
            % Scaling brake down at high steer lets the car remain overspeed
            % deep into corners, so keep the driver's braking request intact.
            brake = max(0, min(1, brake));
        end

        function [maxLateralAccel, maxBrakeAccel] = estimateAvailableAcceleration( ...
                obj, state, surfaceMu, sampleSpeed)
            obj = obj.sanitizeDriverSetup();
            obj.sanitizeVehicleSetup();
            if nargin < 3 || isempty(surfaceMu)
                surfaceMu = state.mu;
            end
            if nargin < 4 || isempty(sampleSpeed)
                sampleSpeed = state.speed;
            end
            [surfaceMu, sampleSpeed, outputShape] = ...
                obj.prepareGripEstimateInputs(surfaceMu, sampleSpeed);

            vm = obj.vehicleManager;
            W = vm.totalMass * 9.81;
            n = numel(surfaceMu);
            maxLateralAccel = zeros(n, 1);
            maxBrakeAccel = zeros(n, 1);

            for i = 1:n
                sampleState = state;
                sampleState.speed = sampleSpeed(i);
                sampleState.vehicleManager = vm;
                aeroForces = vm.aero.computeForces(sampleState);
                F_downforce = aeroForces.Fz_front + aeroForces.Fz_rear;
                totalNormalLoad = max(W + F_downforce, 0);

                rawFrontLoad = W * vm.staticFrontWeight + aeroForces.Fz_front;
                rawRearLoad = W * (1 - vm.staticFrontWeight) + aeroForces.Fz_rear;
                [frontNormalLoad, rearNormalLoad] = obj.redistributeAxleContactLoads( ...
                    rawFrontLoad, rawRearLoad, totalNormalLoad);
                % Cornering capacity is reduced by lateral load transfer when
                % tires are load-sensitive. Estimate the target acceleration as
                % a fixed point: ay determines inside/outside tire loads, those
                % loads determine lateral capacity, and the driver uses only a
                % configured fraction of that capacity.
                maxLateralAccel(i) = obj.estimateLateralAccelWithLoadTransfer( ...
                    frontNormalLoad, rearNormalLoad, surfaceMu(i));

                % Braking consumes longitudinal tire grip, which can differ
                % from lateral cornering grip in both Pacejka and measured tire
                % data. The aero and drag terms are evaluated at sampleSpeed so
                % a slow future corner does not borrow downforce from a fast
                % approach speed.
                % Use the same rolling resistance coefficient as the physics
                % integrator so braking targets are planned from the force
                % model that will actually be stepped by Simulator.
                rollingResistance = vm.rollingResistanceCoefficient * totalNormalLoad;
                maxBrakeAccel(i) = obj.estimateBrakeAccelWithLoadTransfer( ...
                    frontNormalLoad, rearNormalLoad, totalNormalLoad, ...
                    surfaceMu(i), aeroForces.F_drag, ...
                    utils.getStructField(aeroForces, 'dragHeight', 0), ...
                    rollingResistance);
            end

            maxLateralAccel = reshape(maxLateralAccel, outputShape);
            maxBrakeAccel = reshape(maxBrakeAccel, outputShape);
        end
    end

    methods (Access = private)
        function obj = cacheTrackGeometry(obj)
            track = obj.vehicleManager.track;
            trackPts = track.getTrackPoints();

            arcLen = components.Track.computeArcLength(trackPts);
            curvature = track.getCurvature();
            mu = track.getSurfaceFriction();
            [obj.trackArcLen, obj.trackCurvature, obj.trackMu] = ...
                obj.prepareTrackSamples(arcLen, curvature, mu);
            [obj.trackSpeedProfileS, obj.trackSpeedProfile, ...
                obj.trackBrakeAccelProfile] = ...
                obj.precomputeTrackSpeedEnvelope( ...
                    obj.trackArcLen, obj.trackCurvature, obj.trackMu);
        end

        function [arcLen, curvature, mu] = getTrackGeometry(obj)
            if ~isempty(obj.trackArcLen) && ~isempty(obj.trackCurvature) ...
                    && ~isempty(obj.trackMu)
                arcLen = obj.trackArcLen;
                curvature = obj.trackCurvature;
                mu = obj.trackMu;
                return;
            end

            track = obj.vehicleManager.track;
            trackPts = track.getTrackPoints();
            arcLen = components.Track.computeArcLength(trackPts);
            curvature = track.getCurvature();
            mu = track.getSurfaceFriction();
            [arcLen, curvature, mu] = obj.prepareTrackSamples(arcLen, curvature, mu);
        end

        function [profileS, speedProfile, brakeAccelProfile] = ...
                precomputeTrackSpeedEnvelope(obj, arcLen, curvature, mu)
            % PRECOMPUTETRACKSPEEDENVELOPE Build a cheap driver planning profile.
            %
            % The exact local planner solves load-sensitive tire capacity with
            % nested bisections. Doing that every timestep is too expensive for
            % Pacejka tires. This cached envelope keeps the important planning
            % effects, track curvature/mu and speed-dependent aero, while using
            % a static contact-load estimate suitable for driver preview.
            profileS = arcLen(:);
            curvature = curvature(:);
            mu = mu(:);
            n = min([numel(profileS), numel(curvature), numel(mu)]);
            profileS = profileS(1:n);
            curvature = curvature(1:n);
            mu = mu(1:n);

            vm = obj.vehicleManager;
            speedProfile = [];
            brakeAccelProfile = [];
            if n == 0 || isempty(vm) || isempty(vm.tire)
                return;
            end

            speedLimit = max(vm.maxSpeed, 0) * ones(n, 1);
            brakeAccelProfile = 0.1 * ones(n, 1);
            speedGuess = speedLimit;
            cornerIdx = abs(curvature) > obj.curvatureTol;

            % First solve local corner speed limits. Aero load makes cornering
            % capacity speed-dependent, so iterate the pointwise fixed point.
            for iter = 1:4
                for i = 1:n
                    [ayAvailable, ~] = obj.estimateFastPlanningAcceleration( ...
                        speedGuess(i), mu(i));
                    if cornerIdx(i)
                        speedLimit(i) = min(vm.maxSpeed, ...
                            sqrt(max(ayAvailable, 0.1) / max(abs(curvature(i)), eps)));
                    else
                        speedLimit(i) = vm.maxSpeed;
                    end
                end
                speedGuess = speedLimit;
            end

            speedProfile = speedLimit;
            for iter = 1:6
                for i = 1:n
                    [~, brakeAccelProfile(i)] = obj.estimateFastPlanningAcceleration( ...
                        speedProfile(i), mu(i));
                end

                nextProfile = speedLimit;
                for i = n-1:-1:1
                    ds = max(profileS(i+1) - profileS(i), 0.001);
                    segmentBrakeAccel = min(brakeAccelProfile(i), ...
                        brakeAccelProfile(i+1));
                    reachableSpeed = sqrt(nextProfile(i+1)^2 ...
                        + 2 * segmentBrakeAccel * ds);
                    nextProfile(i) = min(nextProfile(i), reachableSpeed);
                end

                if max(abs(nextProfile - speedProfile)) < 1e-3
                    speedProfile = nextProfile;
                    break;
                end
                speedProfile = nextProfile;
            end
        end

        function [ayAvailable, brakeAccel] = estimateFastPlanningAcceleration( ...
                obj, sampleSpeed, surfaceMu)
            vm = obj.vehicleManager;
            sampleSpeed = max(sampleSpeed, 0);
            surfaceMu = max(surfaceMu, 0);

            sampleState = VehicleState('speed', sampleSpeed);
            sampleState.vehicleManager = vm;
            aeroForces = vm.aero.computeForces(sampleState);
            W = vm.totalMass * 9.81;
            totalNormalLoad = max(W + aeroForces.Fz_front + aeroForces.Fz_rear, 0);
            if totalNormalLoad <= eps || vm.totalMass <= eps
                ayAvailable = 0.1;
                brakeAccel = 0.1;
                return;
            end

            representativeCornerLoad = totalNormalLoad / 4;
            peakLatMu = obj.getPeakLateralMu( ...
                vm.tire, representativeCornerLoad, surfaceMu);
            peakLongMu = obj.getPeakLongitudinalMu( ...
                vm.tire, representativeCornerLoad, surfaceMu);

            corneringUsageClamped = max(0, min(1, obj.corneringUsage));
            brakingUsageClamped = max(0, min(1, obj.brakingUsage));

            ayAvailable = corneringUsageClamped ...
                * peakLatMu * totalNormalLoad / max(vm.totalMass, eps);
            hydraulicLimit = max(0, vm.brakeForceCoefficient) * totalNormalLoad;
            gripLimit = peakLongMu * totalNormalLoad;
            brakeForce = min(hydraulicLimit, gripLimit);
            rollingResistance = vm.rollingResistanceCoefficient * totalNormalLoad;
            brakeAccel = (brakingUsageClamped * brakeForce ...
                + max(aeroForces.F_drag, 0) + max(rollingResistance, 0)) ...
                / max(vm.totalMass, eps);

            ayAvailable = max(ayAvailable, 0.1);
            brakeAccel = max(brakeAccel, 0.1);
        end

        function brakeAccel = getCachedBrakeAccel(obj, s)
            brakeAccel = [];
            if isempty(obj.trackSpeedProfileS) || isempty(obj.trackBrakeAccelProfile)
                return;
            end

            brakeAccel = obj.interpolateTrackScalar( ...
                s, obj.trackSpeedProfileS, obj.trackBrakeAccelProfile);
        end

        function profileSpeed = interpolateCachedSpeedProfile(obj, profileS)
            profileSpeed = [];
            if isempty(obj.trackSpeedProfileS) || isempty(obj.trackSpeedProfile)
                return;
            end

            profileSpeed = obj.interpolateTrackScalar( ...
                profileS, obj.trackSpeedProfileS, obj.trackSpeedProfile);
        end

        function edgeBlend = computeEdgeRecoveryBlend(obj, lateralError)
            halfWidth = max(obj.vehicleManager.trackHalfWidth, eps);
            edgeStart = max(0, min(0.95, obj.edgeRecoveryStartFraction)) ...
                * halfWidth;
            edgeBlend = 0;
            if abs(lateralError) > edgeStart
                edgeBlend = (abs(lateralError) - edgeStart) ...
                    / max(halfWidth - edgeStart, eps);
                edgeBlend = max(0, min(1, edgeBlend));
            end
        end

        function headingBlend = computeHeadingRecoveryBlend(obj, headingError)
            startError = max(0, obj.headingRecoveryStart);
            fullError = max(startError + eps, obj.headingRecoveryFull);
            headingBlend = (abs(headingError) - startError) ...
                / max(fullError - startError, eps);
            headingBlend = max(0, min(1, headingBlend));
        end

        function mu = getPeakLateralMu(~, tireModel, normalLoad, surfaceMu)
            % GETPEAKLATERALMU Return lateral peak mu with surface cap.
            tireMu = max(tireModel.getPeakFriction(normalLoad), 0);
            mu = min(tireMu * ones(size(surfaceMu)), max(surfaceMu, 0));
        end

        function mu = getPeakLongitudinalMu(~, tireModel, normalLoad, surfaceMu)
            % GETPEAKLONGITUDINALMU Return the tire's braking/drive peak mu.
            %
            % Older tire models only expose lateral peak friction. Newer models
            % can report longitudinal peak separately; use it when present so
            % braking distances are planned from the same grip axis the
            % Simulator uses for brake torque limits.
            if ismethod(tireModel, 'getPeakLongitudinalFriction')
                tireMu = tireModel.getPeakLongitudinalFriction(normalLoad);
            else
                tireMu = tireModel.getPeakFriction(normalLoad);
            end
            tireMu = max(tireMu, 0);
            mu = min(tireMu * ones(size(surfaceMu)), max(surfaceMu, 0));
        end

        function [frontLoad, rearLoad] = redistributeAxleContactLoads( ...
                ~, rawFrontLoad, rawRearLoad, requestedTotalLoad)
            % REDISTRIBUTEAXLECONTACTLOADS Enforce no-tension axle contact.
            %
            % AeroManager keeps signed front/rear equivalent loads so an
            % overhanging wing preserves its pitch moment. A negative equivalent
            % axle load means the contact model has run out of support there;
            % tires cannot pull down on the road. Clamp tension away, but scale
            % the remaining positive support so the driver planner preserves the
            % requested total normal load instead of inventing extra tire grip.
            requestedTotalLoad = max(requestedTotalLoad, 0);
            if requestedTotalLoad <= eps
                frontLoad = 0;
                rearLoad = 0;
                return;
            end

            loads = max([rawFrontLoad; rawRearLoad], 0);
            positiveSum = sum(loads);
            if positiveSum <= eps
                loads(:) = requestedTotalLoad / 2;
            else
                loads = loads * (requestedTotalLoad / positiveSum);
            end

            frontLoad = loads(1);
            rearLoad = loads(2);
        end

        function axBrake = estimateBrakeAccelWithLoadTransfer(obj, ...
                frontBaseLoad, rearBaseLoad, totalNormalLoad, surfaceMu, ...
                aeroDrag, dragHeight, rollingResistance)
            % ESTIMATEBRAKEACCELWITHLOADTRANSFER Solve fixed-bias braking limit.
            %
            % Braking force at the contact patches acts at ground height and
            % transfers normal load to the front axle. That load transfer feeds
            % back into the fixed front/rear brake-bias limit: too much rear
            % bias can lock the unloading rear axle even while the front axle
            % still has grip. Aerodynamic drag is included in deceleration and
            % its own pitch moment is included through dragHeight: a drag
            % resultant above the CG pitches nose-up and can reduce braking dive,
            % while one below the CG adds nose-down load transfer.
            vm = obj.vehicleManager;
            if totalNormalLoad <= eps || vm.totalMass <= eps
                axBrake = 0.1;
                return;
            end

            hydraulicLimit = max(0, vm.brakeForceCoefficient) ...
                * max(totalNormalLoad, 0);
            if hydraulicLimit <= eps
                brakeForce = 0;
            else
                lo = 0;
                hi = hydraulicLimit;
                for iter = 1:35
                    candidateBrakeForce = 0.5 * (lo + hi);
                    [frontLoad, rearLoad] = obj.computeBrakeTransferredLoads( ...
                        frontBaseLoad, rearBaseLoad, ...
                        candidateBrakeForce + rollingResistance, ...
                        aeroDrag, dragHeight);
                    gripLimit = obj.computeBrakeGripLimit( ...
                        frontLoad, rearLoad, surfaceMu);
                    if candidateBrakeForce <= gripLimit
                        lo = candidateBrakeForce;
                    else
                        hi = candidateBrakeForce;
                    end
                end
                brakeForce = lo;
            end

            brakingUsageClamped = max(0, min(1, obj.brakingUsage));
            brakeLimitedAccel = (brakingUsageClamped * brakeForce ...
                + max(aeroDrag, 0) + max(rollingResistance, 0)) ...
                / max(vm.totalMass, eps);
            axBrake = max(brakeLimitedAccel, 0.1);
        end

        function [frontLoad, rearLoad] = computeBrakeTransferredLoads( ...
                obj, frontBaseLoad, rearBaseLoad, groundRetardingForce, ...
                aeroDrag, dragHeight)
            % COMPUTEBRAKETRANSFERREDLOADS Move axle load from pitch moments.
            vm = obj.vehicleManager;
            frontBaseLoad = max(frontBaseLoad, 0);
            rearBaseLoad = max(rearBaseLoad, 0);
            wheelbase = max(vm.wheelbase, eps);

            % Positive deltaFront means extra front normal load. Ground-plane
            % braking/rolling force creates nose-down pitch, while the chassis
            % sign convention treats F_drag*dragHeight as positive nose-up when
            % the aero drag resultant is above the CG. Convert the combined
            % pitch moment into an equivalent front/rear load pair.
            deltaFront = (max(groundRetardingForce, 0) * max(vm.cgHeight, 0) ...
                - max(aeroDrag, 0) * dragHeight) / wheelbase;
            if deltaFront >= 0
                deltaFront = min(deltaFront, rearBaseLoad);
            else
                deltaFront = -min(abs(deltaFront), frontBaseLoad);
            end

            frontLoad = frontBaseLoad + deltaFront;
            rearLoad = rearBaseLoad - deltaFront;
        end

        function brakeGripLimit = computeBrakeGripLimit( ...
                obj, frontLoad, rearLoad, surfaceMu)
            % COMPUTEBRAKEGRIPLIMIT Convert axle grip to total brake-force cap.
            %
            % Simulator.step applies a fixed brake-bias split before the tire
            % model decides what force can reach the road. Express each axle's
            % longitudinal grip as a cap on total brake command force under
            % that same split, then use the smaller cap.
            vm = obj.vehicleManager;
            frontMu = obj.getPeakLongitudinalMu( ...
                vm.tire, frontLoad / 2, surfaceMu);
            rearMu = obj.getPeakLongitudinalMu( ...
                vm.tire, rearLoad / 2, surfaceMu);
            brakeBiasFront = max(0, min(1, vm.brakeBiasFront));
            brakeBiasRear = 1 - brakeBiasFront;

            brakeGripLimit = inf;
            if brakeBiasFront > eps
                brakeGripLimit = min(brakeGripLimit, frontMu * frontLoad / brakeBiasFront);
            end
            if brakeBiasRear > eps
                brakeGripLimit = min(brakeGripLimit, rearMu * rearLoad / brakeBiasRear);
            end
        end

        function ay = estimateLateralAccelWithLoadTransfer( ...
                obj, frontNormalLoad, rearNormalLoad, surfaceMu)
            % ESTIMATELATERALACCELWITHLOADTRANSFER Solve for usable cornering ay.
            %
            % A tire's peak mu generally drops as normal load rises. During a
            % turn, lateral load transfer moves load from the inside tires to
            % the outside tires, so the four-tire sum is lower than the simple
            % "axle load / 2 at every tire" estimate. This fixed-point solve
            % keeps the driver's speed target aligned with the same basic load
            % transfer physics used by the chassis/suspension model.
            vm = obj.vehicleManager;
            frontNormalLoad = max(frontNormalLoad, 0);
            rearNormalLoad = max(rearNormalLoad, 0);
            totalNormalLoad = frontNormalLoad + rearNormalLoad;
            if totalNormalLoad <= eps || vm.totalMass <= eps
                ay = 0.1;
                return;
            end

            corneringUsageClamped = max(0, min(1, obj.corneringUsage));
            noTransferCapacity = obj.computeLateralCapacityAtAy( ...
                frontNormalLoad, rearNormalLoad, surfaceMu, 0);
            hi = corneringUsageClamped * noTransferCapacity / vm.totalMass;
            hi = max(hi, 0);
            if hi <= 0
                ay = 0.1;
                return;
            end

            lo = 0;
            for iter = 1:35
                mid = 0.5 * (lo + hi);
                capacityAtMid = obj.computeLateralCapacityAtAy( ...
                    frontNormalLoad, rearNormalLoad, surfaceMu, mid);
                usableAyAtMid = corneringUsageClamped * capacityAtMid / vm.totalMass;
                if mid <= usableAyAtMid
                    lo = mid;
                else
                    hi = mid;
                end
            end

            ay = max(lo, 0.1);
        end

        function capacity = computeLateralCapacityAtAy( ...
                obj, frontNormalLoad, rearNormalLoad, surfaceMu, ay)
            % COMPUTELATERALCAPACITYATAY Sum four tire capacities after load transfer.
            vm = obj.vehicleManager;
            frontTransferShare = obj.getFrontRollStiffnessDistribution();
            totalTransfer = max(vm.totalMass, 0) * abs(ay) ...
                * max(vm.cgHeight, 0) / max(vm.trackWidth, eps);
            frontTransfer = min(totalTransfer * frontTransferShare, frontNormalLoad);
            rearTransfer = min(totalTransfer * (1 - frontTransferShare), rearNormalLoad);

            frontInside = max(0.5 * (frontNormalLoad - frontTransfer), 0);
            frontOutside = max(0.5 * (frontNormalLoad + frontTransfer), 0);
            rearInside = max(0.5 * (rearNormalLoad - rearTransfer), 0);
            rearOutside = max(0.5 * (rearNormalLoad + rearTransfer), 0);

            capacity = obj.computeTireLateralCapacity(frontInside, surfaceMu) ...
                + obj.computeTireLateralCapacity(frontOutside, surfaceMu) ...
                + obj.computeTireLateralCapacity(rearInside, surfaceMu) ...
                + obj.computeTireLateralCapacity(rearOutside, surfaceMu);
        end

        function capacity = computeTireLateralCapacity(obj, normalLoad, surfaceMu)
            mu = obj.getPeakLateralMu(obj.vehicleManager.tire, normalLoad, surfaceMu);
            capacity = mu * max(normalLoad, 0);
        end

        function frontShare = getFrontRollStiffnessDistribution(obj)
            % GETFRONTROLLSTIFFNESSDISTRIBUTION Match suspension load-transfer split.
            frontShare = 0.5;
            suspension = obj.vehicleManager.suspension;
            if ~isempty(suspension) && isprop(suspension, 'frontRollStiffDist')
                frontShare = suspension.frontRollStiffDist;
            end
            frontShare = max(0, min(1, frontShare));
        end

        function [surfaceMu, sampleSpeed, outputShape] = ...
                prepareGripEstimateInputs(~, surfaceMu, sampleSpeed)
            % PREPAREGRIPESTIMATEINPUTS Broadcast mu and speed query vectors.
            %
            % estimateAvailableAcceleration() is used both for the current car
            % state and for preview points. Surface grip and candidate speed
            % can each be scalar or profile vectors; broadcasting them here
            % keeps the force estimate explicit about which speed produced the
            % aerodynamic load.
            muShape = size(surfaceMu);
            speedShape = size(sampleSpeed);
            surfaceMu = surfaceMu(:);
            sampleSpeed = sampleSpeed(:);

            if isempty(surfaceMu)
                surfaceMu = 0;
                muShape = [1, 1];
            end
            if isempty(sampleSpeed)
                sampleSpeed = 0;
                speedShape = [1, 1];
            end

            surfaceMu(~isfinite(surfaceMu) | surfaceMu < 0) = 0;
            sampleSpeed(~isfinite(sampleSpeed) | sampleSpeed < 0) = 0;

            nMu = numel(surfaceMu);
            nSpeed = numel(sampleSpeed);
            if nMu == 1 && nSpeed > 1
                surfaceMu = surfaceMu * ones(nSpeed, 1);
                outputShape = speedShape;
            elseif nSpeed == 1 && nMu > 1
                sampleSpeed = sampleSpeed * ones(nMu, 1);
                outputShape = muShape;
            elseif nMu == nSpeed
                outputShape = muShape;
            else
                error('DriverModel:GripEstimateSizeMismatch', ...
                    'surfaceMu and sampleSpeed must be scalar or matching vectors.');
            end
        end

        function lookAheadDist = computeLookaheadDistance(obj, speed, maxBrakeAccel)
            brakeDistance = speed^2 / (2 * maxBrakeAccel);
            lookAheadDist = max([ ...
                obj.minLookaheadDist, ...
                speed * obj.lookaheadTime, ...
                brakeDistance * obj.brakingLookahead + obj.minLookaheadDist]);
        end

        function profileSpeed = computeBackwardSpeedProfile(obj, state, profileS, curvature, surfaceMu)
            surfaceMu = obj.expandProfileVector(surfaceMu, profileS);
            profileSpeed = obj.computeCornerSpeedLimit(state, curvature, surfaceMu);

            for i = numel(profileSpeed)-1:-1:1
                ds = max(profileS(i+1) - profileS(i), 0.001);
                % The car must be able to shed speed while it is inside the
                % segment. Evaluate braking at the endpoint speeds already in
                % the profile, because aero drag/downforce and tire load
                % sensitivity change with speed.
                [~, brakeAccelHere] = obj.estimateAvailableAcceleration( ...
                    state, surfaceMu(i), profileSpeed(i));
                [~, brakeAccelAhead] = obj.estimateAvailableAcceleration( ...
                    state, surfaceMu(i+1), profileSpeed(i+1));
                segmentBrakeAccel = min(brakeAccelHere, brakeAccelAhead);
                reachableSpeed = sqrt(profileSpeed(i+1)^2 + 2 * segmentBrakeAccel * ds);
                profileSpeed(i) = min(profileSpeed(i), reachableSpeed);
            end
        end

        function speedLimit = computeCornerSpeedLimit(obj, state, curvature, surfaceMu)
            vm = obj.vehicleManager;
            absKappa = abs(curvature(:));
            surfaceMu = obj.expandProfileVector(surfaceMu, absKappa);
            speedLimit = vm.maxSpeed * ones(size(absKappa));

            cornerIdx = absKappa > obj.curvatureTol;
            cornerIndices = find(cornerIdx);
            for j = 1:numel(cornerIndices)
                i = cornerIndices(j);
                speedLimit(i) = obj.solveCornerSpeedLimit( ...
                    state, absKappa(i), surfaceMu(i));
            end
            speedLimit = min(speedLimit, vm.maxSpeed);
        end

        function speedLimit = solveCornerSpeedLimit(obj, state, absKappa, surfaceMu)
            % SOLVECORNERSPEEDLIMIT Find v where lateral demand meets capacity.
            %
            % With aero, the available lateral acceleration is itself a function
            % of speed through downforce and load-sensitive tire mu. A direct
            % sqrt(ay/kappa) using the approach speed can overestimate slow
            % corner grip, so solve the fixed point:
            %   v^2 * |kappa| <= ay_available(v)
            vm = obj.vehicleManager;
            if absKappa <= obj.curvatureTol
                speedLimit = vm.maxSpeed;
                return;
            end

            [ayAtMaxSpeed, ~] = obj.estimateAvailableAcceleration( ...
                state, surfaceMu, vm.maxSpeed);
            if vm.maxSpeed^2 * absKappa <= ayAtMaxSpeed
                speedLimit = vm.maxSpeed;
                return;
            end

            lo = 0;
            hi = vm.maxSpeed;
            for iter = 1:40
                mid = 0.5 * (lo + hi);
                [ayAtMid, ~] = obj.estimateAvailableAcceleration( ...
                    state, surfaceMu, mid);
                if mid^2 * absKappa <= ayAtMid
                    lo = mid;
                else
                    hi = mid;
                end
            end
            speedLimit = lo;
        end

        function brake = computeBrakeCommand(obj, speedError)
            brake = speedError / max(obj.brakeBlendSpeed, eps);
            brake = max(obj.minBrakeCommand, brake);
            brake = max(0, min(1, brake));
        end

        function [apexDistance, atApex, inActiveCorner, afterApex] = distanceToRelevantApex(obj, idx, s)
            [arcLen, curvature] = obj.getTrackGeometry();
            absKappa = abs(curvature);
            nPts = numel(curvature);
            apexDistance = inf;
            atApex = false;
            inActiveCorner = false;
            afterApex = false;

            if idx > nPts || all(absKappa <= obj.curvatureTol)
                return;
            end

            [segmentStart, segmentEnd, ~, found] = obj.findCornerSegment(idx);
            if ~found
                return;
            end

            apexS = obj.computeApexS(arcLen, segmentStart, segmentEnd);
            apexDistance = apexS - s;
            inActiveCorner = idx >= segmentStart && idx <= segmentEnd && ...
                absKappa(idx) > obj.curvatureTol;
            afterApex = inActiveCorner && s >= apexS;
            atApex = abs(apexDistance) <= obj.apexDistanceTol && ...
                inActiveCorner;
        end

        function [steer, steeringUsageFrac] = computeSteeringCommand(obj, idx, s, state)
            [arcLen, curvature] = obj.getTrackGeometry();
            steer = 0;
            steeringUsageFrac = 0;

            previewDistance = max(obj.steeringLookaheadDistance, ...
                0.25 * max(state.speed, 0));
            previewS = min(max(s + previewDistance, arcLen(1)), arcLen(end));
            previewCurvature = obj.interpolateTrackScalar( ...
                previewS, arcLen, curvature);
            currentCurvature = obj.interpolateTrackScalar( ...
                s, arcLen, curvature);
            edgeBlend = obj.computeEdgeRecoveryBlend(state.lateralError);
            headingBlend = obj.computeHeadingRecoveryBlend(state.headingError);
            recoveryBlend = max(edgeBlend, headingBlend);

            feedforwardScale = 1 - 0.75 * recoveryBlend;
            steer = feedforwardScale * atan( ...
                obj.vehicleManager.wheelbase * previewCurvature) ...
                * obj.steeringUsage;

            pathSpeedForYaw = state.trackProgressSpeed;
            if pathSpeedForYaw <= 0 || ~isfinite(pathSpeedForYaw)
                pathSpeedForYaw = state.speed;
            end
            % Yaw rate required to follow the centerline is kappa*s_dot.
            % Using path progress speed instead of body speed avoids asking
            % for excess yaw after the car has developed heading or lateral
            % error relative to the track.
            targetYawRate = pathSpeedForYaw * currentCurvature;
            yawRateError = targetYawRate - state.yawRate;
            lateralErrorAngle = atan2(state.lateralError, max(previewDistance, eps));
            straightRecoveryBlend = 1 / (1 + (abs(currentCurvature) / 0.01)^2);
            headingGain = obj.headingErrorFeedbackGain ...
                + straightRecoveryBlend * obj.straightHeadingRecoveryGain ...
                + edgeBlend * obj.edgeHeadingRecoveryGain ...
                + headingBlend * obj.headingMisalignmentRecoveryGain;
            lateralGain = obj.lateralErrorFeedbackGain ...
                + straightRecoveryBlend * obj.straightLateralRecoveryGain ...
                + edgeBlend * obj.edgeLateralRecoveryGain;
            steer = steer + obj.yawRateFeedbackGain * yawRateError ...
                - headingGain * state.headingError ...
                - lateralGain * lateralErrorAngle ...
                - obj.sideslipFeedbackGain * state.sideslipAngle;
            steer = max(-obj.maxSteeringAngle, min(obj.maxSteeringAngle, steer));
            steeringUsageFrac = min(1, abs(steer) / max(obj.maxSteeringAngle, eps));
        end

        function [arcLen, curvature, mu] = prepareTrackSamples(~, arcLen, curvature, mu)
            arcLen = arcLen(:);
            curvature = curvature(:);
            mu = mu(:);
            n = min([numel(arcLen), numel(curvature), numel(mu)]);
            arcLen = arcLen(1:n);
            curvature = curvature(1:n);
            mu = mu(1:n);

            % Duplicate arc-length samples make interp1 ambiguous. Keeping the
            % first point preserves the entry-side property of a zero-length
            % waypoint while preventing division by zero in preview sampling.
            if isempty(arcLen)
                return;
            end
            keep = [true; diff(arcLen) > eps];
            arcLen = arcLen(keep);
            curvature = curvature(keep);
            mu = mu(keep);
        end

        function idx = findTrackIndexAtS(~, s, arcLen, nPts)
            idx = find(arcLen <= s, 1, 'last');
            if isempty(idx)
                idx = 1;
            end
            idx = max(1, min(idx, nPts));
        end

        function profileS = buildPreviewArc(~, s, sEnd, arcLen)
            s = min(max(s, arcLen(1)), arcLen(end));
            sEnd = min(max(sEnd, s), arcLen(end));
            interiorS = arcLen(arcLen > s & arcLen < sEnd);
            profileS = [s; interiorS(:); sEnd];
            profileS = profileS([true; diff(profileS) > eps]);
        end

        function values = interpolateTrackScalar(~, sQuery, arcLen, samples)
            sQueryShape = size(sQuery);
            sQuery = min(max(sQuery(:), arcLen(1)), arcLen(end));
            if numel(arcLen) == 1
                values = samples(1) * ones(size(sQuery));
            else
                values = interp1(arcLen, samples, sQuery, 'linear', 'extrap');
            end
            values = reshape(values, sQueryShape);
        end

        function surfaceMu = getStateSurfaceMu(~, state, fallbackMu)
            surfaceMu = fallbackMu;
            if isprop(state, 'mu') && isfinite(state.mu) && state.mu >= 0
                surfaceMu = state.mu;
            end
        end

        function value = expandProfileVector(~, value, profileS)
            value = value(:);
            n = numel(profileS);
            if numel(value) == 1
                value = value * ones(n, 1);
            elseif numel(value) ~= n
                error('DriverModel:ProfileSizeMismatch', ...
                    'Acceleration profile length must match preview arc length.');
            end
        end

        function scale = computeLongitudinalCommandScale(obj, steeringUsageFrac)
            lateralUse = max(0, min(1, abs(steeringUsageFrac)));
            ellipseScale = sqrt(max(0, 1 - lateralUse^2));
            reserve = max(0, min(1, obj.minLongitudinalCommandScale));
            scale = reserve + (1 - reserve) * ellipseScale;
        end

        function [segmentStart, segmentEnd, turnSign, found] = findCornerSegment(obj, idx)
            [~, curvature] = obj.getTrackGeometry();
            absKappa = abs(curvature);
            nPts = numel(curvature);
            segmentStart = 1;
            segmentEnd = 1;
            turnSign = 0;
            found = false;

            if idx > nPts || all(absKappa <= obj.curvatureTol)
                return;
            end

            if absKappa(idx) > obj.curvatureTol
                segmentStart = idx;
                turnSign = sign(curvature(idx));
                while segmentStart > 1 && ...
                        absKappa(segmentStart - 1) > obj.curvatureTol && ...
                        sign(curvature(segmentStart - 1)) == turnSign
                    segmentStart = segmentStart - 1;
                end
            else
                nextCornerOffset = find(absKappa(idx:end) > obj.curvatureTol, 1, 'first');
                if isempty(nextCornerOffset)
                    return;
                end
                segmentStart = idx + nextCornerOffset - 1;
                turnSign = sign(curvature(segmentStart));
            end

            segmentEnd = segmentStart;
            while segmentEnd < nPts && ...
                    absKappa(segmentEnd + 1) > obj.curvatureTol && ...
                    sign(curvature(segmentEnd + 1)) == turnSign
                segmentEnd = segmentEnd + 1;
            end

            found = true;
        end

        function apexS = computeApexS(obj, arcLen, segmentStart, segmentEnd)
            apexPhaseClamped = obj.getClampedApexPhase();
            segmentStartS = arcLen(segmentStart);
            segmentEndS = arcLen(segmentEnd);
            apexS = segmentStartS + apexPhaseClamped * (segmentEndS - segmentStartS);
        end

        function apexPhaseClamped = getClampedApexPhase(obj)
            apexPhaseClamped = max(0.05, min(0.95, obj.apexPhase));
        end


        function sanitizeVehicleSetup(obj)
            if ~isempty(obj.vehicleManager) ...
                    && ismethod(obj.vehicleManager, 'sanitizeSetup')
                obj.vehicleManager.sanitizeSetup();
            end
        end

        function obj = sanitizeDriverSetup(obj)
            obj.brakingLookahead = utils.coercedPositiveScalarOrDefault( ...
                obj.brakingLookahead, 2.5);
            obj.lookaheadTime = utils.coercedNonnegativeScalarOrDefault( ...
                obj.lookaheadTime, 3.0);
            obj.minLookaheadDist = utils.coercedPositiveScalarOrDefault( ...
                obj.minLookaheadDist, 35);
            obj.hysteresis = utils.coercedNonnegativeScalarOrDefault( ...
                obj.hysteresis, 0.005);
            obj.corneringUsage = utils.coercedUnitScalarOrDefault( ...
                obj.corneringUsage, 0.45);
            obj.brakingUsage = utils.coercedUnitScalarOrDefault( ...
                obj.brakingUsage, 0.60);
            obj.minBrakeCommand = utils.coercedUnitScalarOrDefault( ...
                obj.minBrakeCommand, 0.85);
            obj.brakeBlendSpeed = utils.coercedPositiveScalarOrDefault( ...
                obj.brakeBlendSpeed, 1.0);
            obj.throttleBand = utils.coercedNonnegativeScalarOrDefault( ...
                obj.throttleBand, 0.15);
            obj.apexDistanceTol = utils.coercedNonnegativeScalarOrDefault( ...
                obj.apexDistanceTol, 0.75);
            obj.curvatureTol = utils.coercedPositiveScalarOrDefault( ...
                obj.curvatureTol, 1e-6);
            obj.steeringUsage = utils.coercedNonnegativeScalarOrDefault( ...
                obj.steeringUsage, 1.0);
            obj.maxSteeringAngle = utils.coercedPositiveScalarOrDefault( ...
                obj.maxSteeringAngle, 0.6);
            obj.minLongitudinalCommandScale = utils.coercedUnitScalarOrDefault( ...
                obj.minLongitudinalCommandScale, 0.15);
            obj.apexPhase = utils.coercedUnitScalarOrDefault( ...
                obj.apexPhase, 0.5);
            obj.steeringLookaheadDistance = utils.coercedNonnegativeScalarOrDefault( ...
                obj.steeringLookaheadDistance, 3.0);
            obj.yawRateFeedbackGain = utils.coercedNonnegativeScalarOrDefault( ...
                obj.yawRateFeedbackGain, 0.15);
            obj.headingErrorFeedbackGain = utils.coercedNonnegativeScalarOrDefault( ...
                obj.headingErrorFeedbackGain, 0.10);
            obj.lateralErrorFeedbackGain = utils.coercedNonnegativeScalarOrDefault( ...
                obj.lateralErrorFeedbackGain, 0.04);
            obj.straightHeadingRecoveryGain = utils.coercedNonnegativeScalarOrDefault( ...
                obj.straightHeadingRecoveryGain, 0.00);
            obj.straightLateralRecoveryGain = utils.coercedNonnegativeScalarOrDefault( ...
                obj.straightLateralRecoveryGain, 0.00);
            obj.edgeRecoveryStartFraction = utils.coercedUnitScalarOrDefault( ...
                obj.edgeRecoveryStartFraction, 0.25);
            obj.edgeHeadingRecoveryGain = utils.coercedNonnegativeScalarOrDefault( ...
                obj.edgeHeadingRecoveryGain, 0.15);
            obj.edgeLateralRecoveryGain = utils.coercedNonnegativeScalarOrDefault( ...
                obj.edgeLateralRecoveryGain, 0.45);
            obj.edgeSpeedPenalty = utils.coercedUnitScalarOrDefault( ...
                obj.edgeSpeedPenalty, 0.55);
            obj.headingRecoveryStart = utils.coercedNonnegativeScalarOrDefault( ...
                obj.headingRecoveryStart, 0.12);
            obj.headingRecoveryFull = utils.coercedNonnegativeScalarOrDefault( ...
                obj.headingRecoveryFull, 0.40);
            if obj.headingRecoveryFull <= obj.headingRecoveryStart
                obj.headingRecoveryFull = obj.headingRecoveryStart + eps;
            end
            obj.headingMisalignmentRecoveryGain = utils.coercedNonnegativeScalarOrDefault( ...
                obj.headingMisalignmentRecoveryGain, 0.30);
            obj.sideslipFeedbackGain = utils.coercedNonnegativeScalarOrDefault( ...
                obj.sideslipFeedbackGain, 0.10);
        end
    end
end
