classdef VehicleState
    % VEHICLESTATE Mutable vehicle state container
    % Holds all dynamic state variables for the simulation.
    % Also carries a handle reference to VehicleManager so that
    % components can access vehicle-level constants (air density, wheelbase, etc.)
    
    properties
        % Handle reference to VehicleManager (for access to constants)
        vehicleManager
        
        % Position along track [m]
        s           = 0
        
        % Body-forward speed [m/s]. Total airspeed is hypot(speed, vy).
        speed       = 0
        
        % Longitudinal acceleration [m/s^2] (positive = forward)
        ax          = 0
        
        % Lateral acceleration [m/s^2] (positive = left)
        ay          = 0
        
        % Vehicle heading angle [rad]
        heading     = 0
        
        % Yaw rate [rad/s]
        yawRate     = 0

        % Yaw acceleration [rad/s^2]
        yawAccel    = 0
        
        % Lateral velocity [m/s]
        vy          = 0

        % Vehicle sideslip angle at CG [rad]
        sideslipAngle = 0

        % Difference between vehicle heading and track tangent [rad]
        headingError = 0

        % Lateral displacement from the requested track centerline [m]
        lateralError = 0

        % Rate of change of lateralError [m/s]
        lateralErrorRate = 0

        % Forward progress speed along the track centerline [m/s]
        trackProgressSpeed = 0

        % Pitch angle [rad] (positive = nose up, e.g. acceleration squat)
        pitchAngle  = 0

        % Roll angle [rad] (positive = right side down)
        rollAngle   = 0
        
        % Ride height deviation from nominal [m] (positive = higher, e.g. over a crest)
        rideHeight  = 0
        
        % Throttle position [0-1]
        throttle    = 0
        
        % Brake pressure [0-1]
        brake       = 0
        
        % Steering input [rad]
        steer       = 0
        
        % Current track curvature [1/m]
        curvature   = 0
        
        % Current surface friction coefficient
        mu          = 1.2
        
        % Elapsed simulation time [s]
        time        = 0
        
        % Is the vehicle on track?
        onTrack     = true
    end
    
    methods
        function obj = VehicleState(varargin)
            % VEHICLESTATE Construct with optional name-value pairs
            if nargin > 0
                for i = 1:2:(nargin - 1)
                    propertyName = varargin{i};
                    if (ischar(propertyName) ...
                            || (isstring(propertyName) && isscalar(propertyName))) ...
                            && isprop(obj, propertyName)
                        obj.(char(propertyName)) = varargin{i+1};
                    end
                end
            end
        end

        function obj = set.s(obj, value)
            obj.s = utils.nonnegativeScalarOrDefault(value, 0);
        end

        function obj = set.speed(obj, value)
            obj.speed = utils.nonnegativeScalarOrDefault(value, 0);
        end

        function obj = set.ax(obj, value)
            obj.ax = utils.scalarOrDefault(value, 0);
        end

        function obj = set.ay(obj, value)
            obj.ay = utils.scalarOrDefault(value, 0);
        end

        function obj = set.heading(obj, value)
            obj.heading = utils.scalarOrDefault(value, 0);
        end

        function obj = set.yawRate(obj, value)
            obj.yawRate = utils.scalarOrDefault(value, 0);
        end

        function obj = set.yawAccel(obj, value)
            obj.yawAccel = utils.scalarOrDefault(value, 0);
        end

        function obj = set.vy(obj, value)
            obj.vy = utils.scalarOrDefault(value, 0);
        end

        function obj = set.sideslipAngle(obj, value)
            obj.sideslipAngle = utils.scalarOrDefault(value, 0);
        end

        function obj = set.headingError(obj, value)
            obj.headingError = utils.scalarOrDefault(value, 0);
        end

        function obj = set.lateralError(obj, value)
            obj.lateralError = utils.scalarOrDefault(value, 0);
        end

        function obj = set.lateralErrorRate(obj, value)
            obj.lateralErrorRate = utils.scalarOrDefault(value, 0);
        end

        function obj = set.trackProgressSpeed(obj, value)
            obj.trackProgressSpeed = utils.nonnegativeScalarOrDefault(value, 0);
        end

        function obj = set.pitchAngle(obj, value)
            obj.pitchAngle = utils.scalarOrDefault(value, 0);
        end

        function obj = set.rollAngle(obj, value)
            obj.rollAngle = utils.scalarOrDefault(value, 0);
        end

        function obj = set.rideHeight(obj, value)
            obj.rideHeight = utils.scalarOrDefault(value, 0);
        end

        function obj = set.throttle(obj, value)
            obj.throttle = utils.unitScalarOrDefault(value, 0);
        end

        function obj = set.brake(obj, value)
            obj.brake = utils.unitScalarOrDefault(value, 0);
        end

        function obj = set.steer(obj, value)
            obj.steer = utils.scalarOrDefault(value, 0);
        end

        function obj = set.curvature(obj, value)
            obj.curvature = utils.scalarOrDefault(value, 0);
        end

        function obj = set.mu(obj, value)
            obj.mu = utils.nonnegativeScalarOrDefault(value, 1.2);
        end

        function obj = set.time(obj, value)
            obj.time = utils.nonnegativeScalarOrDefault(value, 0);
        end

        function obj = set.onTrack(obj, value)
            obj.onTrack = utils.logicalScalarOrDefault(value, true);
        end
        
        function obj = updateFromDynamics(obj, ax, ay, ds, dt, curvature, heading, mu, vy, yawRate, yawAccel)
            % UPDATEFROMDYNAMICS Integrate state forward by one timestep
            %   ax         - longitudinal acceleration [m/s^2]
            %   ay         - lateral acceleration [m/s^2]
            %   ds         - distance increment [m]
            %   dt         - time increment [s]
            %   curvature  - track curvature for this integration sample [1/m]
            %   heading    - track tangent heading for this sample [rad]
            %   mu         - surface friction for this sample
            %   vy         - optional lateral velocity from transient yaw model [m/s]
            %   yawRate    - optional yaw rate from transient yaw model [rad/s]
            %   yawAccel   - optional yaw acceleration from transient yaw model [rad/s^2]
            
            obj.ax = ax;
            obj.ay = ay;
            obj.s = obj.s + ds;
            obj.speed = max(0, obj.speed + ax * dt);
            obj.curvature = curvature;
            % Temporarily store the centerline tangent. updatePathTracking()
            % adds headingError at the end of the step so obj.heading remains
            % the vehicle's physical heading, not just the path tangent.
            obj.heading = heading;
            obj.mu = mu;
            
            % Compute platform attitude from chassis/suspension state
            obj.pitchAngle = obj.computePitch();
            obj.rollAngle = obj.computeRoll();
            obj.rideHeight = obj.computeRideHeight();
            
            if nargin >= 10 && ~isempty(vy) && ~isempty(yawRate)
                obj.vy = vy;
                obj.yawRate = yawRate;
                if nargin >= 11 && ~isempty(yawAccel)
                    obj.yawAccel = yawAccel;
                else
                    obj.yawAccel = 0;
                end
            elseif obj.speed > 0.1
                obj.yawRate = obj.speed * curvature;
                obj.yawAccel = 0;
                obj.vy = 0;
            else
                obj.yawRate = 0;
                obj.yawAccel = 0;
                obj.vy = 0;
            end

            obj.sideslipAngle = atan2(obj.vy, max(obj.speed, eps));
            
            obj.time = obj.time + dt;
        end

        function obj = updatePathTracking(obj, trackHalfWidth, dt, ...
                bodyLongitudinalDistance, previousVy, previousYawRate)
            % UPDATEPATHTRACKING Estimate deviation from the track centerline.
            %
            % The lap model advances along a known centerline instead of
            % carrying full global x/y vehicle position. To keep that shortcut
            % from violating physics, this method integrates a simple path-frame
            % error model:
            %   s_dot = (u*cos(error) - vy*sin(error)) / (1 - kappa*ey)
            %   headingError_dot = yawRate - s_dot * curvature
            %   lateralError_dot = u*sin(error) + vy*cos(error)
            % A car that cannot generate the yaw rate or side velocity required
            % by the requested path will drift away from the centerline and can
            % leave the track.
            if nargin < 3 || dt <= 0
                return;
            end
            if nargin < 4 || isempty(bodyLongitudinalDistance)
                bodyLongitudinalDistance = max(obj.speed, 0) * dt;
            end
            if nargin < 5 || isempty(previousVy)
                previousVy = obj.vy;
            end
            if nargin < 6 || isempty(previousYawRate)
                previousYawRate = obj.yawRate;
            end

            trackHeading = obj.heading;
            previousHeadingError = obj.headingError;
            previousLateralError = obj.lateralError;

            bodyLongitudinalSpeed = max(bodyLongitudinalDistance, 0) / dt;
            % bodyLongitudinalDistance already represents the timestep integral
            % of body-forward speed. Use matching mean lateral velocity and yaw
            % rate so path-frame motion is integrated over the same interval
            % instead of pretending the post-step state existed for the whole
            % timestep.
            meanVy = 0.5 * (previousVy + obj.vy);
            meanYawRate = 0.5 * (previousYawRate + obj.yawRate);
            curvatureDenominator = 1 - obj.curvature * previousLateralError;
            if abs(curvatureDenominator) < 0.2
                % The curvilinear transform becomes singular when lateral
                % error approaches local turn radius. At that point the lap
                % model is outside its useful range, so mark off-track and
                % keep the denominator finite for telemetry.
                obj.onTrack = false;
                curvatureDenominator = sign(curvatureDenominator) * 0.2;
                if curvatureDenominator == 0
                    curvatureDenominator = 0.2;
                end
            end

            pathSpeed = (bodyLongitudinalSpeed * cos(previousHeadingError) ...
                - meanVy * sin(previousHeadingError)) / curvatureDenominator;
            if pathSpeed < 0
                % A full x/y model could move backward along the centerline.
                % This lap-time coordinate is one-way, so stop progress and
                % flag the state as no longer a valid on-track lap condition.
                pathSpeed = 0;
                obj.onTrack = false;
            end
            obj.trackProgressSpeed = pathSpeed;
            obj.s = max(0, obj.s - bodyLongitudinalDistance + pathSpeed * dt);

            pathYawRate = pathSpeed * obj.curvature;
            obj.headingError = obj.headingError + ...
                (meanYawRate - pathYawRate) * dt;
            % Keep angular error in a principal range; sin/cos only depend on
            % wrapped angle, and wrapping prevents unbounded numerical growth.
            obj.headingError = atan2(sin(obj.headingError), cos(obj.headingError));

            obj.lateralErrorRate = meanVy * cos(previousHeadingError) ...
                + bodyLongitudinalSpeed * sin(previousHeadingError);
            obj.lateralError = obj.lateralError + obj.lateralErrorRate * dt;
            obj.heading = trackHeading + obj.headingError;

            if abs(obj.lateralError) > max(trackHalfWidth, eps)
                obj.onTrack = false;
            end
        end

        function pitchAngle = computePitch(obj)
            % COMPUTEPITCH Compute pitch angle from suspension compression
            %   positive = nose up (e.g. acceleration squat)
            %   negative = nose down (e.g. braking dive)
            %
            % Delegates to SuspensionManager which uses the differential
            % front/rear damper positions and a trivialized geometry:
            %   pitchAngle = atan2(avgRearCompress - avgFrontCompress, wheelbase)
            
            if isempty(obj.vehicleManager)
                pitchAngle = 0;
                return;
            end

            if ~isempty(obj.vehicleManager.chassis)
                pitchAngle = obj.vehicleManager.chassis.getPitchAngle();
                return;
            end

            if isempty(obj.vehicleManager.suspension)
                pitchAngle = 0;
                return;
            end
            
            pitchAngle = obj.vehicleManager.suspension.computePitchAngle();
        end

        function rollAngle = computeRoll(obj)
            % COMPUTEROLL Compute roll angle from chassis state when present
            if isempty(obj.vehicleManager) || isempty(obj.vehicleManager.chassis)
                rollAngle = 0;
                return;
            end

            rollAngle = obj.vehicleManager.chassis.getRollAngle();
        end

        function rideHeight = computeRideHeight(obj)
            % COMPUTERIDEHEIGHT Convert chassis heave to aero ride-height sign
            % Chassis heave is positive downward; VehicleState rideHeight is
            % positive upward.
            if isempty(obj.vehicleManager) || isempty(obj.vehicleManager.chassis)
                rideHeight = 0;
                return;
            end

            rideHeight = -obj.vehicleManager.chassis.getHeave();
        end
        
        function log = toLogStruct(obj)
            % TOLOGSTRUCT Convert state to a loggable struct
            log.s         = obj.s;
            log.speed     = obj.speed;
            log.speedKmh  = obj.speed * 3.6;
            log.ax        = obj.ax;
            log.ay        = obj.ay;
            log.heading   = obj.heading;
            log.yawRate   = obj.yawRate;
            log.yawAccel  = obj.yawAccel;
            log.vy        = obj.vy;
            log.sideslipAngle = obj.sideslipAngle;
            log.headingError = obj.headingError;
            log.lateralError = obj.lateralError;
            log.lateralErrorRate = obj.lateralErrorRate;
            log.trackProgressSpeed = obj.trackProgressSpeed;
            log.rollAngle = obj.rollAngle;
            log.rideHeight = obj.rideHeight;
            log.throttle  = obj.throttle;
            log.brake     = obj.brake;
            log.steer     = obj.steer;
            log.curvature = obj.curvature;
            log.mu        = obj.mu;
            log.time      = obj.time;
        end
    end
end
