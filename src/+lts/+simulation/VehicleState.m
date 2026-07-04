classdef VehicleState
    % VEHICLESTATE Mutable vehicle state container
    % Holds all dynamic state variables for the simulation.
    % Also carries a handle reference to lts.vehicle.VehicleManager so that
    % components can access vehicle-level constants (air density, wheelbase, etc.)
    
    properties
        % Handle reference to lts.vehicle.VehicleManager (for access to constants)
        vehicleManager
        
        % Position along track [m]
        s           = 0

        % World position [m]
        x           = NaN
        y           = NaN

        % Vehicle yaw angle [rad]
        yaw         = NaN
        
        % Vehicle speed [m/s]
        speed       = 0

        % Body-frame velocity components [m/s]
        vx          = NaN
        vy          = 0

        % Body slip angle [rad], positive when velocity points left of body x-axis
        bodySlipAngle = 0
        
        % Longitudinal acceleration [m/s^2] (positive = forward)
        ax          = 0
        
        % Lateral acceleration [m/s^2] (positive = left)
        ay          = 0

        % Axle-center lateral accelerations [m/s^2] (positive = left)
        frontAxleAy = NaN
        rearAxleAy  = NaN
        
        % Heading angle [rad]
        heading     = 0
        
        % Yaw rate [rad/s]
        yawRate     = 0

        % Yaw acceleration [rad/s^2]
        yawAccel    = 0
        
        % Pitch angle [rad] (positive = nose up, e.g. acceleration squat)
        pitchAngle  = 0

        % Roll angle [rad] (positive = right side down, e.g. in a left turn)
        rollAngle   = 0

        % Front/rear chassis roll angles [rad] and the chassis twist
        % (front - rear). Equal when the tub is torsionally rigid; they
        % differ under asymmetric load with finite torsional rigidity.
        frontRollAngle = 0
        rearRollAngle  = 0
        twistAngle     = 0

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

        % Reference track projection telemetry
        refS        = 0
        refHeading  = 0
        refCurvature = 0
        lateralError = 0
        
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
            specifiedYaw = false;
            specifiedHeading = false;
            if nargin > 0
                for i = 1:2:nargin
                    if isprop(obj, varargin{i})
                        obj.(varargin{i}) = varargin{i+1};
                        propertyName = char(varargin{i});
                        specifiedYaw = specifiedYaw || strcmp(propertyName, 'yaw');
                        specifiedHeading = specifiedHeading || strcmp(propertyName, 'heading');
                    end
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
            obj.bodySlipAngle = obj.computeBodySlipAngle();
        end
        
        function obj = updateFromDynamics(obj, ax, ay, ds, dt, curvature, heading, mu)
            % UPDATEFROMDYNAMICS Integrate state forward by one timestep
            %   ax         - longitudinal acceleration [m/s^2]
            %   ay         - lateral acceleration [m/s^2]
            %   ds         - distance increment [m]
            %   dt         - time increment [s]
            %   curvature  - track curvature at new position [1/m]
            %   heading    - track heading at new position [rad]
            %   mu         - surface friction at new position
            
            obj.ax = ax;
            obj.ay = ay;
            obj.frontAxleAy = ay;
            obj.rearAxleAy = ay;
            obj.s = obj.s + ds;
            obj.speed = max(0, obj.speed + ax * dt);
            obj.vx = obj.speed;
            obj.vy = 0;
            obj.bodySlipAngle = obj.computeBodySlipAngle();
            obj.curvature = curvature;
            obj.heading = heading;
            obj.yaw = heading;
            obj.mu = mu;
            obj.refS = obj.s;
            obj.refHeading = heading;
            obj.refCurvature = curvature;
            
            % Compute pitch angle from current dynamics
            obj.pitchAngle = obj.computePitch();
            obj.rollAngle = obj.computeRoll();
            obj.frontRollAngle = obj.computeFrontRoll();
            obj.rearRollAngle  = obj.computeRearRoll();
            obj.twistAngle     = obj.computeTwist();
            obj.rideHeight = obj.computeRideHeight();

            % Yaw rate from speed and curvature (bicycle model)
            if obj.speed > 0.1
                obj.yawRate = obj.speed * curvature;
            else
                obj.yawRate = 0;
            end
            
            obj.time = obj.time + dt;
        end

        function obj = updateFromPlanarDynamics(obj, ax, ay, yawAccel, ...
                vx, vy, yawRate, yaw, x, y, refS, refHeading, refCurvature, ...
                lateralError, dt, mu, frontAxleAy, rearAxleAy)
            % UPDATEFROMPLANARDYNAMICS Store a free planar 4-wheel state update.
            % The simulator has already integrated body/world velocity and
            % yaw from Newton/Euler equations. This method commits those
            % results and refreshes derived telemetry (speed, body slip,
            % pitch/roll/ride height readbacks, reference projection fields).
            obj.ax = ax;
            obj.ay = ay;
            obj.yawAccel = yawAccel;
            if nargin < 17 || isempty(frontAxleAy)
                frontAxleAy = ay;
            end
            if nargin < 18 || isempty(rearAxleAy)
                rearAxleAy = ay;
            end
            obj.frontAxleAy = frontAxleAy;
            obj.rearAxleAy = rearAxleAy;
            obj.vx = vx;
            obj.vy = vy;
            obj.speed = hypot(vx, vy);
            obj.bodySlipAngle = obj.computeBodySlipAngle();
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
            obj.mu = mu;

            obj.pitchAngle = obj.computePitch();
            obj.rollAngle = obj.computeRoll();
            obj.frontRollAngle = obj.computeFrontRoll();
            obj.rearRollAngle  = obj.computeRearRoll();
            obj.twistAngle     = obj.computeTwist();
            obj.rideHeight = obj.computeRideHeight();
            obj.time = obj.time + dt;
        end
        
        function pitchAngle = computePitch(obj)
            % COMPUTEPITCH Pitch angle [rad], positive = nose up.
            %   When a chassis attitude model is present it owns pitch (an
            %   integrated rigid-body DOF driven by m*ax*cgH + aero pitch
            %   moment). Otherwise pitch is inferred from the suspension
            %   front/rear sprung-position difference.
            if isempty(obj.vehicleManager)
                pitchAngle = 0;
                return;
            end
            if ~isempty(obj.vehicleManager.chassis) && ...
                    isa(obj.vehicleManager.chassis, 'lts.components.Chassis.ChassisComponent')
                pitchAngle = obj.vehicleManager.chassis.getPitchAngle();
                return;
            end
            if ~isempty(obj.vehicleManager.suspension) && ...
                    ismethod(obj.vehicleManager.suspension, 'computePitchAngle')
                pitchAngle = obj.vehicleManager.suspension.computePitchAngle();
                return;
            end
            pitchAngle = 0;
        end

        function rollAngle = computeRoll(obj)
            % COMPUTEROLL Roll angle [rad], positive = right side down.
            % Sourced from the chassis attitude model when present.
            if isempty(obj.vehicleManager) || isempty(obj.vehicleManager.chassis) || ...
                    ~isa(obj.vehicleManager.chassis, 'lts.components.Chassis.ChassisComponent')
                rollAngle = 0;
                return;
            end
            rollAngle = obj.vehicleManager.chassis.getRollAngle();
        end

        function rollAngle = computeFrontRoll(obj)
            % COMPUTEFRONTROLL Front-end chassis roll angle [rad].
            if isempty(obj.vehicleManager) || isempty(obj.vehicleManager.chassis) || ...
                    ~isa(obj.vehicleManager.chassis, 'lts.components.Chassis.ChassisComponent') || ...
                    ~ismethod(obj.vehicleManager.chassis, 'getFrontRollAngle')
                rollAngle = obj.computeRoll();
                return;
            end
            rollAngle = obj.vehicleManager.chassis.getFrontRollAngle();
        end

        function rollAngle = computeRearRoll(obj)
            % COMPUTEREARROLL Rear-end chassis roll angle [rad].
            if isempty(obj.vehicleManager) || isempty(obj.vehicleManager.chassis) || ...
                    ~isa(obj.vehicleManager.chassis, 'lts.components.Chassis.ChassisComponent') || ...
                    ~ismethod(obj.vehicleManager.chassis, 'getRearRollAngle')
                rollAngle = obj.computeRoll();
                return;
            end
            rollAngle = obj.vehicleManager.chassis.getRearRollAngle();
        end

        function twist = computeTwist(obj)
            % COMPUTETWIST Chassis torsional twist [rad] = front - rear roll.
            if isempty(obj.vehicleManager) || isempty(obj.vehicleManager.chassis) || ...
                    ~isa(obj.vehicleManager.chassis, 'lts.components.Chassis.ChassisComponent') || ...
                    ~ismethod(obj.vehicleManager.chassis, 'getTwistAngle')
                twist = 0;
                return;
            end
            twist = obj.vehicleManager.chassis.getTwistAngle();
        end

        function rideHeight = computeRideHeight(obj)
            % COMPUTERIDEHEIGHT Ride-height deviation [m], positive = higher.
            % Downforce compresses the sprung mass downward, so heave
            % (positive down) maps to a negative ride-height deviation.
            if isempty(obj.vehicleManager) || isempty(obj.vehicleManager.chassis) || ...
                    ~isa(obj.vehicleManager.chassis, 'lts.components.Chassis.ChassisComponent')
                rideHeight = 0;
                return;
            end
            rideHeight = -obj.vehicleManager.chassis.getHeave();
        end

        function bodySlipAngle = computeBodySlipAngle(obj)
            % COMPUTEBODYSLIPANGLE Compute body sideslip from body-frame velocity.
            if hypot(obj.vx, obj.vy) <= eps
                bodySlipAngle = 0;
                return;
            end

            bodySlipAngle = atan2(obj.vy, obj.vx);
        end
        
        function log = toLogStruct(obj)
            % TOLOGSTRUCT Convert state to a loggable struct
            log.s         = obj.s;
            log.x         = obj.x;
            log.y         = obj.y;
            log.yaw       = obj.yaw;
            log.speed     = obj.speed;
            log.speedKmh  = obj.speed * 3.6;
            log.vx        = obj.vx;
            log.vy        = obj.vy;
            log.bodySlipAngle = obj.bodySlipAngle;
            log.ax        = obj.ax;
            log.ay        = obj.ay;
            log.frontAxleAy = obj.frontAxleAy;
            log.rearAxleAy  = obj.rearAxleAy;
            log.heading   = obj.heading;
        log.yawRate   = obj.yawRate;
        log.yawAccel  = obj.yawAccel;
        log.pitchAngle = obj.pitchAngle;
        log.rollAngle  = obj.rollAngle;
        log.frontRollAngle = obj.frontRollAngle;
        log.rearRollAngle  = obj.rearRollAngle;
        log.twistAngle     = obj.twistAngle;
        log.rideHeight = obj.rideHeight;
            log.throttle  = obj.throttle;
            log.brake     = obj.brake;
            log.steer     = obj.steer;
            log.curvature = obj.curvature;
            log.refS      = obj.refS;
            log.refHeading = obj.refHeading;
            log.refCurvature = obj.refCurvature;
            log.lateralError = obj.lateralError;
            log.onTrack = obj.onTrack;
            log.mu        = obj.mu;
            log.time      = obj.time;
        end
    end
end
