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
        
        % Vehicle speed [m/s]
        speed       = 0
        
        % Longitudinal acceleration [m/s^2] (positive = forward)
        ax          = 0
        
        % Lateral acceleration [m/s^2] (positive = left)
        ay          = 0
        
        % Heading angle [rad]
        heading     = 0
        
        % Yaw rate [rad/s]
        yawRate     = 0

        % Yaw acceleration [rad/s^2]
        yawAccel    = 0
        
        % Lateral velocity [m/s]
        vy          = 0

        % Vehicle sideslip angle at CG [rad]
        sideslipAngle = 0
        
        % Pitch angle [rad] (positive = nose up, e.g. under braking)
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
                for i = 1:2:nargin
                    if isprop(obj, varargin{i})
                        obj.(varargin{i}) = varargin{i+1};
                    end
                end
            end
        end
        
        function obj = updateFromDynamics(obj, ax, ay, ds, dt, curvature, heading, mu, vy, yawRate, yawAccel)
            % UPDATEFROMDYNAMICS Integrate state forward by one timestep
            %   ax         - longitudinal acceleration [m/s^2]
            %   ay         - lateral acceleration [m/s^2]
            %   ds         - distance increment [m]
            %   dt         - time increment [s]
            %   curvature  - track curvature at new position [1/m]
            %   heading    - track heading at new position [rad]
            %   mu         - surface friction at new position
            %   vy         - optional lateral velocity from transient yaw model [m/s]
            %   yawRate    - optional yaw rate from transient yaw model [rad/s]
            %   yawAccel   - optional yaw acceleration from transient yaw model [rad/s^2]
            
            obj.ax = ax;
            obj.ay = ay;
            obj.s = obj.s + ds;
            obj.speed = max(0, obj.speed + ax * dt);
            obj.curvature = curvature;
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
