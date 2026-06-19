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
        
        % Heading angle [rad]
        heading     = 0
        
        % Yaw rate [rad/s]
        yawRate     = 0

        % Yaw acceleration [rad/s^2]
        yawAccel    = 0
        
        % Pitch angle [rad] (positive = nose up, e.g. acceleration squat)
        pitchAngle  = 0
        
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
            if nargin > 0
                for i = 1:2:nargin
                    if isprop(obj, varargin{i})
                        obj.(varargin{i}) = varargin{i+1};
                    end
                end
            end

            if isnan(obj.vx)
                obj.vx = obj.speed;
            elseif obj.speed <= 0
                obj.speed = hypot(obj.vx, obj.vy);
            end
            if isnan(obj.yaw)
                obj.yaw = obj.heading;
            else
                obj.heading = obj.yaw;
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
                lateralError, dt, mu)
            % UPDATEFROMPLANARDYNAMICS Store a free planar 4-wheel state update.
            obj.ax = ax;
            obj.ay = ay;
            obj.yawAccel = yawAccel;
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
            obj.time = obj.time + dt;
        end
        
        function pitchAngle = computePitch(obj)
            % COMPUTEPITCH Compute pitch angle from suspension compression
            %   positive = nose up (e.g. acceleration squat)
            %   negative = nose down (e.g. braking dive)
            %
            % Delegates to SuspensionManager which uses the differential
            % front/rear sprung-body positions from static equilibrium:
            %   pitchAngle = atan2(avgRearSprungDown - avgFrontSprungDown, wheelbase)
            
            if isempty(obj.vehicleManager) || isempty(obj.vehicleManager.suspension)
                pitchAngle = 0;
                return;
            end
            
            pitchAngle = obj.vehicleManager.suspension.computePitchAngle();
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
            log.heading   = obj.heading;
            log.yawRate   = obj.yawRate;
            log.yawAccel  = obj.yawAccel;
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
