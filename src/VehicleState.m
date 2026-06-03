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
        
        % Lateral velocity [m/s]
        vy          = 0
        
        % Pitch angle [rad] (positive = nose up, e.g. under braking)
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
            obj.curvature = curvature;
            obj.heading = heading;
            obj.mu = mu;
            
            % Compute pitch angle from current dynamics
            obj.pitchAngle = obj.computePitch();
            
            % Yaw rate from speed and curvature (bicycle model)
            if obj.speed > 0.1
                obj.yawRate = obj.speed * curvature;
                obj.vy = ay / obj.speed * 0;  % TODO: proper lateral dynamics
            else
                obj.yawRate = 0;
                obj.vy = 0;
            end
            
            obj.time = obj.time + dt;
        end
        
        function pitchAngle = computePitch(obj)
            % COMPUTEPITCH Compute pitch angle from vehicle dynamics
            % Temporary simplistic model: pitch proportional to longitudinal accel
            %   pitchAngle = pitchStiffness * ax_in_g
            %   positive = nose up (braking)
            %
            % TODO: Replace with proper suspension-based pitch computation
            % considering pitch inertia, damping, and heave coupling
            
            pitchStiffness = 0.002;  % [rad/g] temporary constant
            ax_g = obj.ax / 9.81;
            pitchAngle = pitchStiffness * ax_g;
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
            log.throttle  = obj.throttle;
            log.brake     = obj.brake;
            log.steer     = obj.steer;
            log.curvature = obj.curvature;
            log.mu        = obj.mu;
            log.time      = obj.time;
        end
    end
end