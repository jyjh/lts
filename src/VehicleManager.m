classdef VehicleManager < handle
    % VEHICLEMANAGER Configuration container for vehicle components and parameters
    % Holds references to all swappable components (aero, chassis, suspension, powertrain, tire, track)
    % and all non-changing vehicle parameters (mass, wheelbase, track width, etc).
    %
    % Simulation is handled by the Simulator class, driver decisions by DriverModel.
    
    properties
        % Swappable component objects
        aero        % components.Aero.AeroManager
        chassis     % components.Chassis.ChassisComponent
        suspension  % components.Suspension.SuspensionManager
        powertrain  % components.PowertrainComponent
        tire        % components.Tire.TireModel
        track       % components.Track
        
        % Vehicle parameters
        totalMass     = 215      % Total mass with driver [kg]
        wheelbase     = 1.95     % Wheelbase [m]
        trackWidth    = 1.2      % Track width [m]
        cgHeight      = 0.28     % CG height [m]
        yawInertia    = 85       % Yaw moment of inertia [kg*m^2]
        airDensity    = 1.225    % Air density [kg/m^3]
        staticFrontWeight = 0.45 % Static front weight distribution [0-1]
        brakeBiasFront = 0.60    % Fraction of brake force commanded to front axle [0-1]
        brakeForceCoefficient = 0.70 % Hydraulic brake force capacity as fraction of normal load
        rollingResistanceCoefficient = 0.015 % Rolling resistance force divided by normal load
        yawRateDamping = 25.0    % Numerical/physical yaw damping [N*m*s/rad]
        lateralVelocityDamping = 0.4 % Optional linear lateral damping coefficient [1/s]
        maxSideslipAngle = 0.18  % Sideslip telemetry threshold for path-following validity [rad]
        trackHalfWidth = 1.5      % Allowed lateral path error before leaving track [m]
        
        % Simulation parameters
        maxSpeed      = 80       % Speed limiter [m/s] (~288 km/h)
    end
    
    methods
        function obj = VehicleManager(aero, suspension, powertrain, tire, track, chassis)
            % VEHICLEMANAGER Construct with all component objects
            %   VehicleManager(aero, suspension, powertrain, tire, track)
            %   VehicleManager(aero, suspension, powertrain, tire, track, chassis)
            
            obj.aero = aero;
            obj.suspension = suspension;
            obj.powertrain = powertrain;
            obj.tire = tire;
            obj.track = track;
            if nargin >= 6
                obj.chassis = chassis;
            end
        end

        function sanitizeSetup(obj)
            % SANITIZESETUP Reapply scalar physical limits to public setup fields.
            obj.totalMass = obj.totalMass;
            obj.wheelbase = obj.wheelbase;
            obj.trackWidth = obj.trackWidth;
            obj.cgHeight = obj.cgHeight;
            obj.yawInertia = obj.yawInertia;
            obj.airDensity = obj.airDensity;
            obj.staticFrontWeight = obj.staticFrontWeight;
            obj.brakeBiasFront = obj.brakeBiasFront;
            obj.brakeForceCoefficient = obj.brakeForceCoefficient;
            obj.rollingResistanceCoefficient = obj.rollingResistanceCoefficient;
            obj.yawRateDamping = obj.yawRateDamping;
            obj.lateralVelocityDamping = obj.lateralVelocityDamping;
            obj.maxSideslipAngle = obj.maxSideslipAngle;
            obj.trackHalfWidth = obj.trackHalfWidth;
            obj.maxSpeed = obj.maxSpeed;
        end

        function set.totalMass(obj, value)
            obj.totalMass = utils.positiveScalarOrDefault(value, 280);
        end

        function set.wheelbase(obj, value)
            obj.wheelbase = utils.positiveScalarOrDefault(value, 1.55);
        end

        function set.trackWidth(obj, value)
            obj.trackWidth = utils.positiveScalarOrDefault(value, 1.2);
        end

        function set.cgHeight(obj, value)
            obj.cgHeight = utils.nonnegativeScalarOrDefault(value, 0.28);
        end

        function set.yawInertia(obj, value)
            obj.yawInertia = utils.positiveScalarOrDefault(value, 85);
        end

        function set.airDensity(obj, value)
            obj.airDensity = utils.nonnegativeScalarOrDefault(value, 1.225);
        end

        function set.staticFrontWeight(obj, value)
            obj.staticFrontWeight = utils.unitScalarOrDefault(value, 0.45);
        end

        function set.brakeBiasFront(obj, value)
            obj.brakeBiasFront = utils.unitScalarOrDefault(value, 0.60);
        end

        function set.brakeForceCoefficient(obj, value)
            obj.brakeForceCoefficient = utils.nonnegativeScalarOrDefault(value, 0.70);
        end

        function set.rollingResistanceCoefficient(obj, value)
            obj.rollingResistanceCoefficient = utils.nonnegativeScalarOrDefault(value, 0.015);
        end

        function set.yawRateDamping(obj, value)
            obj.yawRateDamping = utils.nonnegativeScalarOrDefault(value, 25.0);
        end

        function set.lateralVelocityDamping(obj, value)
            obj.lateralVelocityDamping = utils.nonnegativeScalarOrDefault(value, 0.4);
        end

        function set.maxSideslipAngle(obj, value)
            obj.maxSideslipAngle = utils.nonnegativeScalarOrDefault(value, 0.18);
        end

        function set.trackHalfWidth(obj, value)
            obj.trackHalfWidth = utils.positiveScalarOrDefault(value, 1.5);
        end

        function set.maxSpeed(obj, value)
            obj.maxSpeed = utils.positiveScalarOrDefault(value, 80);
        end
    end
end
