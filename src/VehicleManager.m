classdef VehicleManager
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
        totalMass     = 280      % Total mass with driver [kg]
        wheelbase     = 1.55     % Wheelbase [m]
        trackWidth    = 1.2      % Track width [m]
        cgHeight      = 0.28     % CG height [m]
        airDensity    = 1.225    % Air density [kg/m^3]
        staticFrontWeight = 0.45 % Static front weight distribution [0-1]
        brakeBiasFront = 0.60    % Fraction of brake force commanded to front axle [0-1]
        brakeForceCoefficient = 0.70 % Hydraulic brake force capacity as fraction of normal load
        
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
    end
end
