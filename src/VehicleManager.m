classdef VehicleManager
    % VEHICLEMANAGER Configuration container for vehicle components and parameters
    % Holds references to all swappable components (aero, suspension, powertrain, tire, track)
    % and all non-changing vehicle parameters (mass, wheelbase, track width, etc).
    %
    % Simulation is handled by the Simulator class, driver decisions by DriverModel.
    
    properties
        % Swappable component objects
        aero        % components.Aero.AeroManager
        suspension  % components.Suspension.SuspensionManager
        powertrain  % components.PowertrainComponent
        tire        % components.TireModel
        track       % components.Track
        
        % Vehicle parameters
        totalMass     = 280      % Total mass with driver [kg]
        wheelbase     = 1.55     % Wheelbase [m]
        trackWidth    = 1.2      % Track width [m]
        cgHeight      = 0.28     % CG height [m]
        airDensity    = 1.225    % Air density [kg/m^3]
        staticFrontWeight = 0.48 % Static front weight distribution [0-1]
        
        % Simulation parameters
        dt            = 0.001    % Timestep [s]
        maxSpeed      = 40       % Speed limiter [m/s] (~144 km/h)
    end
    
    methods
        function obj = VehicleManager(aero, suspension, powertrain, tire, track, totalMass, dt, maxSpeed)
            % VEHICLEMANAGER Construct with all component objects
            %   VehicleManager(aero, suspension, powertrain, tire, track, totalMass, dt, maxSpeed)
            
            obj.aero = aero;
            obj.suspension = suspension;
            obj.powertrain = powertrain;
            obj.tire = tire;
            obj.track = track;
            obj.totalMass = totalMass;
            obj.dt = dt;
            obj.maxSpeed = maxSpeed;
        end
    end
end