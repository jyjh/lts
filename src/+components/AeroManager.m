classdef AeroManager < components.AeroComponent
    % AEROMANAGER Aggregates multiple AeroComponent objects
    % Computes total aero forces and aero balance from positioned components.
    % Also computes pitch angle from vehicle state for sub-components to use.
    %
    % Usage:
    %   mgr = AeroManager(wheelbase);
    %   mgr = mgr.addComponent(FrontWing(...));
    %   mgr = mgr.addComponent(RearWing(...));
    %   mgr = mgr.addComponent(UnderbodyFloor(...));
    %   totalDownforce = mgr.computeDownforce(vehicleState);
    
    properties
        components = {}       % Cell array of AeroComponent objects
        wheelbase  = 1.55     % Wheelbase [m] for moment calculations
        pitchStiffness = 0.002 % Pitch angle per g of longitudinal accel [rad/g]
                               % (determines how much the car pitches under ax)
    end
    
    methods
        function obj = AeroManager(wheelbase, pitchStiffness)
            % AEROMANAGER Construct with wheelbase and pitch stiffness
            if nargin >= 1
                obj.wheelbase = wheelbase;
            end
            if nargin >= 2
                obj.pitchStiffness = pitchStiffness;
            end
        end
        
        function obj = addComponent(obj, aeroComp)
            % ADDCOMPONENT Add an AeroComponent to the manager
            %   obj = obj.addComponent(FrontWing(...))
            obj.components{end+1} = aeroComp;
        end
        
        function obj = removeComponent(obj, name)
            % REMOVECOMPONENT Remove a component by name
            idx = strcmp({obj.components.name}, name);
            obj.components = obj.components(~idx);
        end
        
        function n = numComponents(obj)
            n = numel(obj.components);
        end
        
        function list = listComponents(obj)
            % LISTCOMPONENTS Print all managed components
            for i = 1:numel(obj.components)
                c = obj.components{i};
                fprintf('  [%d] %s | x=%.3f m, z=%.3f m\n', i, c.getName(), ...
                    c.getLongitudinalPosition(), c.getNominalHeight());
            end
        end
        
        %% ---- AeroComponent interface (aggregated) ----
        
        function F_downforce = computeDownforce(obj, vehicleState)
            % Sum downforce from all components
            stateWithPitch = obj.applyPitchToState(vehicleState);
            F_downforce = 0;
            for i = 1:numel(obj.components)
                F_downforce = F_downforce + obj.components{i}.computeDownforce(stateWithPitch);
            end
        end
        
        function F_drag = computeDrag(obj, vehicleState)
            % Sum drag from all components
            stateWithPitch = obj.applyPitchToState(vehicleState);
            F_drag = 0;
            for i = 1:numel(obj.components)
                F_drag = F_drag + obj.components{i}.computeDrag(stateWithPitch);
            end
        end
        
        function rho = getAirDensity(obj)
            % Return air density from first component (assumed uniform)
            if numel(obj.components) > 0
                rho = obj.components{1}.getAirDensity();
            else
                rho = 1.225;
            end
        end
        
        function balance = computeAeroBalance(obj, vehicleState)
            % COMPUTEAEROBALANCE Fraction of total downforce on front axle
            % Computed from moment balance about CG:
            %   balance = (moment_front / wheelbase) / totalDownforce
            %
            % A component at xPosition > 0 (ahead of CG) loads the front axle.
            % A component at xPosition < 0 (behind CG) loads the rear axle.
            %
            % Front axle load fraction = (0.5*wb + x_i) / wb for each component
            
            stateWithPitch = obj.applyPitchToState(vehicleState);
            totalDownforce = 0;
            frontAxleMoment = 0;  % Sum of F_i * (half_wb + x_i) / wb
            
            halfWB = obj.wheelbase / 2;
            
            for i = 1:numel(obj.components)
                Fi = obj.components{i}.computeDownforce(stateWithPitch);
                xi = obj.components{i}.getLongitudinalPosition();
                
                % Fraction of this force on the front axle
                %   xPosition positive = forward of CG
                %   Front axle is at +halfWB from CG
                %   Rear axle is at -halfWB from CG
                %   Front fraction = (halfWB + xi) / wheelbase
                frontFrac = (halfWB + xi) / obj.wheelbase;
                frontFrac = max(0, min(1, frontFrac));  % Clamp
                
                totalDownforce = totalDownforce + Fi;
                frontAxleMoment = frontAxleMoment + Fi * frontFrac;
            end
            
            if totalDownforce > 0
                balance = frontAxleMoment / totalDownforce;
            else
                balance = 0.5;  % Default 50/50
            end
        end
        
        function results = computePerComponent(obj, vehicleState)
            % COMPUTEPERCOMPONENT Get forces from each component individually
            % Returns struct array with name, downforce, drag, xPosition
            stateWithPitch = obj.applyPitchToState(vehicleState);
            n = numel(obj.components);
            results = struct('name', cell(1,n), ...
                             'downforce', zeros(1,n), ...
                             'drag', zeros(1,n), ...
                             'xPosition', zeros(1,n));
            for i = 1:n
                results(i).name = obj.components{i}.getName();
                results(i).downforce = obj.components{i}.computeDownforce(stateWithPitch);
                results(i).drag = obj.components{i}.computeDrag(stateWithPitch);
                results(i).xPosition = obj.components{i}.getLongitudinalPosition();
            end
        end
        
        function stateOut = applyPitchToState(obj, vehicleState)
            % APPLYPITCHTOSTATE Compute pitch angle from longitudinal accel
            % and inject it into a copy of the vehicle state
            %   pitchAngle = pitchStiffness * ax_in_g
            %   (positive = nose up = braking)
            %
            % Future: this is where track elevation could modify rideHeight
            stateOut = vehicleState;
            ax_g = vehicleState.ax / 9.81;
            stateOut.pitchAngle = obj.pitchStiffness * ax_g;
            % rideHeight remains unchanged (for future: add track elevation)
        end
    end
end