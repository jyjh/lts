classdef AeroManager
    % AEROMANAGER Aggregates multiple AeroComponent objects
    % Computes total aero forces and aero balance from positioned components.
    % Usage:
    %   mgr = AeroManager();
    %   mgr.addComponent(FrontWing(...));
    %   mgr.addComponent(RearWing(...));
    %   totalDownforce = mgr.computeDownforce(vehicleState);
    
    properties
        components = {}       % Cell array of AeroComponent objects
    end
    
    methods
        function obj = AeroManager()
            % AEROMANAGER Construct with no arguments
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
        
        function listComponents(obj)
            % LISTCOMPONENTS Print all managed components
            for i = 1:numel(obj.components)
                c = obj.components{i};
                fprintf('  [%d] %s | x=%.3f m, z=%.3f m\n', i, c.getName(), ...
                    c.getLongitudinalPosition(), c.getNominalHeight());
            end
        end
        
        %% ---- Aggregated force computations ----
        
        function F_downforce = computeDownforce(obj, vehicleState)
            % Sum downforce from all components
            F_downforce = 0;
            for i = 1:numel(obj.components)
                F_downforce = F_downforce + obj.components{i}.computeDownforce(vehicleState);
            end
        end
        
        function F_drag = computeDrag(obj, vehicleState)
            % Sum drag from all components
            F_drag = 0;
            for i = 1:numel(obj.components)
                F_drag = F_drag + obj.components{i}.computeDrag(vehicleState);
            end
        end
        
        function balance = computeAeroBalance(obj, vehicleState)
            % COMPUTEAEROBALANCE Fraction of total downforce on front axle
            % Uses static weight distribution to locate CG between axles,
            % then resolves each component's force through the axle contact patches.
            
            totalDownforce = 0;
            frontAxleMoment = 0;
            
            wb = vehicleState.vehicleManager.wheelbase;
            frontWeightFrac = vehicleState.vehicleManager.suspension.getStaticWeightDistribution();
            b = wb * frontWeightFrac;  % distance from CG to rear axle [m]
            
            for i = 1:numel(obj.components)
                Fi = obj.components{i}.computeDownforce(vehicleState);
                xi = obj.components{i}.getLongitudinalPosition();
                
                frontFrac = (b + xi) / wb;
                frontFrac = max(0, min(1, frontFrac));
                
                totalDownforce = totalDownforce + Fi;
                frontAxleMoment = frontAxleMoment + Fi * frontFrac;
            end
            
            if totalDownforce > 0
                balance = frontAxleMoment / totalDownforce;
            else
                balance = 0.5;
            end
        end
        
        function results = computePerComponent(obj, vehicleState)
            % COMPUTEPERCOMPONENT Get forces from each component individually
            n = numel(obj.components);
            results = struct('name', cell(1,n), ...
                             'downforce', zeros(1,n), ...
                             'drag', zeros(1,n));
            for i = 1:n
                results(i).name = obj.components{i}.getName();
                results(i).downforce = obj.components{i}.computeDownforce(vehicleState);
                results(i).drag = obj.components{i}.computeDrag(vehicleState);
            end
        end
    end
end