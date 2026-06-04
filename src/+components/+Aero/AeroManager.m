classdef AeroManager
    % AEROMANAGER Aggregates multiple AeroComponent objects
    % Resolves each component's downforce to front/rear axle loads via
    % moment balance, and computes total drag with its effective height
    % above the CG.
    %
    % Usage:
    %   mgr = AeroManager();
    %   mgr = mgr.addComponent(FrontWing(...));
    %   mgr = mgr.addComponent(RearWing(...));
    %   forces = mgr.computeForces(vehicleState);
    %     forces.Fz_front    - Downforce resolved to front axle [N]
    %     forces.Fz_rear     - Downforce resolved to rear axle [N]
    %     forces.F_drag      - Total drag force [N]
    %     forces.dragHeight  - Drag resultant height above CG [m]
    
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
        
        %% ---- Main force computation ----
        
        function forces = computeForces(obj, vehicleState)
            % COMPUTEFORCES Resolve all component forces to axle loads
            %   forces = obj.computeForces(vehicleState)
            %
            %   Returns a struct with:
            %     Fz_front    - Downforce on front axle [N]
            %     Fz_rear     - Downforce on rear axle [N]
            %     F_drag      - Total drag force [N]
            %     dragHeight  - Height of drag resultant above CG [m]
            %
            %   Each component's downforce is split between front and rear
            %   axles using moment balance about the CG:
            %     Fi_front = Fi * (b + xi) / wb
            %     Fi_rear  = Fi * (a - xi) / wb
            %   where xi is the component's longitudinal position from CG
            %   (positive forward), a = CG-to-front-axle, b = CG-to-rear-axle.
            %
            %   Drag height is the weighted-average component height minus
            %   the CG height.
            
            wb = vehicleState.vehicleManager.wheelbase;
            cgHeight = vehicleState.vehicleManager.cgHeight;
            frontWeightFrac = vehicleState.vehicleManager.staticFrontWeight;
            
            a = wb * (1 - frontWeightFrac);  % CG to front axle [m]
            b = wb * frontWeightFrac;         % CG to rear axle [m]
            
            Fz_front = 0;
            Fz_rear = 0;
            F_drag = 0;
            dragMoment = 0;  % Sum of Di * zi for weighted average height
            
            for i = 1:numel(obj.components)
                comp = obj.components{i};
                
                % Component forces
                Fi = comp.computeDownforce(vehicleState);
                Di = comp.computeDrag(vehicleState);
                xi = comp.getLongitudinalPosition();  % +forward of CG
                zi = comp.getNominalHeight();          % height above ground
                
                % Resolve downforce to axles via moment balance about CG
                frontFrac = (b + xi) / wb;
                frontFrac = max(0, min(1, frontFrac));
                
                Fz_front = Fz_front + Fi * frontFrac;
                Fz_rear  = Fz_rear  + Fi * (1 - frontFrac);
                
                % Accumulate drag and height-weighted drag
                F_drag = F_drag + Di;
                dragMoment = dragMoment + Di * zi;
            end
            
            % Drag resultant height above CG
            if F_drag > 0
                dragHeight = dragMoment / F_drag - cgHeight;
            else
                dragHeight = 0;
            end
            
            forces.Fz_front   = Fz_front;
            forces.Fz_rear    = Fz_rear;
            forces.F_drag     = F_drag;
            forces.dragHeight = dragHeight;
        end
    end
end