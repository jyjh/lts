classdef AeroManager < lts.components.Aero.AeroComponent
    % AEROMANAGER Aggregates multiple AeroComponent objects
    % Resolves each component's downforce to front/rear axle loads via
    % moment balance, and computes the drag-weighted center of pressure.
    %
    % Usage:
    %   mgr = AeroManager();
    %   mgr = mgr.addComponent(FrontWing(...));
    %   mgr = mgr.addComponent(RearWing(...));
    %   forces = mgr.computeForces(vehicleState);
    %     forces.Fz_front    - Downforce resolved to front axle [N]
    %     forces.Fz_rear     - Downforce resolved to rear axle [N]
    %     forces.F_drag      - Total drag force [N]
    %     forces.dragHeight    - Drag resultant height above ground [m]
    %     forces.dragXPosition - Drag resultant position from CG [m]
    
    properties
        components = {}       % Cell array of AeroComponent objects
    end
    
    methods
        function obj = AeroManager()
            % AEROMANAGER Construct with no arguments
            obj@lts.components.Aero.AeroComponent('AeroManager', 0, 0, 0, 0, 0);
        end

        function F_downforce = computeDownforce(obj, vehicleState)
            % COMPUTEDOWNFORCE Total downforce from all managed components [N]
            F_downforce = 0;
            for i = 1:numel(obj.components)
                F_downforce = F_downforce + obj.components{i}.computeDownforce(vehicleState);
            end
        end

        function F_drag = computeDrag(obj, vehicleState)
            % COMPUTEDRAG Total drag from all managed components [N]
            F_drag = 0;
            for i = 1:numel(obj.components)
                F_drag = F_drag + obj.components{i}.computeDrag(vehicleState);
            end
        end
        
        function obj = addComponent(obj, aeroComp)
            % ADDCOMPONENT Add an AeroComponent to the manager
            %   obj = obj.addComponent(FrontWing(...))
            obj.components{end+1} = aeroComp;
        end
        
        function obj = removeComponent(obj, name)
            % REMOVECOMPONENT Remove a component by name
            idx = cellfun(@(component) strcmp(component.name, name), obj.components);
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
            %     dragHeight    - Absolute height of drag resultant [m]
            %     dragXPosition - Longitudinal drag position from CG [m]
            %
            %   Each component's downforce is split between front and rear
            %   axles using moment balance about the CG:
            %     Fi_front = Fi * (b + xi) / wb
            %     Fi_rear  = Fi * (a - xi) / wb
            %   where xi is the component's longitudinal position from CG
            %   (positive forward), a = CG-to-front-axle, b = CG-to-rear-axle.
            %
            %   Drag height and longitudinal position are drag-force-weighted
            %   resultants. Height is absolute above the ground reference;
            %   longitudinal position is relative to the CG.
            
            wb = vehicleState.vehicleManager.wheelbase;
            frontWeightFrac = vehicleState.vehicleManager.staticFrontWeight;
            
            b = wb * frontWeightFrac;         % CG to rear axle [m]
            
            Fz_front = 0;
            Fz_rear = 0;
            F_drag = 0;
            dragHeightMoment = 0;  % Sum of Di * zi
            dragXMoment = 0;       % Sum of Di * xi
            
            for i = 1:numel(obj.components)
                comp = obj.components{i};
                
                % Component forces
                Fi = comp.computeDownforce(vehicleState);
                Di = comp.computeDrag(vehicleState);
                xi = comp.getLongitudinalPosition();  % +forward of CG
                zi = comp.computeEffectiveHeight(vehicleState);  % height above ground
                if isempty(zi) || ~isfinite(zi)
                    zi = comp.getNominalHeight();
                end
                
                % Resolve downforce to axles via moment balance about CG
                frontFrac = (b + xi) / wb;
                
                Fz_front = Fz_front + Fi * frontFrac;
                Fz_rear  = Fz_rear  + Fi * (1 - frontFrac);
                
                % Accumulate drag and height-weighted drag
                F_drag = F_drag + Di;
                dragHeightMoment = dragHeightMoment + Di * zi;
                dragXMoment = dragXMoment + Di * xi;
            end
            
            % Drag resultant center of pressure.
            if F_drag > 0
                dragHeight = dragHeightMoment / F_drag;
                dragXPosition = dragXMoment / F_drag;
            else
                dragHeight = 0;
                dragXPosition = 0;
            end
            
            forces.Fz_front   = Fz_front;
            forces.Fz_rear    = Fz_rear;
            forces.F_drag     = F_drag;
            forces.dragHeight = dragHeight;
            forces.dragXPosition = dragXPosition;
        end
    end
end
