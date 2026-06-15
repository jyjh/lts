classdef AeroManager < components.Aero.AeroComponent
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
    %     forces.F_side      - Total aero side-drag force [N], body-y positive left
    %     forces.aeroYawMoment - Yaw moment from aero side drag [N*m]
    %     forces.aeroRollMoment - Roll moment from side drag height [N*m]
    %     forces.dragHeight  - Drag resultant height above CG [m]
    
    properties
        components = {}       % Cell array of AeroComponent objects
    end
    
    methods
        function obj = AeroManager()
            % AEROMANAGER Construct with no arguments
            obj@components.Aero.AeroComponent('AeroManager', 0, 0, 0, 0, 0);
        end

        function obj = set.components(obj, value)
            obj.components = components.Aero.AeroManager.filterValidComponents(value);
        end

        function F_downforce = computeDownforce(obj, vehicleState)
            % COMPUTEDOWNFORCE Total downforce from all managed components [N]
            F_downforce = 0;
            validComponents = components.Aero.AeroManager.filterValidComponents(obj.components);
            for i = 1:numel(validComponents)
                F_downforce = F_downforce + validComponents{i}.computeDownforce(vehicleState);
            end
        end

        function F_drag = computeDrag(obj, vehicleState)
            % COMPUTEDRAG Total drag from all managed components [N]
            F_drag = 0;
            validComponents = components.Aero.AeroManager.filterValidComponents(obj.components);
            for i = 1:numel(validComponents)
                F_drag = F_drag + validComponents{i}.computeDrag(vehicleState);
            end
        end
        
        function obj = addComponent(obj, aeroComp)
            % ADDCOMPONENT Add an AeroComponent to the manager
            %   obj = obj.addComponent(FrontWing(...))
            if components.Aero.AeroManager.isValidAeroComponent(aeroComp)
                obj.components{end+1} = aeroComp;
            end
        end
        
        function obj = removeComponent(obj, name)
            % REMOVECOMPONENT Remove a component by name
            obj.components = components.Aero.AeroManager.filterValidComponents(obj.components);
            removeIdx = false(size(obj.components));
            for i = 1:numel(obj.components)
                componentName = obj.components{i}.getName();
                removeIdx(i) = strcmp(char(componentName), char(name));
            end
            obj.components = obj.components(~removeIdx);
        end
        
        function n = numComponents(obj)
            n = numel(components.Aero.AeroManager.filterValidComponents(obj.components));
        end
        
        function listComponents(obj)
            % LISTCOMPONENTS Print all managed components
            validComponents = components.Aero.AeroManager.filterValidComponents(obj.components);
            for i = 1:numel(validComponents)
                c = validComponents{i};
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
            %     F_drag      - Total body-longitudinal drag force [N]
            %     F_side      - Total body-lateral aero drag force [N]
            %     aeroYawMoment - Yaw moment from aero side drag [N*m]
            %     aeroRollMoment - Roll moment from aero side drag [N*m]
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
            F_side = 0;
            aeroYawMoment = 0;
            aeroRollMoment = 0;
            dragMoment = 0;  % Sum of Di * zi for weighted average height
            validComponents = components.Aero.AeroManager.filterValidComponents(obj.components);
            
            for i = 1:numel(validComponents)
                comp = validComponents{i};
                
                % Component forces
                Fi = comp.computeDownforce(vehicleState);
                Di = comp.computeDrag(vehicleState);
                Yi = comp.computeSideDrag(vehicleState);
                xi = comp.getLongitudinalPosition();  % +forward of CG
                % Drag acts at the component's current height, not its setup
                % height. Pitch and heave therefore move the drag pitch moment
                % arm just as they move the aero element in the real car.
                zi = comp.computeEffectiveHeight(vehicleState);
                
                % Resolve downforce to axles via moment balance about CG.
                % Keep the signed equivalent loads for overhanging wings. A
                % front wing ahead of the front axle legitimately unloads the
                % rear axle in the equivalent two-support model; clamping would
                % preserve vertical force but lose the pitch moment.
                frontFrac = (b + xi) / wb;
                
                Fz_front = Fz_front + Fi * frontFrac;
                Fz_rear  = Fz_rear  + Fi * (1 - frontFrac);
                
                % Accumulate drag and height-weighted drag
                F_drag = F_drag + Di;
                F_side = F_side + Yi;
                aeroYawMoment = aeroYawMoment + xi * Yi;
                % Body-y side drag acts at the component height. Its roll
                % moment is not tire load transfer at the ground plane, so the
                % chassis consumes it as a separate signed moment about the CG.
                aeroRollMoment = aeroRollMoment - Yi * (zi - cgHeight);
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
            forces.F_side     = F_side;
            forces.aeroYawMoment = aeroYawMoment;
            forces.aeroRollMoment = aeroRollMoment;
            forces.dragHeight = dragHeight;
            forces.airSpeed = obj.computeAirSpeed(vehicleState);
            forces.aeroSideslipAngle = obj.computeAeroSideslipAngle(vehicleState);
        end
    end

    methods (Static, Access = private)
        function validComponents = filterValidComponents(componentList)
            if ~iscell(componentList)
                validComponents = {};
                return;
            end

            keep = false(size(componentList));
            for i = 1:numel(componentList)
                keep(i) = components.Aero.AeroManager.isValidAeroComponent( ...
                    componentList{i});
            end
            validComponents = componentList(keep);
        end

        function valid = isValidAeroComponent(candidate)
            valid = ~isempty(candidate) ...
                && ismethod(candidate, 'computeDownforce') ...
                && ismethod(candidate, 'computeDrag') ...
                && ismethod(candidate, 'computeSideDrag') ...
                && ismethod(candidate, 'getLongitudinalPosition') ...
                && ismethod(candidate, 'getNominalHeight') ...
                && ismethod(candidate, 'computeEffectiveHeight') ...
                && ismethod(candidate, 'getName');
        end
    end
end
