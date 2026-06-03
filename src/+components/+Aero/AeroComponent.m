classdef (Abstract) AeroComponent
    % AEROCOMPONENT Abstract interface for aerodynamic models
    % Each aero element has its own position, pitch sensitivity, and
    % computes forces based on full vehicle state (speed, pitch, ride height).
    %
    % Multiple AeroComponents are composed by AeroManager.
    
    properties
        name        = 'AeroComponent'  % Descriptive name
        xPosition   = 0    % Longitudinal position from CG [m], positive = forward
        zPosition   = 0    % Nominal height above ground [m]
        ClA         = 1.0  % Downforce coefficient * area [m^2]
        CdA         = 0.5  % Drag coefficient * area [m^2]
        pitchSensitivityClA = 0  % ClA change per radian of pitch [1/rad]
    end
    
    methods (Abstract)
        %   state.speed, state.pitchAngle, state.rideHeight used
        F_downforce = computeDownforce(obj, vehicleState)
        
        % Drag [N] computed from vehicle state
        F_drag = computeDrag(obj, vehicleState)
    end
    
    methods
        function obj = AeroComponent(name, xPos, zPos, ClA, CdA, pitchSensitivityClA)
            obj.name = name;
            obj.xPosition = xPos;
            obj.zPosition = zPos;
            obj.ClA = ClA;
            obj.CdA = CdA;
            obj.pitchSensitivityClA = pitchSensitivityClA;
        end

        function pos = getLongitudinalPosition(obj)
            % Distance from CG [m], positive = forward of CG
            pos = obj.xPosition;
        end
        
        function z = getNominalHeight(obj)
            % Nominal height above reference plane [m]
            z = obj.zPosition;
        end
        
        function n = getName(obj)
            n = obj.name;
        end
        
        function dz = computeHeightChange(obj, vehicleState)
            % COMPUTEHEIGHTCHANGE Local height change due to pitch
            % When the car pitches (nose up), a component ahead of CG
            % moves UP, and a component behind CG moves DOWN.
            %   dz = xPosition * sin(pitchAngle) ≈ xPosition * pitchAngle
            dz = obj.xPosition * vehicleState.pitchAngle;
        end
        
        function effectiveZ = computeEffectiveHeight(obj, vehicleState)
            % COMPUTEEFFECTIVEHEIGHT Actual height of this component
            % Accounts for: nominal z + local pitch displacement + global ride height
            effectiveZ = obj.zPosition ...
                   + obj.computeHeightChange(vehicleState) ...
                   + vehicleState.rideHeight;
        end
    end
end