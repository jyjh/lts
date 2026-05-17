classdef UnderbodyFloor < components.AeroComponent
    % UNDERBODYFLOOR Floor / diffuser / underbody aerodynamic model
    % Positioned near the CG. VERY highly sensitive to ride height (ground effect).
    % The floor is the most ride-height-dependent aero device on an FSAE car.
    %
    % Pitch behavior:
    %   - Nose UP → floor leading edge rises → air escapes under splitter → LESS downforce
    %   - Nose DOWN → floor leading edge drops → stronger underbody seal → MORE downforce
    %   BUT also: too low = stall / porpoising
    %   Net effect: negative pitch sensitivity (loses DF on nose-up)
    %
    % Height behavior:
    %   Downforce increases as floor gets closer to ground, up to a stalling limit.
    %   Modeled as an exponential relationship.

    properties
        ClA              = 0.8    % Downforce coefficient * area [m^2]
        CdA              = 0.10   % Drag coefficient * area [m^2] (floor is low-drag)
        rho              = 1.225  % Air density [kg/m^3]
        pitchSensitivityClA = -8.0 % Very sensitive to pitch (ground effect)
        referenceHeight      = 0.035 % Design ride height [m] for floor
        stallHeight          = 0.015 % Below this height, floor stalls [m]
        heightExponent       = 0.6   % Controls ground-effect sensitivity curve
    end
    
    methods
        function obj = UnderbodyFloor(varargin)
            if nargin > 0
                for i = 1:2:nargin
                    if isprop(obj, varargin{i})
                        obj.(varargin{i}) = varargin{i+1};
                    end
                end
            end
        end
        
        function F_downforce = computeDownforce(obj, vehicleState)
            % Pitch effect
            pitchFactor = 1 + obj.pitchSensitivityClA * vehicleState.pitchAngle;
            
            % Height effect: exponential ground-effect model
            % Downforce scales as (referenceHeight / effectiveHeight)^exponent
            % Below stall height, downforce drops sharply
            effectiveZ = obj.computeEffectiveHeight(vehicleState);
            effectiveZ = max(effectiveZ, 0.005);  % Prevent division by zero
            
            if effectiveZ < obj.stallHeight
                % Stalling region: downforce drops rapidly
                stallFactor = (effectiveZ / obj.stallHeight)^2;
            else
                % Normal ground effect region
                stallFactor = 1.0;
            end
            
            heightFactor = stallFactor * (obj.referenceHeight / effectiveZ)^obj.heightExponent;
            
            % Clamp height factor to reasonable range
            heightFactor = max(0, min(heightFactor, 3.0));
            
            effectiveClA = obj.ClA * pitchFactor * heightFactor;
            effectiveClA = max(0, effectiveClA);
            F_downforce = 0.5 * obj.rho * effectiveClA * vehicleState.speed^2;
        end
        
        function F_drag = computeDrag(obj, vehicleState)
            % Floor drag is mostly from suction-induced pressure drag
            % Increases slightly with more downforce
            pitchFactor = 1 + 0.2 * abs(obj.pitchSensitivityClA) * abs(vehicleState.pitchAngle);
            effectiveCdA = obj.CdA * pitchFactor;
            effectiveCdA = max(0, effectiveCdA);
            F_drag = 0.5 * obj.rho * effectiveCdA * vehicleState.speed^2;
        end
        
        function rho = getAirDensity(obj)
            rho = obj.rho;
        end
    end
end