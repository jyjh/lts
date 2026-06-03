classdef UnderbodyFloor < components.Aero.AeroComponent
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
        stallHeight          = 0.015 % Below this height, floor stalls [m]
        heightExponent       = 0.6   % Controls ground-effect sensitivity curve
    end
    
    methods
        function obj = UnderbodyFloor(xPosition, zPosition, ClA, CdA, pitchSensitivityClA, stallHeight, heightExponent)
            % UNDERBODYFLOOR Construct a floor/diffuser model
            %   UnderbodyFloor(name, xPosition, zPosition, ClA, CdA, pitchSensitivityClA, stallHeight, heightExponent)
            obj@components.Aero.AeroComponent("Underbody Floor", xPosition, zPosition, ClA, CdA, pitchSensitivityClA);
            obj.stallHeight = stallHeight;
            obj.heightExponent = heightExponent;
        end
        
        function F_downforce = computeDownforce(obj, vehicleState)
            % Pitch effect
            pitchFactor = 1 + obj.pitchSensitivityClA * vehicleState.pitchAngle;
            
            % Height effect: exponential ground-effect model
            % Downforce scales as (zPosition / effectiveHeight)^exponent
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
            
            heightFactor = stallFactor * (obj.zPosition / effectiveZ)^obj.heightExponent;
            
            % Clamp height factor to reasonable range
            heightFactor = max(0, min(heightFactor, 3.0));
            
            effectiveClA = obj.ClA * pitchFactor * heightFactor;
            effectiveClA = max(0, effectiveClA);
            rho = vehicleState.vehicleManager.airDensity;
            F_downforce = 0.5 * rho * effectiveClA * vehicleState.speed^2;
        end
        
        function F_drag = computeDrag(obj, vehicleState)
            % Floor drag is mostly from suction-induced pressure drag
            % Increases slightly with more downforce
            pitchFactor = 1 + 0.2 * abs(obj.pitchSensitivityClA) * abs(vehicleState.pitchAngle);
            effectiveCdA = obj.CdA * pitchFactor;
            effectiveCdA = max(0, effectiveCdA);
            rho = vehicleState.vehicleManager.airDensity;
            F_drag = 0.5 * rho * effectiveCdA * vehicleState.speed^2;
        end
    end
end