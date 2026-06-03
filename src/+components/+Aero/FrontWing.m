classdef FrontWing < components.Aero.AeroComponent
    % FRONTWING Front wing aerodynamic model
    % Positioned ahead of the front axle. Highly sensitive to pitch/ride height
    % because it operates in ground effect with a large pitch moment arm.
    %
    % Pitch behavior:
    %   - Nose UP (braking) → increased front ride height → less ground effect → LESS downforce
    %   - Nose DOWN (accel) → decreased front ride height → more ground effect → MORE downforce
    %   This is pitchSensitivityClA < 0 (negative: positive pitch reduces ClA)
    
    properties
        heightSensitivity    = 0.3 % Fractional ClA change per cm of height deviation
    end
    
    methods
        function obj = FrontWing(xPosition, zPosition, ClA, CdA, pitchSensitivityClA, heightSensitivity)
            % FRONTWING Construct a front wing model
            %   FrontWing(name, xPosition, zPosition, ClA, CdA, pitchSensitivityClA, heightSensitivity)
            obj@components.Aero.AeroComponent("Front Wing", xPosition, zPosition, ClA, CdA, pitchSensitivityClA);
            obj.heightSensitivity = heightSensitivity;
        end
        
        function F_downforce = computeDownforce(obj, vehicleState)
            % Pitch effect: positive pitch (nose up) reduces front wing AoA
            pitchFactor = 1 + obj.pitchSensitivityClA * vehicleState.pitchAngle;
            
            % Height effect: front wing sensitive to ride height changes
            effectiveZ = obj.computeEffectiveHeight(vehicleState);
            dz = (effectiveZ - obj.zPosition) * 100;  % cm deviation from design height
            heightFactor = 1 - obj.heightSensitivity * dz / 100;
            
            effectiveClA = obj.ClA * pitchFactor * heightFactor;
            effectiveClA = max(0, effectiveClA);
            rho = vehicleState.vehicleManager.airDensity;
            F_downforce = 0.5 * rho * effectiveClA * vehicleState.speed^2;
        end
        
        function F_drag = computeDrag(obj, vehicleState)
            pitchFactor = 1 + 0.5 * abs(obj.pitchSensitivityClA) * abs(vehicleState.pitchAngle);
            effectiveCdA = obj.CdA * pitchFactor;
            effectiveCdA = max(0, effectiveCdA);
            rho = vehicleState.vehicleManager.airDensity;
            F_drag = 0.5 * rho * effectiveCdA * vehicleState.speed^2;
        end
    end
end