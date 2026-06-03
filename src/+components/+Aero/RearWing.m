classdef RearWing < components.Aero.AeroComponent
    % REARWING Rear wing aerodynamic model
    % Positioned behind the rear axle. Moderate pitch sensitivity.
    % Typically has the highest CdA of all components due to high angle of attack.
    %
    % Pitch behavior:
    %   - Nose UP (braking) → rear squats → rear gets closer to ground → MORE downforce
    %   - Nose DOWN (accel) → rear rises → less ground effect → LESS downforce
    %   This is pitchSensitivityClA > 0 (positive: positive pitch increases ClA)
    
    properties
        heightSensitivity    = 0.15 % Fractional ClA change per cm of height deviation
    end
    
    methods
        function obj = RearWing(xPosition, zPosition, ClA, CdA, pitchSensitivityClA, heightSensitivity)
            % REARWING Construct a rear wing model
            %   RearWing(xPosition, zPosition, ClA, CdA, pitchSensitivityClA, heightSensitivity)
            obj@components.Aero.AeroComponent("Rear Wing", xPosition, zPosition, ClA, CdA, pitchSensitivityClA);
            obj.heightSensitivity = heightSensitivity;
        end
        
        function F_downforce = computeDownforce(obj, vehicleState)
            % Pitch effect: positive pitch (nose up) increases rear wing AoA
            pitchFactor = 1 + obj.pitchSensitivityClA * vehicleState.pitchAngle;
            
            % Height effect: rear wing moderately sensitive
            effectiveZ = obj.computeEffectiveHeight(vehicleState);
            dz = (effectiveZ - obj.zPosition) * 100;  % cm deviation from design height
            heightFactor = 1 - obj.heightSensitivity * dz / 100;
            
            effectiveClA = obj.ClA * pitchFactor * heightFactor;
            effectiveClA = max(0, effectiveClA);
            rho = vehicleState.vehicleManager.airDensity;
            F_downforce = 0.5 * rho * effectiveClA * vehicleState.speed^2;
        end
        
        function F_drag = computeDrag(obj, vehicleState)
            pitchFactor = 1 + 0.3 * abs(obj.pitchSensitivityClA) * abs(vehicleState.pitchAngle);
            effectiveCdA = obj.CdA * pitchFactor;
            effectiveCdA = max(0, effectiveCdA);
            rho = vehicleState.vehicleManager.airDensity;
            F_drag = 0.5 * rho * effectiveCdA * vehicleState.speed^2;
        end
    end
end