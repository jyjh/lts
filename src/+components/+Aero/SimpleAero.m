classdef SimpleAero < components.Aero.AeroComponent
    % SIMPLEAERO Constant-coefficient aerodynamic model with position
    % Uses fixed CdA, ClA coefficients. Can optionally respond to pitch.
    % Useful as a generic aero element or for quick analysis.
    
    properties
        pitchSensitivityCdA = 0   % CdA change per radian of pitch [1/rad]
    end
    
    methods
        function obj = SimpleAero(xPosition, zPosition, ClA, CdA, pitchSensitivityClA, pitchSensitivityCdA)
            % SIMPLEAERO Construct with fixed parameters
            %   SimpleAero(name, xPosition, zPosition, ClA, CdA, pitchSensitivityClA, pitchSensitivityCdA)
            obj@components.Aero.AeroComponent("Simple Aero", xPosition, zPosition, ClA, CdA, pitchSensitivityClA);
            obj.pitchSensitivityCdA = pitchSensitivityCdA;
        end
        
        function F_downforce = computeDownforce(obj, vehicleState)
            % Downforce = 0.5 * rho * ClA_effective * v^2
            effectiveClA = obj.ClA * (1 + obj.pitchSensitivityClA * vehicleState.pitchAngle);
            effectiveClA = max(0, effectiveClA);  % Cannot produce lift (safety clamp)
            rho = vehicleState.vehicleManager.airDensity;
            F_downforce = 0.5 * rho * effectiveClA * vehicleState.speed^2;
        end
        
        function F_drag = computeDrag(obj, vehicleState)
            % Drag = 0.5 * rho * CdA_effective * v^2
            effectiveCdA = obj.CdA * (1 + obj.pitchSensitivityCdA * abs(vehicleState.pitchAngle));
            effectiveCdA = max(0, effectiveCdA);
            rho = vehicleState.vehicleManager.airDensity;
            F_drag = 0.5 * rho * effectiveCdA * vehicleState.speed^2;
        end
    end
end