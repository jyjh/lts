classdef WholeCarAero < components.Aero.AeroComponent
    % WHOLECARAERO Single resultant aerodynamic model for the full vehicle.
    % Uses whole-car ClA/CdA and a center-of-pressure position.

    methods
        function obj = WholeCarAero(xPosition, zPosition, ClA, CdA, pitchSensitivityClA)
            if nargin < 5
                pitchSensitivityClA = 0;
            end
            obj@components.Aero.AeroComponent( ...
                "Whole Car Aero", xPosition, zPosition, ClA, CdA, pitchSensitivityClA);
        end

        function F_downforce = computeDownforce(obj, vehicleState)
            pitchFactor = 1 + obj.pitchSensitivityClA * vehicleState.pitchAngle;
            effectiveClA = max(0, obj.ClA * pitchFactor);
            rho = vehicleState.vehicleManager.airDensity;
            F_downforce = 0.5 * rho * effectiveClA * vehicleState.speed^2;
        end

        function F_drag = computeDrag(obj, vehicleState)
            rho = vehicleState.vehicleManager.airDensity;
            F_drag = 0.5 * rho * max(0, obj.CdA) * vehicleState.speed^2;
        end
    end
end
