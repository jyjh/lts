classdef WholeCarAero < lts.components.Aero.AeroComponent
    % WHOLECARAERO Single resultant aerodynamic model for the full vehicle.
    % Uses whole-car ClA/CdA and a center-of-pressure position.
    %
    % This is the default aero representation: instead of separate wings and
    % floor devices, the car produces one quadratic downforce term and one
    % quadratic drag term. AeroComponent.computeForces then splits the
    % downforce to front/rear axles by moment balance about the CG.

    methods
        function obj = WholeCarAero(xPosition, zPosition, ClA, CdA, pitchSensitivityClA)
            if nargin < 5
                pitchSensitivityClA = 0;
            end
            obj@lts.components.Aero.AeroComponent( ...
                "Whole Car Aero", xPosition, zPosition, ClA, CdA, pitchSensitivityClA);
        end

        function F_downforce = computeDownforce(obj, vehicleState)
            % Downforce = dynamic pressure * ClA.
            % Positive pitch sensitivity changes the effective ClA linearly
            % with chassis pitch; max(0, ...) prevents an unphysical sign flip.
            pitchFactor = 1 + obj.pitchSensitivityClA * vehicleState.pitchAngle;
            effectiveClA = max(0, obj.ClA * pitchFactor);
            rho = vehicleState.vehicleManager.airDensity;
            F_downforce = 0.5 * rho * effectiveClA * vehicleState.speed^2;
        end

        function F_drag = computeDrag(obj, vehicleState)
            % Drag = dynamic pressure * CdA and always opposes velocity in
            % lts.simulation.Simulator.computePlanarDynamics.
            rho = vehicleState.vehicleManager.airDensity;
            F_drag = 0.5 * rho * max(0, obj.CdA) * vehicleState.speed^2;
        end
    end
end
