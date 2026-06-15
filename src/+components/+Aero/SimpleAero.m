classdef SimpleAero < components.Aero.AeroComponent
    % SIMPLEAERO Constant-coefficient aerodynamic model with position
    % Uses fixed CdA, ClA coefficients. Can optionally respond to pitch.
    % Useful as a generic aero element or for quick analysis.
    
    properties
        pitchSensitivityCdA = 0   % CdA change per radian of pitch [1/rad]
    end

    methods
        function obj = set.pitchSensitivityCdA(obj, value)
            obj.pitchSensitivityCdA = utils.scalarOrDefault(value, 0);
        end

        function obj = SimpleAero(xPosition, zPosition, ClA, CdA, pitchSensitivityClA, pitchSensitivityCdA)
            % SIMPLEAERO Construct with fixed parameters
            %   SimpleAero()
            %   SimpleAero(xPosition, zPosition, ClA, CdA, pitchSensitivityClA, pitchSensitivityCdA)
            if nargin < 1 || isempty(xPosition)
                xPosition = 0;
            end
            if nargin < 2 || isempty(zPosition)
                zPosition = 0;
            end
            if nargin < 3 || isempty(ClA)
                ClA = 1.0;
            end
            if nargin < 4 || isempty(CdA)
                CdA = 0.5;
            end
            if nargin < 5 || isempty(pitchSensitivityClA)
                pitchSensitivityClA = 0;
            end
            if nargin < 6 || isempty(pitchSensitivityCdA)
                pitchSensitivityCdA = 0;
            end
            obj@components.Aero.AeroComponent("Simple Aero", xPosition, zPosition, ClA, CdA, pitchSensitivityClA);
            obj.pitchSensitivityCdA = utils.scalarOrDefault( ...
                pitchSensitivityCdA, obj.pitchSensitivityCdA);
        end
        
        function F_downforce = computeDownforce(obj, vehicleState)
            % Downforce = q * ClA_effective, with q from total relative airspeed.
            effectiveClA = obj.ClA * (1 + obj.pitchSensitivityClA * vehicleState.pitchAngle);
            effectiveClA = max(0, effectiveClA);  % Cannot produce lift (safety clamp)
            F_downforce = obj.computeDownforceFromClA(vehicleState, effectiveClA);
        end
        
        function F_drag = computeDrag(obj, vehicleState)
            % Drag is wind-aligned, then projected onto the body-x axis.
            effectiveCdA = obj.CdA * (1 + obj.pitchSensitivityCdA * abs(vehicleState.pitchAngle));
            F_drag = obj.computeLongitudinalDragFromCdA(vehicleState, effectiveCdA);
        end

        function F_side = computeSideDrag(obj, vehicleState)
            % Lateral aero drag uses the same CdA as body-x drag, projected
            % against the relative wind's body-y component.
            effectiveCdA = obj.CdA * (1 + obj.pitchSensitivityCdA * abs(vehicleState.pitchAngle));
            F_side = obj.computeLateralDragFromCdA(vehicleState, effectiveCdA);
        end
    end
end
