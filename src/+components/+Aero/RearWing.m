classdef RearWing < components.Aero.AeroComponent
    % REARWING Rear wing aerodynamic model
    % Positioned behind the rear axle. Moderate pitch sensitivity.
    % Typically has the highest CdA of all components due to high angle of attack.
    %
    % Pitch behavior:
    %   - Nose UP (acceleration squat) → rear gets closer to ground → MORE downforce
    %   - Nose DOWN (braking dive) → rear rises → LESS downforce
    %   This is pitchSensitivityClA > 0 (positive: positive pitch increases ClA)
    
    properties
        heightSensitivity    = 0.005 % Fractional ClA change per cm of height deviation
    end

    methods
        function obj = set.heightSensitivity(obj, value)
            obj.heightSensitivity = utils.nonnegativeScalarOrDefault(value, 0.005);
        end

        function obj = RearWing(xPosition, zPosition, ClA, CdA, pitchSensitivityClA, heightSensitivity)
            % REARWING Construct a rear wing model
            %   RearWing()
            %   RearWing(xPosition, zPosition, ClA, CdA, pitchSensitivityClA, heightSensitivity)
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
            if nargin < 6 || isempty(heightSensitivity)
                heightSensitivity = 0.005;
            end
            obj@components.Aero.AeroComponent("Rear Wing", xPosition, zPosition, ClA, CdA, pitchSensitivityClA);
            obj.heightSensitivity = utils.nonnegativeScalarOrDefault( ...
                heightSensitivity, obj.heightSensitivity);
        end
        
        function F_downforce = computeDownforce(obj, vehicleState)
            % Pitch effect: positive pitch (nose up) increases rear wing AoA
            pitchFactor = 1 + obj.pitchSensitivityClA * vehicleState.pitchAngle;
            
            % Height effect: rear wing moderately sensitive.
            % dz is already in centimeters, so heightSensitivity is applied
            % directly as fractional ClA change per centimeter.
            effectiveZ = obj.computeEffectiveHeight(vehicleState);
            dz = (effectiveZ - obj.zPosition) * 100;  % cm deviation from design height
            heightFactor = 1 - obj.heightSensitivity * dz;
            
            % Pitch and height factors are independent losses. If either
            % goes past zero outside the valid aero map, it should stall the
            % element instead of multiplying two negatives back into downforce.
            effectiveClA = obj.ClA * max(0, pitchFactor) * max(0, heightFactor);
            F_downforce = obj.computeDownforceFromClA(vehicleState, effectiveClA);
        end
        
        function F_drag = computeDrag(obj, vehicleState)
            pitchFactor = 1 + 0.3 * abs(obj.pitchSensitivityClA) * abs(vehicleState.pitchAngle);
            effectiveCdA = obj.CdA * pitchFactor;
            F_drag = obj.computeLongitudinalDragFromCdA(vehicleState, effectiveCdA);
        end

        function F_side = computeSideDrag(obj, vehicleState)
            pitchFactor = 1 + 0.3 * abs(obj.pitchSensitivityClA) * abs(vehicleState.pitchAngle);
            effectiveCdA = obj.CdA * pitchFactor;
            F_side = obj.computeLateralDragFromCdA(vehicleState, effectiveCdA);
        end
    end
end
