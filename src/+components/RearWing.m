classdef RearWing < components.AeroComponent
    % REARWING Rear wing aerodynamic model
    % Positioned behind the rear axle. Moderate pitch sensitivity.
    % Typically has the highest CdA of all components due to high angle of attack.
    %
    % Pitch behavior:
    %   - Nose UP (braking) → rear squats → rear gets closer to ground → MORE downforce
    %   - Nose DOWN (accel) → rear rises → less ground effect → LESS downforce
    %   This is pitchSensitivityClA > 0 (positive: positive pitch increases ClA)
    
    properties
        ClA              = 1.1    % Downforce coefficient * area [m^2]
        CdA              = 0.55   % Drag coefficient * area [m^2]
        rho              = 1.225  % Air density [kg/m^3]
        pitchSensitivityClA = 3.0 % ClA change per radian of pitch (pos = gains DF on nose-up)
        heightSensitivity    = 0.15 % Fractional ClA change per cm of height deviation
        referenceHeight      = 0.30 % Design ride height [m]
    end
    
    methods
        function obj = RearWing(varargin)
            if nargin > 0
                for i = 1:2:nargin
                    if isprop(obj, varargin{i})
                        obj.(varargin{i}) = varargin{i+1};
                    end
                end
            end
        end
        
        function F_downforce = computeDownforce(obj, vehicleState)
            % Pitch effect: positive pitch (nose up) increases rear wing AoA
            pitchFactor = 1 + obj.pitchSensitivityClA * vehicleState.pitchAngle;
            
            % Height effect: rear wing moderately sensitive
            effectiveZ = obj.computeEffectiveHeight(vehicleState);
            dz = (effectiveZ - obj.referenceHeight) * 100;  % cm deviation
            heightFactor = 1 - obj.heightSensitivity * dz / 100;
            
            effectiveClA = obj.ClA * pitchFactor * heightFactor;
            effectiveClA = max(0, effectiveClA);
            F_downforce = 0.5 * obj.rho * effectiveClA * vehicleState.speed^2;
        end
        
        function F_drag = computeDrag(obj, vehicleState)
            pitchFactor = 1 + 0.3 * abs(obj.pitchSensitivityClA) * abs(vehicleState.pitchAngle);
            effectiveCdA = obj.CdA * pitchFactor;
            effectiveCdA = max(0, effectiveCdA);
            F_drag = 0.5 * obj.rho * effectiveCdA * vehicleState.speed^2;
        end
        
        function rho = getAirDensity(obj)
            rho = obj.rho;
        end
    end
end