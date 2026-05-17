classdef FrontWing < components.AeroComponent
    % FRONTWING Front wing aerodynamic model
    % Positioned ahead of the front axle. Highly sensitive to pitch/ride height
    % because it operates in ground effect with a large pitch moment arm.
    %
    % Pitch behavior:
    %   - Nose UP (braking) → increased front ride height → less ground effect → LESS downforce
    %   - Nose DOWN (accel) → decreased front ride height → more ground effect → MORE downforce
    %   This is pitchSensitivityClA < 0 (negative: positive pitch reduces ClA)
    
    properties
        ClA              = 0.9    % Downforce coefficient * area [m^2]
        CdA              = 0.35   % Drag coefficient * area [m^2]
        rho              = 1.225  % Air density [kg/m^3]
        pitchSensitivityClA = -5.0 % ClA change per radian of pitch (neg = loses DF on nose-up)
        heightSensitivity    = 0.3 % Fractional ClA change per cm of height deviation
        referenceHeight      = 0.04 % Design ride height [m] for peak performance
    end
    
    methods
        function obj = FrontWing(varargin)
            if nargin > 0
                for i = 1:2:nargin
                    if isprop(obj, varargin{i})
                        obj.(varargin{i}) = varargin{i+1};
                    end
                end
            end
        end
        
        function F_downforce = computeDownforce(obj, vehicleState)
            % Pitch effect: positive pitch (nose up) reduces front wing AoA
            pitchFactor = 1 + obj.pitchSensitivityClA * vehicleState.pitchAngle;
            
            % Height effect: front wing sensitive to ride height changes
            effectiveZ = obj.computeEffectiveHeight(vehicleState);
            dz = (effectiveZ - obj.referenceHeight) * 100;  % cm deviation
            heightFactor = 1 - obj.heightSensitivity * dz / 100;
            
            effectiveClA = obj.ClA * pitchFactor * heightFactor;
            effectiveClA = max(0, effectiveClA);
            F_downforce = 0.5 * obj.rho * effectiveClA * vehicleState.speed^2;
        end
        
        function F_drag = computeDrag(obj, vehicleState)
            pitchFactor = 1 + 0.5 * abs(obj.pitchSensitivityClA) * abs(vehicleState.pitchAngle);
            effectiveCdA = obj.CdA * pitchFactor;
            effectiveCdA = max(0, effectiveCdA);
            F_drag = 0.5 * obj.rho * effectiveCdA * vehicleState.speed^2;
        end
        
        function rho = getAirDensity(obj)
            rho = obj.rho;
        end
    end
end