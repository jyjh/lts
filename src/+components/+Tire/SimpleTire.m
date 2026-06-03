classdef SimpleTire < components.Tire.TireModel
    % SIMPLETIRE Linear tire model with saturation
    % Uses a linear region up to a peak slip, then saturates
    % Includes basic load sensitivity (friction decreases with load)
    
    properties
        corneringStiffness = 800   % Cornering stiffness per tire [N/deg] (per side of axle)
        longitudinalStiffness = 10000 % Longitudinal stiffness per tire [N/unit slip]
        peakMuLat          = 1.8   % Peak lateral friction coefficient
        peakMuLong         = 1.8   % Peak longitudinal friction coefficient
        peakSlipAngle      = 5.0   % Slip angle at peak lateral force [deg]
        peakSlipRatio      = 0.10  % Slip ratio at peak longitudinal force
        loadSensitivityExp = -0.1  % Load sensitivity exponent (negative = mu drops with load)
    end
    
    methods
        function obj = SimpleTire(corneringStiffness, longitudinalStiffness, peakMuLat, loadSensitivityExp)
            % SIMPLETIRE Construct with fixed parameters
            %   SimpleTire(corneringStiffness, longitudinalStiffness, peakMuLat, loadSensitivityExp)
            obj.corneringStiffness = corneringStiffness;
            obj.longitudinalStiffness = longitudinalStiffness;
            obj.peakMuLat = peakMuLat;
            obj.loadSensitivityExp = loadSensitivityExp;
        end
        
        function Fy = computeLateralForce(obj, normalLoad, slipAngle, mu)
            % Compute lateral force using linear-saturation model
            %   Fy = min(Calpha * alpha, mu * Fz)
            if normalLoad <= 0
                Fy = 0;
                return;
            end
            
            % Adjust friction for load sensitivity
            % Reference load = 1500 N (typical FSAE corner weight)
            refLoad = 1500;
            adjustedMu = mu * (normalLoad / refLoad)^obj.loadSensitivityExp;
            
            % Linear force
            Fy_linear = obj.corneringStiffness * abs(slipAngle);
            
            % Maximum force (saturation)
            Fy_max = adjustedMu * normalLoad;
            
            % Take minimum and apply sign
            Fy = sign(slipAngle) * min(Fy_linear, Fy_max);
        end
        
        function Fx = computeLongitudinalForce(obj, normalLoad, slipRatio, mu)
            % Compute longitudinal force using linear-saturation model
            if normalLoad <= 0
                Fx = 0;
                return;
            end
            
            refLoad = 1500;
            adjustedMu = mu * (normalLoad / refLoad)^obj.loadSensitivityExp;
            
            % Linear force
            Fx_linear = obj.longitudinalStiffness * abs(slipRatio);
            
            % Maximum force (saturation)
            Fx_max = adjustedMu * normalLoad;
            
            % Take minimum and apply sign
            Fx = sign(slipRatio) * min(Fx_linear, Fx_max);
        end
        
        function mu = getPeakFriction(obj, normalLoad)
            % Get peak friction coefficient adjusted for load
            refLoad = 1500;
            mu = obj.peakMuLat * (normalLoad / refLoad)^obj.loadSensitivityExp;
        end
    end
end