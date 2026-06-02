classdef (Abstract) TireModel
    % TIREMODEL Abstract interface for tire models
    % Provides tire force computation based on load, slip, and friction
    
    methods (Abstract)
        % Lateral tire force [N] given normal load, slip angle, and surface friction
        Fy = computeLateralForce(obj, normalLoad, slipAngle, mu)
        
        % Longitudinal tire force [N] given normal load, slip ratio, and surface friction
        Fx = computeLongitudinalForce(obj, normalLoad, slipRatio, mu)
        
        % Peak friction coefficient for given load
        peakMu = getPeakFriction(obj, normalLoad)
        
        % Combined slip force (optional for simple models)
        % Default implementation uses friction ellipse
    end
end