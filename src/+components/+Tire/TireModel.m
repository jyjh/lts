classdef (Abstract) TireModel
    % TIREMODEL Abstract interface for tire force models
    %
    % Defines the contract that any tire model must implement:
    %   - computeLateralForce(normalLoad, slipAngle, mu) → Fy [N]
    %   - computeLongitudinalForce(normalLoad, slipRatio, mu) → Fx [N]
    %   - getPeakFriction(normalLoad) → peakMu [-]
    %
    % Concrete implementations:
    %   - SimpleTire  — linear tire with constant friction
    %   - PacejkaTire — Pacejka Magic Formula via MFeval (.tir file)
    
    methods (Abstract)
        Fy = computeLateralForce(obj, normalLoad, slipAngle, mu)
        Fx = computeLongitudinalForce(obj, normalLoad, slipRatio, mu)
        peakMu = getPeakFriction(obj, normalLoad)
    end
end