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
    
    properties (Abstract)
        FL  % components.Tire.TireState front-left
        FR  % components.Tire.TireState front-right
        RL  % components.Tire.TireState rear-left
        RR  % components.Tire.TireState rear-right
    end

    methods (Abstract)
        Fy = computeLateralForce(obj, normalLoad, slipAngle, mu)
        Fx = computeLongitudinalForce(obj, normalLoad, slipRatio, mu)
        peakMu = getPeakFriction(obj, normalLoad)
        kappa = computeSlipRatio(obj, cornerState, vehicleSpeed)
        updateWheelDynamics(obj, cornerState, driveTorque, brakeTorque, dt)
        updateAllFromState(obj, state, vehicleManager, cornerLoads, mu)
    end
end
