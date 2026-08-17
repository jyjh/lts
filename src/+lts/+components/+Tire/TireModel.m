classdef (Abstract) TireModel
    % TIREMODEL Abstract interface for tire force models
    %
    % Defines the contract that any tire model must implement:
    %   - computeLateralForce(normalLoad, slipAngle, mu) → Fy [N]
    %   - computeLongitudinalForce(normalLoad, slipRatio, mu) → Fx [N]
    %   - getPeakFriction(normalLoad) → peakMu [-]
    %   - computeSlipRatioFromKinematics(cornerState, longitudinalSpeed) → kappa
    %   - updateWheelDynamics(cornerState, driveTorque, brakeTorque, dt)
    %   - updateCorner / updateAllCorners per-corner and batch evaluation
    %
    % The mu arguments are retained for source compatibility; the built-in
    % model derives grip entirely from tire data and ignores surface mu.
    %
    % Concrete implementations:
    %   - PacejkaTire — supported Pacejka Magic Formula model via MFeval

    properties (Abstract)
        FL  % lts.components.Tire.TireState front-left
        FR  % lts.components.Tire.TireState front-right
        RL  % lts.components.Tire.TireState rear-left
        RR  % lts.components.Tire.TireState rear-right
    end

    methods (Abstract)
        Fy = computeLateralForce(obj, normalLoad, slipAngle, mu)
        Fx = computeLongitudinalForce(obj, normalLoad, slipRatio, mu)
        peakMu = getPeakFriction(obj, normalLoad)
        kappa = computeSlipRatioFromKinematics(obj, cornerState, longitudinalSpeed)
        updateWheelDynamics(obj, cornerState, driveTorque, brakeTorque, dt)
    end
end
