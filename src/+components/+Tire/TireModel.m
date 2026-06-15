classdef (Abstract) TireModel < handle
    % TIREMODEL Abstract interface for tire force models
    %
    % Defines the contract that any tire model must implement:
    %   - computeLateralForce(normalLoad, slipAngle, mu) → Fy [N]
    %   - computeLongitudinalForce(normalLoad, slipRatio, mu) → Fx [N]
    %   - getPeakFriction(normalLoad) → peakMu [-]
    %   - updateWheelDynamics(...) → integrated wheel-speed telemetry
    % Optional richer models may also expose getPeakLongitudinalFriction()
    % when longitudinal and lateral grip peaks differ.
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
        omegaTelemetry = updateWheelDynamics(obj, cornerState, driveTorque, brakeTorque, dt)
        updateAllFromState(obj, state, vehicleManager, cornerLoads, mu, dt)
    end

    methods (Access = protected)
        function omegaMean = computeMeanPositiveAngularVelocity(~, omegaBeforeRaw, omegaAfterRaw)
            % COMPUTEMEANPOSITIVEANGULARVELOCITY Integrate forward wheel speed.
            %
            % Wheel dynamics use explicit Euler plus a zero-speed clamp. If the
            % unclamped solution crosses below zero, the wheel was stopped for
            % only part of the timestep. The mean nonnegative speed is then the
            % triangular area under omega(t) divided by dt, not simply
            % 0.5*(omega_before + 0).
            omegaBefore = max(omegaBeforeRaw, 0);
            omegaAfter = max(omegaAfterRaw, 0);
            if omegaBefore > 0 && omegaAfterRaw < 0
                activeFraction = omegaBefore / max(omegaBefore - omegaAfterRaw, eps);
                omegaMean = 0.5 * omegaBefore * activeFraction;
            else
                omegaMean = 0.5 * (omegaBefore + omegaAfter);
            end
            omegaMean = max(omegaMean, 0);
        end
    end
end
