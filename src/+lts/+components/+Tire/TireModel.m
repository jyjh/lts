classdef (Abstract) TireModel
    % TIREMODEL Abstract interface for tire force models
    %
    % Defines the contract that the Simulator expects from a tire model.
    % The Simulator's per-step path calls updateAllCorners (batch) for the
    % fast path, falling back to updateCorner (single) when a tire model
    % does not provide the batch method. Wheel angular velocity is advanced
    % by updateWheelDynamics, called from the Simulator's wheel-contact
    % solve loop, with slip ratios derived via
    % computeSlipRatioFromKinematics.
    %
    % Standalone query methods (computeLateralForce, computeLongitudinalForce,
    % getPeakFriction) are used by scripts and diagnostics, not the main loop.
    %
    % The mu arguments are retained for source compatibility; the built-in
    % model derives grip entirely from tire data and ignores surface mu.
    %
    % Concrete implementations:
    %   - PacejkaTire — Pacejka Magic Formula model via MFeval

    properties (Abstract)
        FL  % lts.components.Tire.TireState front-left
        FR  % lts.components.Tire.TireState front-right
        RL  % lts.components.Tire.TireState rear-left
        RR  % lts.components.Tire.TireState rear-right
    end

    methods (Abstract)
        % --- Standalone queries (scripts, diagnostics) ---
        Fy = computeLateralForce(obj, normalLoad, slipAngle, mu)
        Fx = computeLongitudinalForce(obj, normalLoad, slipRatio, mu)
        peakMu = getPeakFriction(obj, normalLoad)
        % Peak of the pure-longitudinal |Fx|/Fz curve: the capability
        % measure for brake capacity and traction limits (the lateral peak
        % underestimates both). Not used by the per-step loop.
        peakMu = getPeakLongitudinalFriction(obj, normalLoad)

        % --- Simulator per-step contract ---
        % MFeval-consistent local-wheel slip ratio (reverse-capable).
        kappa = computeSlipRatioFromKinematics(obj, cornerState, longitudinalSpeed)

        % Integrate one wheel's angular velocity. inertia and
        % longitudinalSpeed are optional (default to obj.wheelInertia and
        % omega*R). The Simulator passes per-wheel reflected rotor inertia
        % on driven wheels.
        updateWheelDynamics(obj, cornerState, driveTorque, brakeTorque, dt, inertia, longitudinalSpeed)

        % Batch-evaluate all four corners. Slip values are steady-state
        % (kinematic); the implementation applies contact-patch relaxation
        % when dt > 0. relaxationMode is 'advance' (commit lagged state),
        % 'steady' (evaluate at ss, preserve lagged state), or 'hold'
        % (evaluate at previous lagged state).
        updateAllCorners(obj, Fz_FL, Fz_FR, Fz_RL, Fz_RR, ...
            slipAngle_FL, slipAngle_FR, slipAngle_RL, slipAngle_RR, ...
            kappa_FL, kappa_FR, kappa_RL, kappa_RR, ...
            camber_FL, camber_FR, camber_RL, camber_RR, dt, longSpeeds, ...
            surfaceMu, computePeakMu, relaxationMode)
    end
end
