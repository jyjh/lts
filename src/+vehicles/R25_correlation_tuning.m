function cfg = R25_correlation_tuning(cfg)
    % R25_CORRELATION_TUNING Overlay for lap5/raw correlation work.
    %
    % This intentionally keeps vehicles.R25 as the base truth and only applies
    % correlation-specific assumptions that are still under investigation.

    if nargin < 1 || isempty(cfg)
        cfg = vehicles.R25();
    end

    cfg.name = string(cfg.name) + "_corrTune";

    % Logged speed vs BAMOCAR motor RPM indicates an effective ratio above the
    % 3.36 ratio embedded in the default EMRAX map. Use 3.5 as the current
    % working hypothesis for lap5/raw correlation.
    cfg.powertrain.finalDriveRatio = 3.36;
    cfg.powertrain.efficiency = 0.9;

    % The first pressure-brake pulse in lap5/raw under-decelerates with the
    % spec-sheet pressure calibration. Keep this in the correlation overlay
    % until the line-pressure sensor scaling / hydraulic model is verified.
    brakePressureGain = 2.9;
    cfg.brakePressure.frontForcePerBar = ...
        cfg.brakePressure.frontForcePerBar * brakePressureGain;
    cfg.brakePressure.rearForcePerBar = ...
        cfg.brakePressure.rearForcePerBar * brakePressureGain;

    % Below about 20% pedal the real car behaves more like coast than drive.
    % cfg.powertrain.throttleDeadband = 0.2;

    % BAMOCAR D3 torque mode is better represented as a shaped motor
    % current/torque request than as raw pedal * full-throttle force. This
    % first-pass correlation curve is post-deadband: it softens low/mid pedal
    % while preserving the EMRAX full-throttle envelope at 100% pedal.
    cfg.powertrain.throttleMapInput = [0.00 0.15 0.35 0.60 0.80 1.00];
    cfg.powertrain.throttleMapOutput = [0.00 0.15 0.35 0.6 0.8 1.00];
    cfg.powertrain.motoringDragTorque = 15;
    cfg.powertrain.motoringDragThrottleThreshold = 0.2;

    % Light zero-pedal regen. This is deliberately smaller than the coast drag
    % term above; it represents controller-commanded negative q-axis current
    % at true pedal release, not a broad low-throttle correction.
    cfg.powertrain.regenEnabled = true;
    cfg.powertrain.regenTorqueLimitNm = 5;
    cfg.powertrain.regenEnabledSpeedFloor = 2.0;
end
