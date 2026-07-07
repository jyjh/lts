function cfg = R25_correlation_tuning(cfg)
    % R25_CORRELATION_TUNING Overlay for lap5/raw correlation work.
    %
    % This intentionally keeps lts.vehicles.R25 as the base truth and only applies
    % correlation-specific assumptions that are still under investigation.

    if nargin < 1 || isempty(cfg)
        cfg = lts.vehicles.R25();
    end

    cfg.name = string(cfg.name) + "_corrTune";

    % The R25 spec lists a 16.0 in tire. Keep the physical 3.36 final drive
    % and correct the tire radius instead of masking the old 18 in placeholder
    % radius with an artificial effective ratio.
    cfg.tire.wheelRadius = 0.2032;
    cfg.powertrain.finalDriveRatio = 3.36;
    cfg.powertrain.efficiency = 0.78;
    % Keep the motoring correlation scalar separate from regen. Direct
    % torque-command regen is capped by logged pack power, so only final-drive
    % style losses should be reflected from the pack back to the tire patch.
    cfg.powertrain.regenEfficiency = 0.92;

    % Keep the pressure-brake calibration at the spec-sheet 1 g force table.
    % The previous 1.5x correlation gain compensated for the old planar
    % integration energy error and now over-brakes the 28-30 s stop, which
    % carries a speed deficit into the later high-lateral sector.

    % Below the first few percent of pedal the real car behaves more like
    % coast than drive; keep this below the logger's low-pedal cruise range.
    % cfg.powertrain.throttleDeadband = 0.07;

    % BAMOCAR D3 torque mode is better represented as a shaped motor
    % current/torque request than as raw pedal * full-throttle force. This
    % first-pass correlation curve is post-deadband: it softens low/mid pedal
    % while preserving the EMRAX full-throttle envelope at 100% pedal.
    cfg.powertrain.throttleMapInput = [0.00 0.20 0.35 0.60 0.80 1.00];
    cfg.powertrain.throttleMapOutput = [0.00 0.20 0.35 0.60 0.80 0.90];
    % cfg.powertrain.motoringDragTorque = 22;
    % cfg.powertrain.motoringDragThrottleThreshold = 0.2;

    % The nominal Drexler ramp-plate model over-locks in the lap5 6-7 s
    % transition: the outside rear spends too much capacity on drive, leaving
    % rear lateral acceleration ~0.2 g low on average. Keep the hardware model
    % but soften its effective ramp/preload for correlation until measured
    % diff breakaway/lock data is available.
    cfg.powertrain.differential.rampTorqueScale = 0.25;
    cfg.powertrain.differential.preloadBreakawayTorqueNm = 2;

    % Light zero-pedal regen. This is deliberately smaller than the coast drag
    % term above; it represents controller-commanded negative q-axis current
    % at true pedal release, not a broad low-throttle correction.
    cfg.powertrain.regenEnabled = true;
    cfg.powertrain.regenTorqueLimitNm = 5;
    cfg.powertrain.regenEnabledSpeedFloor = 2.0;
end
