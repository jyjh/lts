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
    cfg.powertrain.efficiency = 0.85;
    % Keep the motoring correlation scalar separate from regen. Direct
    % torque-command regen can be capped by logged pack power when
    % LimitMotorTorqueByPackPower is enabled, so only final-drive style losses
    % should be reflected from the pack back to the tire patch.
    cfg.powertrain.regenEfficiency = 0.92;

    % The nominal Drexler ramp-plate model over-locks in the lap5 6-7 s
    % transition: the outside rear spends too much capacity on drive, leaving
    % rear lateral acceleration ~0.2 g low on average. Keep the hardware model
    % but soften its effective ramp/preload for correlation until measured
    % diff breakaway/lock data is available.
    cfg.powertrain.differential.rampTorqueScale = 0.25;
    cfg.powertrain.differential.preloadBreakawayTorqueNm = 2;

    % The first 5 s starts during a logged transient. Median-filter the
    % startup yaw/lateral channels over a short window instead of trusting
    % a single noisy initial sample from the accelerometers.
    cfg.correlation.useLoggedYawRate = true;
    cfg.correlation.useLoggedTransientState = true;
    cfg.correlation.initialTransientWindowS = 0.3;
end
