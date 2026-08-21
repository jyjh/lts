function cfg = R25_correlation_tuning(cfg)
    % R25_CORRELATION_TUNING Overlay for lap5/raw correlation work.
    %
    % This intentionally keeps lts.vehicles.R25 as the base truth and only applies
    % correlation-specific assumptions that are still under investigation.

    if nargin < 1 || isempty(cfg)
        cfg = lts.vehicles.R25();
    end

    cfg.name = string(cfg.name) + "_corrTune";

    % Restore the historical lap-5 effective tire artifact for legacy replay
    % diagnostics. Production design studies reject this overlay.
    cfg.tire.tirFile = 'Hoosier 43100 18.0x6.0-10 R20_7 - Scaled.tir';

    % Effective correlation drag includes the measured whole-car aero plus
    % speed-dependent rotating/drivetrain losses not represented separately.
    % The physical R25 aero definition remains unchanged in lts.vehicles.R25.
    % With the unitless BAMOCAR Iq signal converted using 0.5 Arms/count and
    % the EMRAX MV torque constant, the earlier whole-lap Iq fit is valid:
    % straight, low-lateral-acceleration drive samples identify a 0.18 m^2
    % reduction and whole-lap interpolation places the speed-error minimum at
    % 2.65 m^2.
    cfg.aero.CdA = 2.65;
    % Do not apply an additional fitted torque-proportional loss after the
    % measured Iq-derived shaft torque. The previous 0.97 scalar reduced every
    % motoring event and amplified every regen event (the loss direction is
    % reversed while back-driving), which produces the persistent low-speed
    % offset after 45 s. Keep this path power-neutral until measured
    % shaft-to-axle loss data is available; wheel/rotor inertia, rolling
    % resistance, and aero drag remain modeled independently.
    cfg.powertrain.deliveredTorqueDrivetrainEfficiency = 1.0;
    % Apply the extractor's idempotent power-conservation repair inside the
    % MATLAB replay path as well. This is required when run_correlation is
    % given an older ReplayCsv, because that path intentionally skips MoTeC
    % extraction. Align the pack channel 15 ms later than motor torque. This
    % is the lap5/raw alignment that minimizes the physically reconstructed
    % braking impulse, rather than the old +85 ms setting that minimized only
    % the count of violating samples while fabricating much more regen torque.
    % Unity is the maximum possible regen conversion efficiency and therefore
    % yields the minimum physically admissible shaft-braking torque.
    % Repair activation follows the signed motor command; the always-negative
    % regen capability channel is only a fallback when that command is absent.
    cfg.correlation.repairInvalidDeliveredRegen = true;
    cfg.correlation.regenRepairPackPowerAdvanceS = -0.015;
    cfg.correlation.regenRepairMinimumChargingPowerW = 1000;
    cfg.correlation.regenRepairMinimumMotorSpeedRpm = 300;
    cfg.correlation.regenRepairRequestThresholdNm = 5;
    cfg.correlation.regenRepairPowerToleranceW = 500;
    cfg.correlation.regenRepairMaximumEfficiency = 1.0;
    cfg.correlation.regenRepairMaximumTorqueNm = 170;

    % The first 5 s starts during a logged transient. Fit a short local
    % boundary trend to the startup yaw/lateral channels instead of leaking
    % later transient evolution into the t=0 state.
    cfg.correlation.useLoggedYawRate = true;
    % The lap5/raw rear wheel channels are not a dynamically consistent
    % startup pair: RL is failed at zero and RR has a persistent linear-speed
    % calibration offset. Do not seed the four wheels from those channels.
    cfg.correlation.useLoggedWheelSpeeds = false;
    % The individual rear-wheel channels are unsuitable for initialization
    % (RL is failed and RR has a linear-speed scale offset), but motor RPM
    % provides the measured differential-carrier speed. Seed the rear-pair
    % mean from RPM/FDR while retaining the yaw-induced left/right kinematic
    % difference. Front wheels remain initialized from contact kinematics.
    cfg.correlation.useLoggedDrivenWheelCarrierSpeed = true;
    cfg.correlation.useLoggedTransientState = true;
    cfg.correlation.initialTransientWindowS = 0.3;

    % The updated 43075 scaling exposes an axle-balance error rather than a
    % whole-tire grip error. With equal front/rear effective stiffness, front
    % lateral force gives the model too much yaw authority: -1.98 g at 16.7 s
    % and -2.20 g at 74.2 s. Reducing global LMUY or LKY also weakens rear
    % stability and prevents lateral force from returning cleanly to zero.
    % Retain rear stiffness and peak grip, and apply the measured front-axle
    % cornering-stiffness correction only in correlation replay.
    cfg.tire.lateralStiffnessScaleByCorner = [0.65 0.65 1 1];
    % Use the logged hydraulic pressures with the R25 force-per-bar
    % calibration. This avoids the lap-normalized brake-ratio path, which
    % over-decelerates the later stops and leaves a persistent speed offset.
    cfg.correlation.brakeMode = "pressure";

    % The R25 steering sensor is known to be accurate near its measured
    % endpoints but nonlinear and slightly mis-zeroed around center. Preserve
    % +/-22 deg exactly, use a quadratic calibration with a reduced center
    % slope, and fade the zero correction out at the endpoints. The delay
    % aligns the corrected command with the measured yaw response.
    % Recalibrated after applying the 43075 tire stiffness: the wider,
    % shorter-sidewall tire changes yaw gain, so retaining the calibration
    % derived with the unscaled 43100 tire over-predicts accumulated heading.
    % These values are the best directly validated whole-lap bracket: an
    % interpolation between brackets was rejected because the coupled vehicle
    % response is nonlinear. The measured endpoint mapping is retained.
    cfg.correlation.steeringCenterGain = 0.550;
    % Do not inject a steering command at the raw sensor center. The former
    % -0.585 deg offset produced a persistent roughly -0.13 g lateral bias
    % during otherwise straight periods.
    cfg.correlation.steeringCenterOffsetRad = 0;
    cfg.correlation.steeringCalibrationEndAngleRad = deg2rad(22);
    cfg.correlation.steeringDelayS = 0.0;
end
