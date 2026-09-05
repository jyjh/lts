function [segments, pooled, outputs] = run_correlation_segments(varargin)
% RUN_CORRELATION_SEGMENTS Chunked correlation replay with per-chunk reset.
%
% Whole-lap free-space replay drifts: it is open-loop, so any small model
% error compounds until the replayed controls no longer correspond to the
% simulated car's situation. This runner breaks the replay into fixed-duration
% segments instead. Every segment rebuilds the vehicle from config and
% warm-starts it from the MEASURED channels at the segment boundary (the same
% CorrelationStateInitializer used by run_correlation), so drift cannot
% accumulate across a segment boundary while the lap's full range of
% conditions still gets exercised and scored.
%
%   lts.app.run_correlation_segments( ...
%       'ReplayCsv', 'exports/correlation_lap5_raw_R25_corrTune_..._replay.csv', ...
%       'VehicleConfig', @lts.vehicles.R25, ...
%       'TuningFile', 'R25_correlation_tuning', ...
%       'PowertrainMode', 'motor_torque_command', ...
%       'SegmentLengthS', 5, 'SettleWindowS', 0.5)
%
% Scoring skips the first SettleWindowS of each segment so boundary-fit
% transients are not scored as physics error. The pooled metrics are
% sample-weighted over all scored segments. Segments run with free-space
% replay semantics (no path projection) and lean telemetry by default.
%
% Outputs:
%   segments - struct array, one per scored segment: t0/t1 (original replay
%              time), scoredSamples, speed/yaw/lateral RMSE, bias, and
%              per-segment yaw drift.
%   pooled   - sample-weighted RMSE/bias across all scored segments.
%   outputs  - configuration echo (segment parameters, tuning, modes).

repoRoot = lts.util.repoRoot(mfilename('fullpath'));

parser = inputParser;
parser.addParameter('ReplayCsv', '', @(x) ischar(x) || isstring(x));
parser.addParameter('VehicleConfig', @lts.vehicles.R25);
parser.addParameter('TuningFile', [], @(x) isempty(x) || isa(x, 'function_handle') || ischar(x) || isstring(x) || isa(x, 'lts.vehicle.VehicleConfig') || isstruct(x));
parser.addParameter('Track', '2026enduro');
parser.addParameter('Dt', 0.001, @(x) isnumeric(x) && isscalar(x) && x > 0);
parser.addParameter('TelemetryMode', 'lean', ...
    @(x) any(strcmpi(string(x), ["full", "lean"])));
parser.addParameter('PowertrainMode', 'motor_torque_command', @(x) ischar(x) || isstring(x));
parser.addParameter('BrakeMode', 'ratio', @(x) ischar(x) || isstring(x));
parser.addParameter('LimitMotorTorqueByPackPower', true, @(x) islogical(x) || isnumeric(x));
parser.addParameter('PackPowerAdvanceS', 0, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x));
parser.addParameter('MotorTorqueCommandDelayS', 0, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x));
parser.addParameter('SegmentLengthS', 5, @(x) isnumeric(x) && isscalar(x) && x > 0);
parser.addParameter('SegmentStrideS', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x) && x > 0));
parser.addParameter('SettleWindowS', 0.5, @(x) isnumeric(x) && isscalar(x) && x >= 0);
parser.addParameter('SegmentRange', [], @(x) isempty(x) || isnumeric(x) && isvector(x));
parser.addParameter('UseLoggedPosition', true, @(x) islogical(x) || isnumeric(x));
parser.addParameter('UseLoggedYawRate', [], @(x) isempty(x) || islogical(x) || isnumeric(x));
parser.addParameter('UseLoggedWheelSpeeds', [], @(x) isempty(x) || islogical(x) || isnumeric(x));
parser.addParameter('UseLoggedDrivenWheelCarrierSpeed', [], ...
    @(x) isempty(x) || islogical(x) || isnumeric(x));
parser.addParameter('UseLoggedTransientState', [], @(x) isempty(x) || islogical(x) || isnumeric(x));
parser.addParameter('InitialTransientWindowS', [], ...
    @(x) isempty(x) || (isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0));
parser.addParameter('SteeringCenterGain', [], ...
    @(x) isempty(x) || (isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0 && x <= 2));
parser.addParameter('SteeringCenterOffsetRad', [], ...
    @(x) isempty(x) || (isnumeric(x) && isscalar(x) && isfinite(x)));
parser.addParameter('SteeringCalibrationEndAngleRad', [], ...
    @(x) isempty(x) || (isnumeric(x) && isscalar(x) && isfinite(x) && x > 0));
parser.addParameter('SteeringDelayS', [], ...
    @(x) isempty(x) || (isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0));
parser.addParameter('ShowProgress', true, @(x) islogical(x) || isnumeric(x));
parser.parse(varargin{:});
opts = parser.Results;

if isempty(opts.ReplayCsv)
    error('run_correlation_segments:MissingInput', ...
        'Provide ReplayCsv (extract it first with run_correlation or extract_motec_lap.py).');
end

opts.PowertrainMode = lts.correlation.CorrelationAppSupport.validatePowertrainMode(opts.PowertrainMode);
track = lts.correlation.CorrelationAppSupport.loadTrack(opts.Track, repoRoot);
config = lts.correlation.CorrelationAppSupport.loadVehicleConfig(opts.VehicleConfig);
config = lts.correlation.CorrelationAppSupport.applyVehicleTuning( ...
    config, [], opts.TuningFile);
correlationOverrides = ...
    lts.correlation.CorrelationAppSupport.vehicleCorrelationStruct(config);
if ~isempty(correlationOverrides) && isfield(correlationOverrides, 'brakeMode')
    opts.BrakeMode = lts.correlation.CorrelationAppSupport.validateBrakeMode( ...
        correlationOverrides.brakeMode);
end

useLoggedYawRate = lts.correlation.CorrelationAppSupport.vehicleCorrelationFlag( ...
    config, 'useLoggedYawRate', true);
if ~isempty(opts.UseLoggedYawRate)
    useLoggedYawRate = logical(opts.UseLoggedYawRate);
end
useLoggedTransientState = lts.correlation.CorrelationAppSupport.vehicleCorrelationFlag( ...
    config, 'useLoggedTransientState', true);
if ~isempty(opts.UseLoggedTransientState)
    useLoggedTransientState = logical(opts.UseLoggedTransientState);
end
useLoggedWheelSpeeds = lts.correlation.CorrelationAppSupport.vehicleCorrelationFlag( ...
    config, 'useLoggedWheelSpeeds', true);
if ~isempty(opts.UseLoggedWheelSpeeds)
    useLoggedWheelSpeeds = logical(opts.UseLoggedWheelSpeeds);
end
useLoggedDrivenWheelCarrierSpeed = ...
    lts.correlation.CorrelationAppSupport.vehicleCorrelationFlag( ...
        config, 'useLoggedDrivenWheelCarrierSpeed', false);
if ~isempty(opts.UseLoggedDrivenWheelCarrierSpeed)
    useLoggedDrivenWheelCarrierSpeed = logical(opts.UseLoggedDrivenWheelCarrierSpeed);
end
initialTransientWindowS = lts.correlation.CorrelationAppSupport.vehicleCorrelationScalar( ...
    config, 'initialTransientWindowS', 0);
if ~isempty(opts.InitialTransientWindowS)
    initialTransientWindowS = double(opts.InitialTransientWindowS);
end

steeringCenterGain = lts.correlation.CorrelationAppSupport.vehicleCorrelationScalar( ...
    config, 'steeringCenterGain', 1);
steeringCenterOffsetRad = lts.correlation.CorrelationAppSupport.vehicleCorrelationScalar( ...
    config, 'steeringCenterOffsetRad', 0);
steeringCalibrationEndAngleRad = lts.correlation.CorrelationAppSupport.vehicleCorrelationScalar( ...
    config, 'steeringCalibrationEndAngleRad', deg2rad(22));
steeringDelayS = lts.correlation.CorrelationAppSupport.vehicleCorrelationScalar( ...
    config, 'steeringDelayS', 0);
if ~isempty(opts.SteeringCenterGain)
    steeringCenterGain = double(opts.SteeringCenterGain);
end
if ~isempty(opts.SteeringCenterOffsetRad)
    steeringCenterOffsetRad = double(opts.SteeringCenterOffsetRad);
end
if ~isempty(opts.SteeringCalibrationEndAngleRad)
    steeringCalibrationEndAngleRad = double(opts.SteeringCalibrationEndAngleRad);
end
if ~isempty(opts.SteeringDelayS)
    steeringDelayS = double(opts.SteeringDelayS);
end

profile = lts.correlation.CorrelationReplayProfile.fromCsv(opts.ReplayCsv);
if opts.PowertrainMode == "motor_torque_delivered" && ...
        lts.correlation.CorrelationAppSupport.vehicleCorrelationFlag( ...
            config, 'repairInvalidDeliveredRegen', false)
    profile = profile.withPowerConservingRegenRepair( ...
        'PackPowerAdvanceS', ...
        lts.correlation.CorrelationAppSupport.vehicleCorrelationScalar( ...
            config, 'regenRepairPackPowerAdvanceS', -0.015), ...
        'MinimumChargingPowerW', ...
        lts.correlation.CorrelationAppSupport.vehicleCorrelationScalar( ...
            config, 'regenRepairMinimumChargingPowerW', 1000), ...
        'MinimumMotorSpeedRpm', ...
        lts.correlation.CorrelationAppSupport.vehicleCorrelationScalar( ...
            config, 'regenRepairMinimumMotorSpeedRpm', 300), ...
        'RegenRequestThresholdNm', ...
        lts.correlation.CorrelationAppSupport.vehicleCorrelationScalar( ...
            config, 'regenRepairRequestThresholdNm', 5), ...
        'PowerToleranceW', ...
        lts.correlation.CorrelationAppSupport.vehicleCorrelationScalar( ...
            config, 'regenRepairPowerToleranceW', 500), ...
        'MaximumRegenEfficiency', ...
        lts.correlation.CorrelationAppSupport.vehicleCorrelationScalar( ...
            config, 'regenRepairMaximumEfficiency', 1), ...
        'MaximumReconstructedTorqueNm', ...
        lts.correlation.CorrelationAppSupport.vehicleCorrelationScalar( ...
            config, 'regenRepairMaximumTorqueNm', 170));
end
profile = profile.withSteeringCalibration( ...
    steeringCenterGain, steeringCalibrationEndAngleRad, steeringDelayS, ...
    steeringCenterOffsetRad);
profile = profile.withPackPowerAdvance(opts.PackPowerAdvanceS);
profile = profile.withMotorTorqueCommandDelay(opts.MotorTorqueCommandDelayS);

% One whole-profile preflight for mode validation and data-quality warnings.
preflightVehicle = lts.vehicle.VehicleManager.fromConfig(config, track, opts.Dt);
lts.correlation.CorrelationAppSupport.preflight( ...
    profile, track, preflightVehicle, 1.0, '', ...
    opts.BrakeMode, opts.PowertrainMode, ...
    opts.LimitMotorTorqueByPackPower, ...
    opts.PackPowerAdvanceS, opts.MotorTorqueCommandDelayS);

% Segment grid over the (rebased) profile time axis.
tStart = double(profile.time(1));
tEnd = double(profile.time(end));
stride = opts.SegmentLengthS;
if ~isempty(opts.SegmentStrideS)
    stride = double(opts.SegmentStrideS);
end
if stride > opts.SegmentLengthS
    error('run_correlation_segments:InvalidStride', ...
        'SegmentStrideS (%.6g) cannot exceed SegmentLengthS (%.6g).', ...
        stride, opts.SegmentLengthS);
end
segmentCount = floor((tEnd - tStart - opts.SegmentLengthS) / stride + 32 * eps) + 1;
segmentCount = max(segmentCount, 0);
if segmentCount < 1
    error('run_correlation_segments:ProfileTooShort', ...
        'Replay range [%.6g, %.6g] s is shorter than one %.6g s segment.', ...
        tStart, tEnd, opts.SegmentLengthS);
end
segmentIndices = 1:segmentCount;
if ~isempty(opts.SegmentRange)
    segmentIndices = double(opts.SegmentRange(:)).';
    if any(segmentIndices < 1 | segmentIndices > segmentCount | ...
            segmentIndices ~= round(segmentIndices))
        error('run_correlation_segments:InvalidSegmentRange', ...
            'SegmentRange must contain 1-based indices between 1 and %d.', ...
            segmentCount);
    end
end

outputs = struct();
outputs.replayCsv = char(opts.ReplayCsv);
outputs.vehicleConfig = char(config.name);
outputs.track = char(string(opts.Track));
outputs.dt = opts.Dt;
outputs.telemetryMode = char(string(opts.TelemetryMode));
outputs.powertrainMode = char(opts.PowertrainMode);
outputs.brakeMode = char(opts.BrakeMode);
outputs.segmentLengthS = opts.SegmentLengthS;
outputs.segmentStrideS = stride;
outputs.settleWindowS = opts.SettleWindowS;
outputs.segmentCount = segmentCount;
outputs.steeringCenterGain = steeringCenterGain;
outputs.initialTransientWindowS = initialTransientWindowS;
outputs.useLoggedYawRate = useLoggedYawRate;
outputs.useLoggedTransientState = useLoggedTransientState;

fieldNames = {'t0', 't1', 'scoredSamples', 'status', 'speed_rmse_mps', ...
    'speed_bias_mps', 'yaw_rmse_radps', 'yaw_bias_radps', ...
    'yaw_drift_deg', 'ay_rmse_g'};
for i = 1:numel(segmentIndices)
    idx = segmentIndices(i);
    t0 = tStart + (idx - 1) * stride;
    row = struct();
    for f = fieldNames
        row.(f{1}) = NaN;
    end
    row.t0 = t0;
    row.t1 = t0 + opts.SegmentLengthS;
    row.status = 'not_run';
    scored = false;

    try
        sub = profile.window(t0 - tStart, opts.SegmentLengthS);
        vehicle = lts.vehicle.VehicleManager.fromConfig(config, track, opts.Dt);
        initialState = lts.correlation.CorrelationStateInitializer.fromReplayProfile( ...
            sub, [], vehicle, ...
            'UseLoggedPosition', opts.UseLoggedPosition, ...
            'UseLoggedYawRate', useLoggedYawRate, ...
            'UseLoggedWheelSpeeds', useLoggedWheelSpeeds, ...
            'UseLoggedDrivenWheelCarrierSpeed', useLoggedDrivenWheelCarrierSpeed, ...
            'UseLoggedTransientState', useLoggedTransientState, ...
            'InitialTransientWindowS', initialTransientWindowS);
        simulator = lts.simulation.Simulator(vehicle, [], opts.Dt);
        simulator.telemetryMode = lower(string(opts.TelemetryMode));
        stateLog = simulator.simulateReplay( ...
            initialState, track, sub, ...
            'ReplayDomain', 'time', ...
            'BrakeMode', opts.BrakeMode, ...
            'PowertrainMode', opts.PowertrainMode, ...
            'LimitMotorTorqueByPackPower', opts.LimitMotorTorqueByPackPower, ...
            'AllowPedalOverlap', true, ...
            'ApplySteeringSlew', false, ...
            'StopOnOffTrack', false, ...
            'StopAtTrackEnd', false, ...
            'StopAtReplayEnd', true, ...
            'ReferenceMode', 'free', ...
            'SurfaceMu', 1.0);

        metrics = localScoreSegment(stateLog, sub, opts.SettleWindowS);
        metricFields = {'scoredSamples', 'speed_rmse_mps', 'speed_bias_mps', ...
            'yaw_rmse_radps', 'yaw_bias_radps', 'yaw_drift_deg', 'ay_rmse_g'};
        for f = metricFields
            row.(f{1}) = metrics.(f{1});
        end
        if row.scoredSamples >= 2
            row.status = 'scored';
        else
            row.status = 'too_short_to_score';
        end
        expectedEnd = sub.time(end);
        if stateLog.time(end) < expectedEnd - 2 * opts.Dt
            row.status = 'early_stop';
        end
    catch err
        row.status = ['error: ' err.message];
    end

    for f = fieldNames
        segments(i).(f{1}) = row.(f{1});
    end
    if opts.ShowProgress
        fprintf('SEGMENT %2d/%2d [%6.2f-%6.2f s] %s  speedRMSE=%6.3f  yawRMSE=%6.3f  yawDrift=%+7.2f deg\n', ...
            i, numel(segmentIndices), row.t0, row.t1, row.status, ...
            row.speed_rmse_mps, row.yaw_rmse_radps, row.yaw_drift_deg);
    end
end

pooled = localPoolSegments(segments);
if opts.ShowProgress
    fprintf('POOLED over %d scored samples: speedRMSE=%.3f m/s (bias %+.3f)  yawRMSE=%.3f rad/s (bias %+.3f)  ayRMSE=%.3f g\n', ...
        pooled.scoredSamples, pooled.speed_rmse_mps, pooled.speed_bias_mps, ...
        pooled.yaw_rmse_radps, pooled.yaw_bias_radps, pooled.ay_rmse_g);
end
end

function [row, scored] = localScoreSegment(stateLog, sub, settleWindowS)
% Score one segment: interpolate measured channels onto the stateLog time
% axis and accumulate error statistics over the post-settle window.
    row = struct();
    fields = {'scoredSamples', 'speed_rmse_mps', 'speed_bias_mps', ...
        'yaw_rmse_radps', 'yaw_bias_radps', 'yaw_drift_deg', 'ay_rmse_g'};
    for f = fields
        row.(f{1}) = NaN;
    end
    row.scoredSamples = 0;
    scored = false;

    t = stateLog.time(:);
    mask = t >= settleWindowS & t <= sub.time(end);
    if nnz(mask) < 2
        return;
    end

    speedErr = stateLog.speed(:) - interp1(sub.time, sub.speed, t, 'linear', NaN);
    speedErr = speedErr(mask);
    valid = ~isnan(speedErr);
    row.scoredSamples = nnz(valid);
    if ~any(valid)
        return;
    end
    row.speed_rmse_mps = sqrt(mean(speedErr(valid).^2));
    row.speed_bias_mps = mean(speedErr(valid));

    try
        yawMeas = interp1(sub.time, sub.yawRate, t, 'linear', NaN);
        yawErr = stateLog.yawRate(:) - yawMeas;
        yawMask = mask & ~isnan(yawErr);
        if any(yawMask)
            row.yaw_rmse_radps = sqrt(mean(yawErr(yawMask).^2));
            row.yaw_bias_radps = mean(yawErr(yawMask));
            row.yaw_drift_deg = trapz(t(yawMask), yawErr(yawMask)) * 180 / pi;
        end
    catch
        % Logged yaw-rate channel absent; leave NaN.
    end

    try
        ayMeas = interp1(sub.time, sub.latAccelG, t, 'linear', NaN) * 9.80665;
        ayErr = stateLog.ay(:) - ayMeas;
        ayErr = ayErr(mask);
        valid = ~isnan(ayErr);
        if any(valid)
            row.ay_rmse_g = sqrt(mean(ayErr(valid).^2)) / 9.80665;
        end
    catch
        % Logged lateral-accel channel absent; leave NaN.
    end
    scored = true;
end

function pooled = localPoolSegments(segments)
% Sample-weighted pooled statistics across scored segments.
    pooled = struct('scoredSamples', 0, 'speed_rmse_mps', NaN, ...
        'speed_bias_mps', NaN, 'yaw_rmse_radps', NaN, 'yaw_bias_radps', NaN, ...
        'ay_rmse_g', NaN);
    isScored = strcmp({segments.status}, 'scored');
    if ~any(isScored)
        return;
    end
    scored = segments(isScored);
    total = sum([scored.scoredSamples]);
    sse = 0;
    speedBiasNum = 0;
    for i = 1:numel(scored)
        n = scored(i).scoredSamples;
        sse = sse + n * scored(i).speed_rmse_mps^2;
        speedBiasNum = speedBiasNum + scored(i).speed_bias_mps * n;
    end
    pooled.scoredSamples = total;
    pooled.speed_rmse_mps = sqrt(sse / total);
    pooled.speed_bias_mps = speedBiasNum / total;

    yawSse = 0; yawAbs = 0; yawBiasNum = 0; yawCount = 0;
    for i = 1:numel(scored)
        if isfinite(scored(i).yaw_rmse_radps)
            yawSse = yawSse + scored(i).yaw_rmse_radps^2 * scored(i).scoredSamples;
            yawBiasNum = yawBiasNum + scored(i).yaw_bias_radps * scored(i).scoredSamples;
            yawCount = yawCount + scored(i).scoredSamples;
        end
    end
    if yawCount > 0
        pooled.yaw_rmse_radps = sqrt(yawSse / yawCount);
        pooled.yaw_bias_radps = yawBiasNum / yawCount;
    end

    aySse = 0; ayCount = 0;
    for i = 1:numel(scored)
        if isfinite(scored(i).ay_rmse_g)
            aySse = aySse + scored(i).ay_rmse_g^2 * scored(i).scoredSamples;
            ayCount = ayCount + scored(i).scoredSamples;
        end
    end
    if ayCount > 0
        pooled.ay_rmse_g = sqrt(aySse / ayCount);
    end
end
