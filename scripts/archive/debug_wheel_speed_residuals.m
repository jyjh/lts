function debug_wheel_speed_residuals()
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(repoRoot, 'src'));

sourceReplay = fullfile(repoRoot, 'exports', ...
    'correlation_lap5_raw_R25_corrTune_20260707_144900_replay.csv');
shortReplay = fullfile(repoRoot, 'exports', '_tmp_wheel_replay_0_10p5.csv');
outputBase = fullfile(repoRoot, 'exports', '_tmp_wheel_lkx04');

R = readtable(sourceReplay);
R = R(R.time_s <= 10.5, :);
R.distance_m = R.distance_m - R.distance_m(1);
writetable(R, shortReplay);

[~, ~, outputs] = lts.app.run_correlation( ...
    'ReplayCsv', shortReplay, ...
    'VehicleConfig', @lts.vehicles.R25, ...
    'TuningFile', 'R25_correlation_tuning', ...
    'BrakeMode', 'pressure', ...
    'PowertrainMode', 'motor_torque_command', ...
    'OutputBase', outputBase, ...
    'ExportMoTeC', false, ...
    'ShowPlots', false);

S = readtable(outputs.csvFile, 'VariableNamingRule', 'preserve');
summarizeWheelResiduals(S, R);
end

function summarizeWheelResiduals(S, R)
t = column(S, {'Control Time (s)', 'Time', 'time_s'});
speed = column(S, {'Ground Speed (m/s)', 'Speed (m/s)', 'speed_mps'});
Rwheel = 0.2032;

sim.FL = column(S, {'Wheel Speed Front Left Sensor Linear (m/s)'});
sim.FR = column(S, {'Wheel Speed Front Right Sensor Linear (m/s)'});
sim.RL = column(S, {'Wheel Speed Rear Left Sensor Linear (m/s)'});
sim.RR = column(S, {'Wheel Speed Rear Right Sensor Linear (m/s)'});
slip.RL = column(S, {'Slip Ratio RL'});
slip.RR = column(S, {'Slip Ratio RR'});
drive.RL = column(S, {'Drive Torque RL (Nm)'});
drive.RR = column(S, {'Drive Torque RR (Nm)'});
fx.RR = column(S, {'Tire Fx RR (N)'});
motorTorque = column(S, {'Motor Torque (Nm)'});
motorRpm = column(S, {'Motor RPM (rpm)'});

raw.FL = interp1(R.time_s, R.wheel_speed_fl_mps, t, 'linear', 'extrap');
raw.FR = interp1(R.time_s, R.wheel_speed_fr_mps, t, 'linear', 'extrap');
raw.RR = interp1(R.time_s, R.wheel_speed_rr_mps, t, 'linear', 'extrap');
raw.speed = interp1(R.time_s, R.speed_mps, t, 'linear', 'extrap');

fprintf('\n=== Current-code wheel residuals (sim - raw, m/s) ===\n');
for name = ["FL", "FR", "RR"]
    err = sim.(name) - raw.(name);
    mask = isfinite(err) & t <= 10.5;
    [maxAbsErr, maxIdxLocal] = max(abs(err(mask)));
    idxs = find(mask);
    idx = idxs(maxIdxLocal);
    fprintf('%s 0-2s: mean %+7.3f rmse %7.3f max %+7.3f at %.3fs | 0-10.5s: mean %+7.3f rmse %7.3f max %+7.3f at %.3fs\n', ...
        name, ...
        mean(err(mask & t <= 2)), sqrt(mean(err(mask & t <= 2).^2)), ...
        signedMax(err(mask & t <= 2)), t(firstSignedMaxIndex(err, mask & t <= 2)), ...
        mean(err(mask)), sqrt(mean(err(mask).^2)), ...
        err(idx), t(idx));
end

queryTimes = [0 0.5 1 2 3 4 5 6.52 10];
fprintf('\n=== Point samples ===\n');
for q = queryTimes
    [~, idx] = min(abs(t - q));
    fprintf(['t %.3f speed %.3f rawSpeed %.3f FL %.3f rawFL %.3f ' ...
        'RR %.3f rawRR %.3f slipRL %.3f%% slipRR %.3f%% ' ...
        'driveRL %.1f driveRR %.1f fxRR %.1f motor %.1f rpm %.0f torqueBalanceRR %.3f\n'], ...
        t(idx), speed(idx), raw.speed(idx), ...
        sim.FL(idx), raw.FL(idx), sim.RR(idx), raw.RR(idx), ...
        100 * slip.RL(idx), 100 * slip.RR(idx), ...
        drive.RL(idx), drive.RR(idx), fx.RR(idx), ...
        motorTorque(idx), motorRpm(idx), ...
        drive.RR(idx) / Rwheel - fx.RR(idx));
end
end

function values = column(T, names)
values = NaN(height(T), 1);
vars = string(T.Properties.VariableNames);
for i = 1:numel(names)
    idx = find(vars == string(names{i}), 1);
    if ~isempty(idx)
        values = T.(vars(idx));
        values = values(:);
        return;
    end
end
end

function value = signedMax(x)
[~, idx] = max(abs(x));
value = x(idx);
end

function idx = firstSignedMaxIndex(err, mask)
idxs = find(mask);
[~, local] = max(abs(err(mask)));
idx = idxs(local);
end
