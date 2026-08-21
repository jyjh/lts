function tests = CorrelationTuningTest
tests = functiontests(localfunctions);
end

function testReplayWindowInterpolatesAndRebasesEveryChannel(testCase)
profile = lts.correlation.CorrelationReplayProfile( ...
    'SourceFile', 'synthetic.csv', ...
    'Time', [10; 11; 12; 13], ...
    'Distance', [100; 110; 121; 133], ...
    'Throttle', [0; 0.2; 0.4; 0.6], ...
    'Brake', [0; 0; 0.1; 0.2], ...
    'Steer', [0; 0.1; 0.2; 0.3], ...
    'Speed', [10; 11; 12; 13], ...
    'YawRate', [0; 1; 2; 3], ...
    'LatAccelG', [0; 0.1; 0.2; 0.3]);

segment = profile.window(0.5, 2);

verifyEqual(testCase, segment.sourceFile, 'synthetic.csv');
verifyEqual(testCase, segment.time, [0; 0.5; 1.5; 2], 'AbsTol', 1e-12);
verifyEqual(testCase, segment.distance, [0; 5; 16; 22], 'AbsTol', 1e-12);
verifyEqual(testCase, segment.speed, [10.5; 11; 12; 12.5], 'AbsTol', 1e-12);
verifyEqual(testCase, segment.yawRate, [0.5; 1; 2; 2.5], 'AbsTol', 1e-12);
verifyEqual(testCase, segment.duration(), 2, 'AbsTol', 1e-12);
end

function testReplayWindowRejectsOutsideRange(testCase)
profile = localProfile((0:0.1:1).');
verifyError(testCase, @() profile.window(0.5, 0.6), ...
    'lts_correlation_CorrelationReplayProfile:WindowOutsideProfile');
end

function testParameterRegistryRoundTripAndApply(testCase)
root = lts.util.repoRoot(mfilename('fullpath'));
registry = lts.correlation.CorrelationParameterRegistry.load(fullfile( ...
    root, 'config', 'correlation', 'lap5_ml_parameter_space.json'));
baseline = lts.correlation.CorrelationParameterRegistry.baseline(registry);
normalized = lts.correlation.CorrelationParameterRegistry.encode(registry, baseline);
verifyEqual(testCase, ...
    lts.correlation.CorrelationParameterRegistry.decode(registry, normalized), ...
    baseline, 'RelTol', 1e-12, 'AbsTol', 1e-12);

cfg = lts.vehicles.R25_correlation_tuning(lts.vehicles.R25());
cfg = lts.correlation.CorrelationParameterRegistry.apply(registry, cfg, baseline);
verifyEqual(testCase, cfg.yawInertia, 130, 'AbsTol', 1e-12);
verifyEqual(testCase, cfg.tire.lateralStiffnessScaleByCorner, ...
    [0.65 0.65 1 1], 'AbsTol', 1e-12);
verifyEqual(testCase, cfg.suspension.front.springRate, 52537.9, 'AbsTol', 1e-12);
end

function testExactPredictionScoresZeroAndOffsetIsPenalized(testCase)
time = (0:0.1:1).';
speed = 10 + time;
yawRate = 0.2 * time;
latG = 0.5 * time;
longG = -0.1 * time;
frontWheel = speed + 0.1;
rearCarrier = speed + 0.2;
fdr = 3.36;
radius = 0.2032;
motorRpm = rearCarrier / radius * fdr * 60 / (2*pi);
profile = lts.correlation.CorrelationReplayProfile( ...
    'Time', time, 'Distance', cumtrapz(time, speed), ...
    'Throttle', zeros(size(time)), 'Brake', zeros(size(time)), ...
    'Steer', zeros(size(time)), 'Speed', speed, ...
    'YawRate', yawRate, 'LatAccelG', latG, ...
    'FrontLatAccelG', latG, 'RearLatAccelG', latG, ...
    'LongAccelG', longG, ...
    'WheelSpeedFL', frontWheel, 'WheelSpeedFR', frontWheel, ...
    'MotorRpm', motorRpm);
stateLog = struct( ...
    'controlTime', time, 'speed', speed, 'yawRate', yawRate, ...
    'ay', latG * 9.80665, 'frontAxleAy', latG * 9.80665, ...
    'rearAxleAy', latG * 9.80665, 'ax', longG * 9.80665, ...
    'tireSpeed_FL', frontWheel, 'tireSpeed_FR', frontWheel, ...
    'tireSpeed_RL', rearCarrier, 'tireSpeed_RR', rearCarrier);

exact = lts.correlation.CorrelationScore.evaluate(stateLog, profile);
verifyEqual(testCase, exact.score, 0, 'AbsTol', 1e-14);
stateLog.speed = stateLog.speed + 1;
offset = lts.correlation.CorrelationScore.evaluate(stateLog, profile);
verifyGreaterThan(testCase, offset.score, 0);
verifyGreaterThan(testCase, offset.speed, 0);
verifyEqual(testCase, offset.yawRate, 0, 'AbsTol', 1e-14);
end

function testRunawayScalarChannelLossIsBounded(testCase)
time = (0:0.1:1).';
profile = localProfile(time);
stateLog = struct('controlTime', time, ...
    'speed', profile.speed + 1e9);
result = lts.correlation.CorrelationScore.evaluate(stateLog, profile);
verifyEqual(testCase, result.speed, 50, 'AbsTol', 1e-12);
verifyEqual(testCase, result.score, 50, 'AbsTol', 1e-12);
end

function testScoreRenormalizesWhenChannelsAreMissing(testCase)
time = (0:0.1:1).';
profile = localProfile(time);
stateLog = struct('controlTime', time, 'speed', profile.speed);
result = lts.correlation.CorrelationScore.evaluate(stateLog, profile);
verifyEqual(testCase, result.score, 0, 'AbsTol', 1e-14);
verifyTrue(testCase, isnan(result.yawRate));
end

function testWindowsAlternateTrainAndValidation(testCase)
profile = localProfile((0:0.1:12).');
windows = lts.correlation.CorrelationTuningEvaluator.makeWindows(profile, 3);
verifyEqual(testCase, windows.start_s, [0; 3; 6; 9], 'AbsTol', 1e-12);
verifyEqual(testCase, windows.split, ["train"; "validation"; "train"; "validation"]);
end

function testMixedHorizonsShareLeakFreeAnchorSplit(testCase)
profile = localProfile((0:0.1:24).');
windows = lts.correlation.CorrelationTuningEvaluator.makeWindows(profile, [12 3 6]);
verifyEqual(testCase, windows.anchor_id, [1; 1; 1; 2; 2; 2]);
verifyEqual(testCase, windows.start_s, [0; 0; 0; 12; 12; 12], 'AbsTol', 1e-12);
verifyEqual(testCase, windows.horizon_s, [3; 6; 12; 3; 6; 12], 'AbsTol', 1e-12);
verifyEqual(testCase, windows.split, ...
    ["train"; "train"; "train"; "validation"; "validation"; "validation"]);
verifyLessThanOrEqual(testCase, ...
    max(windows.start_s(windows.split == "train") + ...
    windows.horizon_s(windows.split == "train")), ...
    min(windows.start_s(windows.split == "validation")));
end

function testGpsKinematicsDerivesSpeedAndBodyAcceleration(testCase)
time = (0:0.05:10).';
earthRadius = 6371008.8;
east = 8 .* time + 0.5 .* 1.5 .* time.^2;
lat = ones(size(time));
lon = 103 + rad2deg(east ./ (earthRadius * cosd(lat(1))));
profile = lts.correlation.CorrelationReplayProfile( ...
    'Time', time, 'Distance', zeros(size(time)), ...
    'Throttle', zeros(size(time)), 'Brake', zeros(size(time)), ...
    'Steer', zeros(size(time)), 'Speed', ones(size(time)), ...
    'GpsLat', lat, 'GpsLon', lon);
[profile, report] = profile.withGpsKinematics('SmoothingWindowS', 0.15);
interior = time >= 1 & time <= 9;
verifyEqual(testCase, report.status, "applied");
verifyEqual(testCase, median(profile.speed(interior)), 15.5, 'AbsTol', 0.1);
verifyEqual(testCase, median(profile.longAccelG(interior)), ...
    1.5 / 9.80665, 'AbsTol', 0.01);
verifyEqual(testCase, median(profile.latAccelG(interior)), 0, 'AbsTol', 1e-5);
mid = find(time == 5, 1);
verifyEqual(testCase, profile.x(mid), east(mid), 'AbsTol', 0.1);
end

function testGpsTraceIsScoredAfterAnchorAndHeadingAlignment(testCase)
time = (0:0.1:3).';
x = 10 .* time;
y = 0.2 .* time.^2;
profile = lts.correlation.CorrelationReplayProfile( ...
    'Time', time, 'Distance', cumtrapz(time, 10 * ones(size(time))), ...
    'Throttle', zeros(size(time)), 'Brake', zeros(size(time)), ...
    'Steer', zeros(size(time)), 'Speed', 10 * ones(size(time)), ...
    'X', x, 'Y', y);
stateLog = struct('controlTime', time, 'x', x + 100, 'y', y - 40);
exact = lts.correlation.CorrelationScore.evaluate(stateLog, profile);
verifyEqual(testCase, exact.gpsTrace, 0, 'AbsTol', 1e-12);
stateLog.y = stateLog.y + 0.5 .* time.^2;
offset = lts.correlation.CorrelationScore.evaluate(stateLog, profile);
verifyGreaterThan(testCase, offset.gpsTrace, 0);
verifyGreaterThan(testCase, offset.score, 0);
end

function testTuneRejectsBothReplayAndMotecInput(testCase)
checkpoint = [tempname '_correlation_input_test'];
cleanup = onCleanup(@() localRemoveFolder(checkpoint)); %#ok<NASGU>
verifyError(testCase, @() lts.app.tune_correlation( ...
    'LegacyDiagnostic', true, ...
    'ReplayCsv', 'already_normalized.csv', ...
    'MoTeCFile', 'raw.ld', ...
    'CheckpointDirectory', checkpoint, ...
    'Workers', 1), ...
    'tune_correlation:AmbiguousInput');
end

function testReducedReplayRespondsToPhysicalCandidate(testCase)
root = lts.util.repoRoot(mfilename('fullpath'));
registry = lts.correlation.CorrelationParameterRegistry.load(fullfile( ...
    root, 'config', 'correlation', 'lap5_ml_parameter_space.json'));
valuesA = lts.correlation.CorrelationParameterRegistry.baseline(registry);
valuesB = valuesA;
names = lts.correlation.CorrelationParameterRegistry.names(registry);
valuesB(names == "cda") = 3.8;

time = (0:0.01:0.2).';
speed = 15 * ones(size(time));
radius = 0.2032;
fdr = 3.36;
motorRpm = speed / radius * fdr * 60 / (2*pi);
profile = lts.correlation.CorrelationReplayProfile( ...
    'Time', time, 'Distance', speed .* time, ...
    'Throttle', zeros(size(time)), 'Brake', zeros(size(time)), ...
    'BrakePressureFrontBar', zeros(size(time)), ...
    'BrakePressureRearBar', zeros(size(time)), ...
    'MotorTorqueCommandNm', zeros(size(time)), ...
    'MotorTorqueDeliveredNm', zeros(size(time)), ...
    'MotorRpm', motorRpm, 'Steer', zeros(size(time)), ...
    'Speed', speed, 'YawRate', zeros(size(time)), ...
    'LatAccelG', zeros(size(time)), 'LongAccelG', zeros(size(time)), ...
    'WheelSpeedFL', speed, 'WheelSpeedFR', speed);
cfg = lts.vehicles.R25_correlation_tuning(lts.vehicles.R25());
track = lts.components.TestTrack('straight10');
windows = table(1, 0, 0.2, "train", ...
    'VariableNames', {'window_id', 'start_s', 'horizon_s', 'split'});

scoreA = lts.correlation.CorrelationTuningEvaluator.evaluateCandidate( ...
    1, valuesA, registry, cfg, profile, track, windows, ...
    'Dt', 0.01, 'ExcludeInitialS', 0.05, 'Split', "train");
scoreB = lts.correlation.CorrelationTuningEvaluator.evaluateCandidate( ...
    2, valuesB, registry, cfg, profile, track, windows, ...
    'Dt', 0.01, 'ExcludeInitialS', 0.05, 'Split', "train");

verifyTrue(testCase, isfinite(scoreA.score));
verifyTrue(testCase, isfinite(scoreB.score));
verifyGreaterThan(testCase, abs(scoreA.score - scoreB.score), 1e-12);
end

function testCandidateReusesVehicleAcrossWindows(testCase)
% A1 regression guard: a candidate with multiple windows must build the
% vehicle once and reuse it (via resetForSimulation) for each window. The
% per-window scores must be finite and the candidate must not rebuild the
% .tir/.mat each time. We assert finiteness across two windows of the same
% candidate, which exercises the shared-vehicle path.
root = lts.util.repoRoot(mfilename('fullpath'));
registry = lts.correlation.CorrelationParameterRegistry.load(fullfile( ...
    root, 'config', 'correlation', 'lap5_ml_parameter_space.json'));
values = lts.correlation.CorrelationParameterRegistry.baseline(registry);

time = (0:0.01:0.4).';
speed = 15 * ones(size(time));
radius = 0.2032;
fdr = 3.36;
motorRpm = speed / radius * fdr * 60 / (2*pi);
profile = lts.correlation.CorrelationReplayProfile( ...
    'Time', time, 'Distance', speed .* time, ...
    'Throttle', zeros(size(time)), 'Brake', zeros(size(time)), ...
    'BrakePressureFrontBar', zeros(size(time)), ...
    'BrakePressureRearBar', zeros(size(time)), ...
    'MotorTorqueCommandNm', zeros(size(time)), ...
    'MotorTorqueDeliveredNm', zeros(size(time)), ...
    'MotorRpm', motorRpm, 'Steer', zeros(size(time)), ...
    'Speed', speed, 'YawRate', zeros(size(time)), ...
    'LatAccelG', zeros(size(time)), 'LongAccelG', zeros(size(time)), ...
    'WheelSpeedFL', speed, 'WheelSpeedFR', speed);
cfg = lts.vehicles.R25_correlation_tuning(lts.vehicles.R25());
track = lts.components.TestTrack('straight10');
% Two windows sharing the same candidate.
windows = table([1; 2], [0; 0.2], [0.2; 0.2], ["train"; "train"], ...
    'VariableNames', {'window_id', 'start_s', 'horizon_s', 'split'});

[summary, detail] = lts.correlation.CorrelationTuningEvaluator.evaluateCandidate( ...
    1, values, registry, cfg, profile, track, windows, ...
    'Dt', 0.01, 'ExcludeInitialS', 0.05, 'Split', "train");

verifyEqual(testCase, summary.completed_windows, 2);
verifyTrue(testCase, all(isfinite(detail.score)), ...
    'Both windows must produce finite scores under the shared vehicle.');
verifyTrue(testCase, all(~contains(detail.status, "error")), ...
    'No window may report an error status.');
end

function profile = localProfile(time)
profile = lts.correlation.CorrelationReplayProfile( ...
    'Time', time, ...
    'Distance', 10 * (time - time(1)), ...
    'Throttle', zeros(size(time)), ...
    'Brake', zeros(size(time)), ...
    'Steer', zeros(size(time)), ...
    'Speed', 10 * ones(size(time)));
end

function testUntrustedVehicleFunctionNameIsRejected(testCase)
% C3 regression: a caller-supplied vehicle/tuning function name that does
% not resolve into a trusted lts.* package must be rejected rather than
% resolved with str2func + executed.
% A bare car name is still resolved under lts.vehicles.* (trusted).
cfg = lts.correlation.CorrelationAppSupport.loadVehicleConfig('R25');
verifyEqual(testCase, string(cfg.name), "R25");
% A dotted name outside the trusted prefixes is rejected.
verifyError(testCase, @() ...
    lts.correlation.CorrelationAppSupport.loadVehicleConfig('evil.steal'), ...
    'run_correlation:UntrustedFunctionName');
% A trusted-package dotted name resolves to a VehicleConfig.
trusted = lts.correlation.CorrelationAppSupport.loadVehicleConfig('lts.vehicles.R25');
verifyTrue(testCase, isa(trusted, 'lts.vehicle.VehicleConfig'));
end

function localRemoveFolder(folder)
if exist(folder, 'dir')
    rmdir(folder, 's');
end
end
