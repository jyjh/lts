function tests = PlantValidationTest
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
root = lts.util.repoRoot(mfilename('fullpath'));
testCase.TestData.governed = lts.governance.GovernedVehicle.build( ...
    @lts.vehicles.R25, fullfile(root, 'config', 'governance', 'r25_parameter_manifest.json'), ...
    fullfile(root, 'config', 'governance', 'r25_provisional_calibration.json'));
time = (0:0.01:0.2).';
zero = zeros(size(time));
speed = 10 * ones(size(time));
testCase.TestData.profile = lts.correlation.CorrelationReplayProfile( ...
    'Time', time, 'Throttle', zero, 'Brake', zero, 'Steer', zero, ...
    'Speed', speed, 'MotorTorqueDeliveredNm', zero, ...
    'BrakePressureFrontBar', zero, 'BrakePressureRearBar', zero);
testCase.TestData.replay = [tempname '.csv'];
writetable(table(time, zero, zero, zero, speed, zero, zero, zero, ...
    'VariableNames', {'time_s', 'throttle_ratio', 'brake_ratio', 'steer_rad', ...
    'speed_mps', 'motor_torque_delivered_nm', ...
    'brake_pressure_front_bar', 'brake_pressure_rear_bar'}), testCase.TestData.replay);
end

function teardownOnce(testCase)
delete(testCase.TestData.replay);
end

function testValidatorRunsCompleteReplay(testCase)
assumeTrue(testCase, tireDataAvailable(), 'TTC tire data not present.');
report = lts.validation.PlantValidator.validate(testCase.TestData.profile, ...
    testCase.TestData.governed, lts.components.TestTrack('straight10'));
verifyEqual(testCase, report.calibrationId, testCase.TestData.governed.calibrationId);
verifyEqual(testCase, report.initializationCount, 1);
verifyEqual(testCase, report.stateResetCount, 0);
verifyEqual(testCase, report.scoreDiagnostic.status, "ok");
verifyGreaterThan(testCase, report.scoreDiagnostic.sampleCount, 0);
end

function testPublicValidationEntryPoint(testCase)
assumeTrue(testCase, tireDataAvailable(), 'TTC tire data not present.');
report = lts.app.run_plant_validation('ReplayCsv', testCase.TestData.replay, ...
    'Track', lts.components.TestTrack('straight10'), 'PreferGpsKinematics', false);
verifyEqual(testCase, report.schema, "lts.validation.plant-report.v1");
verifyEqual(testCase, report.scoreDiagnostic.status, "ok");
verifyTrue(testCase, isfield(report, 'preprocessing'));
end

function testValidatorPropagatesInvalidVehicle(testCase)
governed = testCase.TestData.governed;
governed.config.totalMass = 0;
verifyError(testCase, @() lts.validation.PlantValidator.validate( ...
    testCase.TestData.profile, governed, lts.components.TestTrack('straight10')), ...
    'lts_vehicle_VehicleConfig:OutOfRange');
end
