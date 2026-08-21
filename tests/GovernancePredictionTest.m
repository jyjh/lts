function tests = GovernancePredictionTest
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
root = lts.util.repoRoot(mfilename('fullpath'));
testCase.TestData.root = root;
testCase.TestData.manifestFile = fullfile(root, 'config', 'governance', ...
    'r25_parameter_manifest.json');
testCase.TestData.artifactFile = fullfile(root, 'config', 'governance', ...
    'r25_provisional_calibration.json');
end

function testManifestAndArtifactLoad(testCase)
manifest = lts.governance.ParameterManifest.load( ...
    testCase.TestData.manifestFile);
artifact = lts.governance.CalibrationArtifact.load( ...
    testCase.TestData.artifactFile, manifest);
verifyEqual(testCase, string(manifest.id), "r25-governed-v1");
verifyEqual(testCase, string(artifact.certification), "provisional");
verifyFalse(testCase, logical(artifact.legacy));
end

function testMalformedJsonRaisesTypedError(testCase)
% A truncated/malformed JSON file must produce a typed, file-named error
% from each loader rather than an opaque base jsondecode error.
[badFile, tmpDir] = localWriteTempJson('{not valid json');
cleanup = onCleanup(@() localRmdirRecursive(tmpDir));
manifest = lts.governance.ParameterManifest.load( ...
    testCase.TestData.manifestFile);
verifyError(testCase, @() ...
    lts.governance.ParameterManifest.load(badFile), ...
    'lts_governance_ParameterManifest:InvalidJson');
verifyError(testCase, @() ...
    lts.governance.CalibrationArtifact.load(badFile, manifest), ...
    'lts_governance_CalibrationArtifact:InvalidJson');
verifyError(testCase, @() ...
    lts.governance.DatasetCatalog.load(badFile), ...
    'lts_governance_DatasetCatalog:InvalidJson');
verifyError(testCase, @() ...
    lts.correlation.CorrelationParameterRegistry.load(badFile), ...
    'lts_correlation_CorrelationParameterRegistry:InvalidJson');
verifyError(testCase, @() ...
    lts.prediction.DesignChange.load(badFile), ...
    'lts_prediction_DesignChange:InvalidJson');
end

function testMeasuredAndDesignParametersCannotBeFitted(testCase)
manifest = lts.governance.ParameterManifest.load( ...
    testCase.TestData.manifestFile);
verifyError(testCase, @() ...
    lts.governance.ParameterManifest.assertCalibratable(manifest, "total_mass"), ...
    'lts_governance_ParameterManifest:FitForbidden');
verifyError(testCase, @() ...
    lts.governance.ParameterManifest.assertCalibratable(manifest, "wheelbase"), ...
    'lts_governance_ParameterManifest:FitForbidden');
end

function testLegacyTunerRequiresExplicitDiagnosticOptIn(testCase)
verifyError(testCase, @() lts.app.tune_correlation(), ...
    'tune_correlation:LegacyOptInRequired');
end

function testLegacyArtifactCannotBuildGovernedVehicle(testCase)
manifest = lts.governance.ParameterManifest.load( ...
    testCase.TestData.manifestFile);
artifact = lts.governance.CalibrationArtifact.load( ...
    testCase.TestData.artifactFile, manifest);
artifact.legacy = true;
verifyError(testCase, @() lts.governance.GovernedVehicle.build( ...
    @lts.vehicles.R25, manifest, artifact), ...
    'lts_governance_CalibrationArtifact:LegacyForbidden');
end

function testMassChangeUpdatesCoupledPropertiesWithoutRefit(testCase)
governed = localGoverned(testCase);
change = struct('schema', "lts.prediction.design-change.v1", ...
    'id', "remove_mass", 'source', "scale", ...
    'provenance', "weighed component and installed position", ...
    'operations', struct('type', "mass", 'deltaMassKg', -5, ...
        'xFromCgM', 0.2, 'yFromCgM', 0.1, 'zFromGroundM', 0.15));
variant = lts.prediction.DesignChange.apply(governed, change);
verifyEqual(testCase, variant.config.totalMass, ...
    governed.config.totalMass - 5, 'AbsTol', 1e-12);
verifyNotEqual(testCase, variant.config.cgHeight, governed.config.cgHeight);
verifyNotEqual(testCase, variant.config.yawInertia, governed.config.yawInertia);
verifyEqual(testCase, variant.calibrationId, governed.calibrationId);
verifyEqual(testCase, variant.artifact, governed.artifact);
end

function testIncompleteAeroChangeRejected(testCase)
change = struct('schema', "lts.prediction.design-change.v1", ...
    'id', "bad_aero", 'source', "CFD", 'provenance', "partial", ...
    'operations', struct('type', "aero_map", 'ClA', 5, 'CdA', 2));
verifyError(testCase, @() lts.prediction.DesignChange.validate(change), ...
    'lts_prediction_DesignChange:IncompleteOperation');
end

function testPairedUncertaintyIsSeededAndDoesNotMutateBaseline(testCase)
governed = localGoverned(testCase);
a = lts.prediction.PairedUncertainty.sampleArtifacts(governed, 5, 42);
b = lts.prediction.PairedUncertainty.sampleArtifacts(governed, 5, 42);
verifyEqual(testCase, [a.parameters], [b.parameters]);
verifyEqual(testCase, governed.artifact.parameters(1).value, 130);
end

function testIdentifiabilityFlagsConfoundedColumns(testCase)
x = linspace(0, 1, 20).';
report = lts.validation.IdentifiabilityReport.analyze( ...
    [x, 2*x], ["a", "b"]);
verifyFalse(testCase, report.fullRank);
verifyEqual(testCase, numel(report.confoundedPairs), 1);
end

function testDatasetCatalogStartsProvisional(testCase)
catalog = lts.governance.DatasetCatalog.load(fullfile( ...
    testCase.TestData.root, 'config', 'governance', ...
    'r25_dataset_catalog.json'));
verifyEqual(testCase, ...
    lts.governance.DatasetCatalog.maximumCertification(catalog), ...
    "provisional");
end

function testValidationDatasetCannotLeakIntoCalibration(testCase)
manifest = lts.governance.ParameterManifest.load( ...
    testCase.TestData.manifestFile);
dataset = struct('id', "held_out", 'role', "baseline-validation", ...
    'vehicleConfiguration', "R25", 'testDay', "synthetic", ...
    'sourceFile', "does_not_matter.csv", ...
    'sha256', repmat('0', 1, 64), 'conditions', struct(), ...
    'intervention', struct());
catalog = struct('schema', "lts.governance.dataset-catalog.v1", ...
    'datasets', dataset);
stage = struct('name', "mass_properties", ...
    'parameters', "yaw_inertia", ...
    'residual', @(candidate) candidate.yawInertia - 150);
verifyError(testCase, @() lts.app.calibrate_plant( ...
    'Manifest', manifest, 'DatasetCatalog', catalog, ...
    'DatasetIds', "held_out", 'Stages', stage), ...
    'calibrate_plant:DatasetLeakage');
end

function testCertificationGateRequiresNoRefitAndAccurateDelta(testCase)
catalog = struct('schema', "lts.governance.dataset-catalog.v1", ...
    'datasets', struct('id', "ballast_ab", ...
        'role', "transport-validation", ...
        'vehicleConfiguration', "R25_ballast", 'testDay', "synthetic", ...
        'sourceFile', "ballast.csv", 'sha256', repmat('0', 1, 64), ...
        'conditions', struct(), 'intervention', struct('deltaMassKg', 5)));
baseline = struct('absoluteErrorFraction', 0.01);
transport = struct('measuredDeltaS', 0.5, 'predictedDeltaS', 0.6, ...
    'variantRefitted', false);
% VerifySources=false: this catalog is synthetic (nonexistent sourceFile),
% used only to exercise the acceptance math, not source integrity.
result = lts.validation.CertificationGate.assess( ...
    catalog, baseline, transport, 'VerifySources', false);
verifyEqual(testCase, result.certification, "transport-validated");
transport.variantRefitted = true;
result = lts.validation.CertificationGate.assess( ...
    catalog, baseline, transport, 'VerifySources', false);
verifyEqual(testCase, result.certification, "baseline-validated");
end

function testCertificationGateTreatsNanDeltasAsFailure(testCase)
% A NaN delta previously made sign() return NaN, and logical(NaN) errors.
% It must be treated as a transport failure, not a crash.
catalog = struct('schema', "lts.governance.dataset-catalog.v1", ...
    'datasets', struct('id', "ballast_ab", ...
        'role', "transport-validation", ...
        'vehicleConfiguration', "R25_ballast", 'testDay', "synthetic", ...
        'sourceFile', "ballast.csv", 'sha256', repmat('0', 1, 64), ...
        'conditions', struct(), 'intervention', struct('deltaMassKg', 5)));
baseline = struct('absoluteErrorFraction', 0.01);
transport = struct('measuredDeltaS', NaN, 'predictedDeltaS', 0.6, ...
    'variantRefitted', false);
result = lts.validation.CertificationGate.assess( ...
    catalog, baseline, transport, 'VerifySources', false);
verifyFalse(testCase, result.transportPassed);
verifyEqual(testCase, result.certification, "baseline-validated");
end

function testCertificationGateVerifiesSourcesByDefault(testCase)
% By default assess() must refuse to certify against a catalog whose
% recorded source hashes were never checked. A synthetic catalog with a
% nonexistent sourceFile must error rather than silently certify.
root = lts.util.repoRoot(mfilename('fullpath'));
tmpDir = fullfile(root, 'exports', 'tmp_cert_test');
mkdir(tmpDir);
cleanup = onCleanup(@() localRmdirRecursive(tmpDir));
catalogPath = fullfile(tmpDir, 'catalog.json');
catalog = struct('schema', "lts.governance.dataset-catalog.v1", ...
    'datasets', struct('id', "ballast_ab", ...
        'role', "transport-validation", ...
        'vehicleConfiguration', "R25_ballast", 'testDay', "synthetic", ...
        'sourceFile', "ballast.csv", 'sha256', repmat('0', 1, 64), ...
        'conditions', struct(), 'intervention', struct('deltaMassKg', 5)));
fid = fopen(catalogPath, 'w');
fprintf(fid, '%s', jsonencode(catalog));
fclose(fid);
baseline = struct('absoluteErrorFraction', 0.01);
transport = struct('measuredDeltaS', 0.5, 'predictedDeltaS', 0.6, ...
    'variantRefitted', false);
verifyError(testCase, @() lts.validation.CertificationGate.assess( ...
    catalogPath, baseline, transport), ...
    'lts_governance_DatasetCatalog:MissingSource');
end

function testDatasetCatalogRejectsSourcePathOutsideRoot(testCase)
% C2 regression: a catalog sourceFile that escapes rootDirectory via '..'
% must be rejected, not followed. Otherwise a malicious/buggy catalog could
% read (or hash) arbitrary files outside the repo.
root = lts.util.repoRoot(mfilename('fullpath'));
tmpDir = fullfile(root, 'exports', 'tmp_catalog_path_test');
outsideDir = fullfile(root, 'exports', 'tmp_catalog_outside');
mkdir(tmpDir); mkdir(outsideDir);
cleanup = onCleanup(@() localRemoveTwo(tmpDir, outsideDir));

% A real file inside tmpDir whose hash we record correctly (accepted case).
insideFile = fullfile(tmpDir, 'inside.csv');
fid = fopen(insideFile, 'w'); fprintf(fid, 'hello\n'); fclose(fid);
% Compute the hash inline (DatasetCatalog.sha256 is private) using the same
% Java SHA-256 the catalog uses, so the accepted-case hash matches exactly.
insideHash = localSha256(insideFile);

% An escape path: ../tmp_catalog_outside/secret.csv points outside tmpDir.
escapeFile = fullfile(outsideDir, 'secret.csv');
fid = fopen(escapeFile, 'w'); fprintf(fid, 'secret\n'); fclose(fid);
escapeHash = localSha256(escapeFile);

% Legitimate in-root entry verifies successfully.
goodCatalog = struct('schema', "lts.governance.dataset-catalog.v1", ...
    'datasets', struct('id', "ok", 'role', "calibration", ...
    'vehicleConfiguration', "R25", 'testDay', "t", ...
    'sourceFile', "inside.csv", 'sha256', insideHash, ...
    'conditions', struct(), 'intervention', struct()));
report = lts.governance.DatasetCatalog.verifySources(goodCatalog, tmpDir);
verifyTrue(testCase, report(1).verified);

% Escape entry must be rejected with PathOutsideRoot (hash is correct, so
% only the path confinement check catches it).
escapeRelative = fullfile('..', 'tmp_catalog_outside', 'secret.csv');
badCatalog = struct('schema', "lts.governance.dataset-catalog.v1", ...
    'datasets', struct('id', "escape", 'role', "calibration", ...
    'vehicleConfiguration', "R25", 'testDay', "t", ...
    'sourceFile', escapeRelative, ...
    'sha256', escapeHash, 'conditions', struct(), 'intervention', struct()));
verifyError(testCase, @() ...
    lts.governance.DatasetCatalog.verifySources(badCatalog, tmpDir), ...
    'lts_governance_DatasetCatalog:PathOutsideRoot');
end

function localRemoveTwo(dirA, dirB)
if exist(dirA, 'dir'); rmdir(dirA, 's'); end
if exist(dirB, 'dir'); rmdir(dirB, 's'); end
end

function digest = localSha256(file)
% Mirror DatasetCatalog.sha256 (private) so tests can build a matching hash.
fid = fopen(file, 'rb');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
bytes = fread(fid, Inf, '*uint8');
engine = java.security.MessageDigest.getInstance('SHA-256');
engine.update(bytes);
raw = typecast(engine.digest(), 'uint8');
digest = lower(reshape(dec2hex(raw, 2).', 1, []));
end

function localRmdirRecursive(dir)
if exist(dir, 'dir')
    rmdir(dir, 's');
end
end

function [filePath, tmpDir] = localWriteTempJson(content)
root = lts.util.repoRoot(mfilename('fullpath'));
tmpDir = fullfile(root, 'exports', 'tmp_json_test');
mkdir(tmpDir);
filePath = fullfile(tmpDir, 'malformed.json');
fid = fopen(filePath, 'w');
fprintf(fid, '%s', content);
fclose(fid);
end

function testStagedCalibrationImprovesGovernedParameter(testCase)
manifest = lts.governance.ParameterManifest.load( ...
    testCase.TestData.manifestFile);
config = lts.vehicles.R25();
stage = struct('name', "mass_properties", ...
    'parameters', "yaw_inertia", ...
    'residual', @(candidate) (candidate.yawInertia - 150) / 2);
output = lts.calibration.StagedCalibrator.fit(config, manifest, stage, ...
    'MaxIterations', 80, 'ArtifactId', 'synthetic');
verifyLessThan(testCase, abs(output.config.yawInertia - 150), ...
    abs(config.yawInertia - 150));
verifyEqual(testCase, string(output.artifact.certification), "provisional");
verifyTrue(testCase, output.stages.identifiability.fullRank);
end

function testDesignStudyRunsPairedWithoutRefit(testCase)
    assumeTrue(testCase, tireDataAvailable(), 'TTC tire data not present: see src/+lts/+components/+Tire/README.md');
governed = localGoverned(testCase);
change = lts.prediction.DesignChange.load(fullfile( ...
    testCase.TestData.root, 'config', 'design_changes', ...
    'example_ballast_removal.json'));
track = lts.components.TestTrack('straight10');
% Dt must resolve the suspension/tire dynamics. At Dt=0.01 the stiff
% quarter-car modes diverge (damper velocities reach ~1-4 m/s vs ~0.04 m/s
% at the simulator's 0.001 design step), so the variant-faster assertion
% becomes a coincidence of the coarse integration rather than physics.
studyDt = 0.002;
result = lts.prediction.DesignStudy.run(governed, change, track, ...
    'AllowProvisional', true, ...
    'OptimizerOptions', struct('Dt', studyDt, ...
        'LineOffsetFractions', 0.65, 'UsageSchedule', 0.75));
verifyTrue(testCase, isfinite(result.baseline.lapTime));
verifyTrue(testCase, isfinite(result.variant.lapTime));
verifyFalse(testCase, result.variantRefitted);
verifyEqual(testCase, result.decisionStatus, "uncertified");
verifyLessThan(testCase, result.variant.lapTime, result.baseline.lapTime);
repeat = lts.prediction.HierarchicalOptimizer.optimize( ...
    governed.config, track, 'Dt', studyDt, ...
    'LineOffsetFractions', 0.65, 'UsageSchedule', 0.75);
verifyEqual(testCase, repeat.lapTime, result.baseline.lapTime, ...
    'AbsTol', 1e-12);
end

function testParallelOptimizerMatchesSerialResult(testCase)
% A3 regression: the opt-in Parallel grid (full UsageSchedule x
% LineOffsetFractions, no early-exit) must select the same winner lap time
% as the default serial path. Each candidate is an independent sim, so the
% result is order-independent.
governed = localGoverned(testCase);
track = lts.components.TestTrack('straight10');
serial = lts.prediction.HierarchicalOptimizer.optimize( ...
    governed.config, track, 'Dt', 0.01, ...
    'LineOffsetFractions', [0.45 0.65], 'UsageSchedule', [0.98 0.88]);
parallel = lts.prediction.HierarchicalOptimizer.optimize( ...
    governed.config, track, 'Dt', 0.01, ...
    'LineOffsetFractions', [0.45 0.65], 'UsageSchedule', [0.98 0.88], ...
    'Parallel', true);
verifyEqual(testCase, parallel.lapTime, serial.lapTime, 'AbsTol', 1e-9);
verifyEqual(testCase, parallel.feasible, serial.feasible);
% Parallel evaluates the full grid (2x2=4); serial early-exits at the first
% feasible tier, so it explores fewer. Confirm parallel at least covers it.
verifyGreaterThanOrEqual(testCase, numel(parallel.candidates), ...
    numel(serial.candidates));
end

function testCleanR25AndLegacyOverlayAreSeparated(testCase)
clean = lts.vehicles.R25();
legacy = lts.vehicles.R25_correlation_tuning(clean);
verifyEqual(testCase, clean.tire.tirFile, ...
    'Hoosier 43100 18.0x6.0-10 R20_7.tir');
verifyEqual(testCase, legacy.tire.tirFile, ...
    'Hoosier 43100 18.0x6.0-10 R20_7 - Scaled.tir');
verifyEqual(testCase, clean.aero.CdA, 1.81, 'AbsTol', 1e-12);
verifyEqual(testCase, legacy.aero.CdA, 2.65, 'AbsTol', 1e-12);
end

function testVehicleConfigRejectsObviousTypos(testCase)
    assumeTrue(testCase, tireDataAvailable(), 'TTC tire data not present: see src/+lts/+components/+Tire/README.md');
% B5 regression: the build boundary must catch a nonsensical vehicle-level
% scalar instead of letting it surface deep in simulation as NaN/div0.
cfg = lts.vehicles.R25();
track = lts.components.TestTrack('straight10');
verifyError(testCase, @() lts.vehicle.VehicleManager.fromConfig( ...
    localWith(cfg, 'totalMass', 0), track, 0.01, 'Verbose', false), ...
    'lts_vehicle_VehicleConfig:OutOfRange');
verifyError(testCase, @() lts.vehicle.VehicleManager.fromConfig( ...
    localWith(cfg, 'wheelbase', -1), track, 0.01, 'Verbose', false), ...
    'lts_vehicle_VehicleConfig:OutOfRange');
verifyError(testCase, @() lts.vehicle.VehicleManager.fromConfig( ...
    localWith(cfg, 'staticFrontWeight', 1.5), track, 0.01, 'Verbose', false), ...
    'lts_vehicle_VehicleConfig:InvalidFraction');
verifyError(testCase, @() lts.vehicle.VehicleManager.fromConfig( ...
    localWith(cfg, 'totalMass', NaN), track, 0.01, 'Verbose', false), ...
    'lts_vehicle_VehicleConfig:InvalidScalar');
invalidMass = cfg;
invalidMass.unsprungMass = invalidMass.totalMass / 4;
verifyError(testCase, @() lts.vehicle.VehicleManager.fromConfig( ...
    invalidMass, track, 0.01, 'Verbose', false), ...
    'lts_vehicle_VehicleConfig:InvalidMassBreakdown');
invalidTorsion = cfg;
invalidTorsion.chassis.torsionalRigidity = -1;
verifyError(testCase, @() lts.vehicle.VehicleManager.fromConfig( ...
    invalidTorsion, track, 0.01, 'Verbose', false), ...
    'lts_vehicle_VehicleConfig:InvalidTorsionalRigidity');
% A well-formed config must pass through unchanged.
built = lts.vehicle.VehicleManager.fromConfig(cfg, track, 0.01, 'Verbose', false);
verifyEqual(testCase, built.totalMass, cfg.totalMass, 'AbsTol', 1e-12);
end

function out = localWith(cfg, field, value)
out = cfg;
out.(field) = value;
end

function governed = localGoverned(testCase)
governed = lts.governance.GovernedVehicle.build(@lts.vehicles.R25, ...
    testCase.TestData.manifestFile, testCase.TestData.artifactFile);
end
