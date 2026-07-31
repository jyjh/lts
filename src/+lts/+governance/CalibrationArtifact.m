classdef CalibrationArtifact
    %CALIBRATIONARTIFACT Governed, immutable calibration result.

    properties (Constant)
        Schema = "lts.governance.calibration-artifact.v1"
        States = ["provisional", "baseline-validated", "transport-validated"]
    end

    methods (Static)
        function artifact = load(filePath, manifest)
            if ~exist(filePath, 'file')
                error('lts_governance_CalibrationArtifact:MissingFile', ...
                    'Calibration artifact does not exist: %s', filePath);
            end
            artifact = jsondecode(fileread(filePath));
            artifact = lts.governance.CalibrationArtifact.validate(artifact, manifest);
            artifact.sourceFile = char(filePath);
        end

        function artifact = validate(artifact, manifest)
            if ~isstruct(artifact) || ~isfield(artifact, 'schema') || ...
                    string(artifact.schema) ~= lts.governance.CalibrationArtifact.Schema
                error('lts_governance_CalibrationArtifact:InvalidSchema', ...
                    'Expected schema %s.', lts.governance.CalibrationArtifact.Schema);
            end
            required = {'id', 'manifestId', 'certification', 'parameters', ...
                'sourceDatasetIds', 'provenance', 'legacy'};
            for i = 1:numel(required)
                if ~isfield(artifact, required{i})
                    error('lts_governance_CalibrationArtifact:MissingField', ...
                        'Calibration artifact is missing "%s".', required{i});
                end
            end
            if string(artifact.manifestId) ~= string(manifest.id)
                error('lts_governance_CalibrationArtifact:ManifestMismatch', ...
                    'Artifact targets manifest "%s", not "%s".', ...
                    artifact.manifestId, manifest.id);
            end
            if ~any(string(artifact.certification) == ...
                    lts.governance.CalibrationArtifact.States)
                error('lts_governance_CalibrationArtifact:InvalidCertification', ...
                    'Unknown certification state "%s".', artifact.certification);
            end
            params = lts.governance.CalibrationArtifact.asStructArray(artifact.parameters);
            names = strings(numel(params), 1);
            for i = 1:numel(params)
                if ~isfield(params(i), 'name') || ~isfield(params(i), 'value') || ...
                        ~isscalar(params(i).value) || ~isfinite(params(i).value)
                    error('lts_governance_CalibrationArtifact:InvalidParameter', ...
                        'Artifact parameter %d requires a finite scalar name/value.', i);
                end
                names(i) = string(params(i).name);
            end
            if numel(unique(names)) ~= numel(names)
                error('lts_governance_CalibrationArtifact:DuplicateParameter', ...
                    'Artifact parameter names must be unique.');
            end
            lts.governance.ParameterManifest.assertCalibratable(manifest, names);
            artifact.parameters = params;
        end

        function assertProductionSafe(artifact)
            if logical(artifact.legacy)
                error('lts_governance_CalibrationArtifact:LegacyForbidden', ...
                    'Legacy/effective calibration artifacts cannot be used in design studies.');
            end
        end

        function config = apply(config, manifest, artifact)
            artifact = lts.governance.CalibrationArtifact.validate(artifact, manifest);
            lts.governance.CalibrationArtifact.assertProductionSafe(artifact);
            for i = 1:numel(artifact.parameters)
                p = lts.governance.ParameterManifest.find( ...
                    manifest, artifact.parameters(i).name);
                value = double(artifact.parameters(i).value);
                domain = p.calibrationDomain;
                if value < domain.lower || value > domain.upper
                    error('lts_governance_CalibrationArtifact:OutsideDomain', ...
                        'Calibrated parameter "%s" is outside its governed domain.', p.name);
                end
                config = lts.governance.ParameterManifest.setValue( ...
                    config, p.path, value);
            end
        end
    end

    methods (Static, Access = private)
        function params = asStructArray(raw)
            if isempty(raw)
                params = struct('name', {}, 'value', {});
            elseif iscell(raw)
                params = vertcat(raw{:});
            else
                params = raw(:);
            end
        end
    end
end
