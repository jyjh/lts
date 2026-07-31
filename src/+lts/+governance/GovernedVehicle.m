classdef GovernedVehicle
    %GOVERNEDVEHICLE Build a vehicle from manifest-approved evidence.

    methods (Static)
        function governed = build(base, manifest, artifact, varargin)
            parser = inputParser;
            parser.addParameter('Scenario', struct(), @isstruct);
            parser.parse(varargin{:});
            if ischar(manifest) || isstring(manifest)
                manifest = lts.governance.ParameterManifest.load(manifest);
            else
                manifest = lts.governance.ParameterManifest.validate(manifest);
            end
            if ischar(artifact) || isstring(artifact)
                artifact = lts.governance.CalibrationArtifact.load(artifact, manifest);
            else
                artifact = lts.governance.CalibrationArtifact.validate(artifact, manifest);
            end
            lts.governance.CalibrationArtifact.assertProductionSafe(artifact);
            if isa(base, 'function_handle')
                config = base();
            else
                config = base;
            end
            if ~isa(config, 'lts.vehicle.VehicleConfig')
                error('lts_governance_GovernedVehicle:InvalidConfig', ...
                    'Base must be a VehicleConfig or factory returning one.');
            end
            config = lts.governance.CalibrationArtifact.apply(config, manifest, artifact);
            scenario = parser.Results.Scenario;
            fields = fieldnames(scenario);
            for i = 1:numel(fields)
                p = lts.governance.ParameterManifest.find(manifest, fields{i});
                if string(p.role) ~= "run_nuisance"
                    error('lts_governance_GovernedVehicle:ScenarioRoleMismatch', ...
                        'Scenario field "%s" is not governed as run_nuisance.', fields{i});
                end
                config = lts.governance.ParameterManifest.setValue( ...
                    config, p.path, scenario.(fields{i}));
            end
            governed = struct( ...
                'schema', "lts.governance.governed-vehicle.v1", ...
                'config', config, ...
                'manifest', manifest, ...
                'artifact', artifact, ...
                'scenario', scenario, ...
                'certification', string(artifact.certification), ...
                'calibrationId', string(artifact.id), ...
                'domainWarnings', ...
                    lts.governance.ParameterManifest.domainWarnings(manifest, config));
        end
    end
end
