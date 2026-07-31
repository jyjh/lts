classdef StagedCalibrator
    %STAGEDCALIBRATOR Prior-constrained identification of governed parameters.
    %
    % Each stage is a struct with:
    %   name       - sensor, longitudinal, braking, or lateral
    %   parameters - manifest parameter names
    %   residual   - function handle residual(config), one value per observation

    methods (Static)
        function output = fit(baseConfig, manifest, stages, varargin)
            parser = inputParser;
            parser.addParameter('MaxIterations', 100, ...
                @(x) isnumeric(x) && isscalar(x) && x > 0);
            parser.addParameter('FiniteDifferenceStep', 1e-3, ...
                @(x) isnumeric(x) && isscalar(x) && x > 0);
            parser.addParameter('RequireIdentifiable', true, ...
                @(x) islogical(x) || isnumeric(x));
            parser.addParameter('ArtifactId', 'provisional_calibration', ...
                @(x) ischar(x) || isstring(x));
            parser.parse(varargin{:});
            opts = parser.Results;
            if ischar(manifest) || isstring(manifest)
                manifest = lts.governance.ParameterManifest.load(manifest);
            else
                manifest = lts.governance.ParameterManifest.validate(manifest);
            end
            stages = lts.calibration.StagedCalibrator.asStructArray(stages);
            config = baseConfig;
            allParameters = strings(0, 1);
            reports = repmat(struct('name', "", 'parameters', strings(0, 1), ...
                'identifiability', struct(), 'initialLoss', NaN, ...
                'finalLoss', NaN, 'exitFlag', NaN), numel(stages), 1);
            values = struct();
            for s = 1:numel(stages)
                stage = stages(s);
                if ~isfield(stage, 'name') || ~isfield(stage, 'parameters') || ...
                        ~isfield(stage, 'residual') || ~isa(stage.residual, 'function_handle')
                    error('lts_calibration_StagedCalibrator:InvalidStage', ...
                        'Every stage needs name, parameters, and residual callback.');
                end
                names = string(stage.parameters(:));
                lts.governance.ParameterManifest.assertCalibratable(manifest, names);
                if any(ismember(names, allParameters))
                    error('lts_calibration_StagedCalibrator:RepeatedParameter', ...
                        'A global parameter may be identified in only one stage.');
                end
                allParameters = [allParameters; names]; %#ok<AGROW>
                [z0, parameters] = lts.calibration.StagedCalibrator.encodeConfig( ...
                    config, manifest, names);
                residual0 = lts.calibration.StagedCalibrator.residual( ...
                    z0, config, parameters, stage.residual, true);
                sensitivity = lts.calibration.StagedCalibrator.sensitivity( ...
                    z0, config, parameters, stage.residual, opts.FiniteDifferenceStep);
                identifiability = lts.validation.IdentifiabilityReport.analyze( ...
                    sensitivity, names);
                if logical(opts.RequireIdentifiable) && ~identifiability.fullRank
                    error('lts_calibration_StagedCalibrator:NotIdentifiable', ...
                        'Stage "%s" has rank %d for %d parameters.', ...
                        stage.name, identifiability.rank, identifiability.parameterCount);
                end
                objective = @(z) sum(lts.calibration.StagedCalibrator.residual( ...
                    z, config, parameters, stage.residual, true).^2);
                options = optimset('Display', 'off', 'MaxIter', opts.MaxIterations, ...
                    'MaxFunEvals', max(200, 20 * numel(z0)));
                [winner, finalLoss, exitFlag] = fminsearch(objective, z0, options);
                config = lts.calibration.StagedCalibrator.decodeConfig( ...
                    config, parameters, winner);
                for i = 1:numel(parameters)
                    values.(char(names(i))) = ...
                        lts.governance.ParameterManifest.getValue( ...
                            config, parameters(i).path);
                end
                reports(s) = struct('name', string(stage.name), ...
                    'parameters', names, 'identifiability', identifiability, ...
                    'initialLoss', sum(residual0.^2), ...
                    'finalLoss', finalLoss, 'exitFlag', exitFlag);
            end
            artifactParameters = repmat(struct('name', "", 'value', NaN), ...
                numel(allParameters), 1);
            for i = 1:numel(allParameters)
                artifactParameters(i).name = allParameters(i);
                artifactParameters(i).value = values.(char(allParameters(i)));
            end
            artifact = struct('schema', ...
                lts.governance.CalibrationArtifact.Schema, ...
                'id', string(opts.ArtifactId), ...
                'manifestId', string(manifest.id), ...
                'certification', "provisional", ...
                'parameters', artifactParameters, ...
                'sourceDatasetIds', strings(0, 1), ...
                'provenance', "Generated by governed staged calibration", ...
                'legacy', false);
            artifact = lts.governance.CalibrationArtifact.validate(artifact, manifest);
            output = struct('config', config, 'artifact', artifact, ...
                'stages', reports);
        end
    end

    methods (Static, Access = private)
        function stages = asStructArray(raw)
            if iscell(raw)
                stages = vertcat(raw{:});
            else
                stages = raw(:);
            end
        end

        function [z, params] = encodeConfig(config, manifest, names)
            params = repmat(struct(), numel(names), 1);
            z = zeros(numel(names), 1);
            for i = 1:numel(names)
                p = lts.governance.ParameterManifest.find(manifest, names(i));
                value = lts.governance.ParameterManifest.getValue(config, p.path);
                lo = p.calibrationDomain.lower;
                hi = p.calibrationDomain.upper;
                fraction = lts.util.clamp((value - lo) / max(hi - lo, eps), ...
                    1e-6, 1 - 1e-6);
                z(i) = log(fraction / (1 - fraction));
                params(i).name = string(p.name);
                params(i).path = string(p.path);
                params(i).lower = lo;
                params(i).upper = hi;
                params(i).prior = value;
                params(i).sigma = max(double(p.uncertainty.standardDeviation), eps);
            end
        end

        function config = decodeConfig(config, params, z)
            fraction = 1 ./ (1 + exp(-z(:)));
            for i = 1:numel(params)
                value = params(i).lower + fraction(i) * ...
                    (params(i).upper - params(i).lower);
                config = lts.governance.ParameterManifest.setValue( ...
                    config, params(i).path, value);
            end
        end

        function r = residual(z, config, params, callback, includePrior)
            candidate = lts.calibration.StagedCalibrator.decodeConfig(config, params, z);
            r = double(callback(candidate));
            r = r(:);
            if isempty(r) || any(~isfinite(r))
                r = 1e6 * ones(max(numel(r), 1), 1);
            end
            if includePrior
                prior = zeros(numel(params), 1);
                for i = 1:numel(params)
                    value = lts.governance.ParameterManifest.getValue( ...
                        candidate, params(i).path);
                    prior(i) = (value - params(i).prior) / params(i).sigma;
                end
                r = [r; prior];
            end
        end

        function S = sensitivity(z, config, params, callback, step)
            base = lts.calibration.StagedCalibrator.residual( ...
                z, config, params, callback, false);
            S = zeros(numel(base), numel(z));
            for i = 1:numel(z)
                perturbed = z;
                perturbed(i) = perturbed(i) + step;
                r = lts.calibration.StagedCalibrator.residual( ...
                    perturbed, config, params, callback, false);
                if numel(r) ~= numel(base)
                    error('lts_calibration_StagedCalibrator:ResidualSizeChanged', ...
                        'Residual callback size changed during sensitivity analysis.');
                end
                S(:, i) = (r - base) / step;
            end
        end
    end
end
