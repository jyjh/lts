classdef PairedUncertainty
    %PAIREDUNCERTAINTY Deterministic shared calibration samples for A/B studies.

    methods (Static)
        function samples = sampleArtifacts(governed, count, seed)
            count = round(double(count));
            if count < 0 || ~isfinite(count)
                error('lts_prediction_PairedUncertainty:InvalidCount', ...
                    'Sample count must be a finite nonnegative integer.');
            end
            if count == 0
                samples = repmat(governed.artifact, 0, 1);
                return;
            end
            stream = RandStream('mt19937ar', 'Seed', double(seed));
            base = governed.artifact;
            samples = repmat(base, count, 1);
            for i = 1:numel(base.parameters)
                p = lts.governance.ParameterManifest.find( ...
                    governed.manifest, base.parameters(i).name);
                mu = double(base.parameters(i).value);
                sigma = double(p.uncertainty.standardDeviation);
                distribution = string(p.uncertainty.distribution);
                if distribution == "fixed" || sigma == 0
                    values = mu * ones(count, 1);
                elseif distribution == "normal"
                    values = mu + sigma .* randn(stream, count, 1);
                else
                    halfWidth = sqrt(3) * sigma;
                    values = mu - halfWidth + 2 * halfWidth .* rand(stream, count, 1);
                end
                values = min(max(values, p.calibrationDomain.lower), ...
                    p.calibrationDomain.upper);
                for k = 1:count
                    samples(k).parameters(i).value = values(k);
                    samples(k).id = sprintf('%s_sample_%04d', base.id, k);
                end
            end
        end

        function interval = summarize(values)
            values = sort(double(values(isfinite(values))));
            if isempty(values)
                interval = struct('median', NaN, 'lower95', NaN, ...
                    'upper95', NaN, 'sampleCount', 0);
                return;
            end
            interval = struct('median', median(values), ...
                'lower95', lts.prediction.PairedUncertainty.quantile(values, 0.025), ...
                'upper95', lts.prediction.PairedUncertainty.quantile(values, 0.975), ...
                'sampleCount', numel(values));
        end
    end

    methods (Static, Access = private)
        function q = quantile(values, probability)
            if numel(values) == 1
                q = values(1);
                return;
            end
            index = 1 + probability * (numel(values) - 1);
            lo = floor(index);
            hi = ceil(index);
            q = values(lo) + (index - lo) * (values(hi) - values(lo));
        end
    end
end
