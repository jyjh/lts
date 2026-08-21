classdef IdentifiabilityReport
    %IDENTIFIABILITYREPORT Diagnose rank and confounding in sensitivities.

    methods (Static)
        function report = analyze(sensitivity, parameterNames, varargin)
            parser = inputParser;
            parser.addParameter('RankTolerance', [], @isnumeric);
            parser.addParameter('CorrelationThreshold', 0.95, ...
                @(x) isnumeric(x) && isscalar(x) && x > 0 && x <= 1);
            parser.parse(varargin{:});
            S = double(sensitivity);
            names = string(parameterNames(:));
            if size(S, 2) ~= numel(names) || isempty(S) || any(~isfinite(S), 'all')
                error('lts_validation_IdentifiabilityReport:InvalidSensitivity', ...
                    'Sensitivity must be finite with one column per parameter.');
            end
            columnNorm = vecnorm(S, 2, 1);
            scaled = S ./ max(columnNorm, eps);
            singularValues = svd(scaled, 'econ');
            tolerance = parser.Results.RankTolerance;
            if isempty(tolerance)
                tolerance = max(size(scaled)) * eps(max(singularValues));
            end
            rankValue = nnz(singularValues > tolerance);
            correlation = corrcoef(scaled);
            pairs = struct('first', {}, 'second', {}, 'correlation', {});
            for i = 1:numel(names)
                for j = i + 1:numel(names)
                    if isfinite(correlation(i, j)) && ...
                            abs(correlation(i, j)) >= parser.Results.CorrelationThreshold
                        pairs(end + 1) = struct( ... %#ok<AGROW>
                            'first', names(i), 'second', names(j), ...
                            'correlation', correlation(i, j));
                    end
                end
            end
            report = struct( ...
                'schema', "lts.validation.identifiability-report.v1", ...
                'parameterNames', names, ...
                'rank', rankValue, ...
                'parameterCount', numel(names), ...
                'fullRank', rankValue == numel(names), ...
                'singularValues', singularValues, ...
                'conditionNumber', singularValues(1) / max(singularValues(end), eps), ...
                'columnNorms', columnNorm, ...
                'confoundedPairs', pairs);
        end
    end
end
