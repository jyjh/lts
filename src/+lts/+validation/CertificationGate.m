classdef CertificationGate
    %CERTIFICATIONGATE Apply baseline and transport acceptance criteria.

    methods (Static)
        function result = assess(catalog, baselineResults, transportResults)
            if ischar(catalog) || isstring(catalog)
                catalog = lts.governance.DatasetCatalog.load(catalog);
            else
                catalog = lts.governance.DatasetCatalog.validate(catalog);
            end
            baselineResults = lts.validation.CertificationGate.asStructArray( ...
                baselineResults);
            transportResults = lts.validation.CertificationGate.asStructArray( ...
                transportResults);
            baselinePass = ~isempty(baselineResults) && ...
                all([baselineResults.absoluteErrorFraction] <= 0.02);
            transportPass = ~isempty(transportResults);
            for i = 1:numel(transportResults)
                r = transportResults(i);
                tolerance = max(0.20 * abs(r.measuredDeltaS), 0.2);
                transportPass = transportPass && ...
                    sign(r.predictedDeltaS) == sign(r.measuredDeltaS) && ...
                    abs(r.predictedDeltaS - r.measuredDeltaS) <= tolerance && ...
                    ~logical(r.variantRefitted);
            end
            evidence = lts.governance.DatasetCatalog.maximumCertification(catalog);
            if baselinePass && transportPass && evidence == "transport-validated"
                state = "transport-validated";
            elseif baselinePass && evidence ~= "provisional"
                state = "baseline-validated";
            else
                state = "provisional";
            end
            result = struct('schema', "lts.validation.certification-gate.v1", ...
                'certification', state, 'baselinePassed', baselinePass, ...
                'transportPassed', transportPass, ...
                'absoluteToleranceFraction', 0.02, ...
                'deltaToleranceRule', "max(20% of measured delta, 0.2 s)");
        end
    end

    methods (Static, Access = private)
        function values = asStructArray(raw)
            if isempty(raw)
                values = struct([]);
            elseif iscell(raw)
                values = vertcat(raw{:});
            else
                values = raw(:);
            end
        end
    end
end
