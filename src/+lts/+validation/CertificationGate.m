classdef CertificationGate
    %CERTIFICATIONGATE Apply baseline and transport acceptance criteria.

    methods (Static)
        function result = assess(catalog, baselineResults, transportResults, varargin)
            parser = inputParser;
            parser.addParameter('VerifySources', true, @(x) ...
                islogical(x) || (isnumeric(x) && isscalar(x)));
            parser.addParameter('RootDirectory', '', @(x) ...
                ischar(x) || isstring(x));
            parser.parse(varargin{:});
            opts = parser.Results;

            if ischar(catalog) || isstring(catalog)
                catalog = lts.governance.DatasetCatalog.load(catalog);
            else
                catalog = lts.governance.DatasetCatalog.validate(catalog);
            end
            % Certification is the trust boundary for a production decision.
            % Refuse to compute it against sources whose recorded hashes were
            % never checked, unless the caller (e.g. a synthetic unit test)
            % explicitly opts out with VerifySources=false.
            if logical(opts.VerifySources)
                rootDirectory = char(opts.RootDirectory);
                lts.governance.DatasetCatalog.verifySources(catalog, rootDirectory);
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
                transportPass = transportPass && ...
                    lts.validation.CertificationGate.transportResultPasses(r);
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

        function tf = transportResultPasses(r)
            % A NaN in either delta makes sign() return NaN, and
            % logical(NaN) errors rather than being treated as false. Treat
            % any non-finite delta or a refit as a hard failure.
            predictedDelta = r.predictedDeltaS;
            measuredDelta = r.measuredDeltaS;
            if ~isfinite(predictedDelta) || ~isfinite(measuredDelta)
                tf = false;
                return;
            end
            tolerance = max(0.20 * abs(measuredDelta), 0.2);
            tf = sign(predictedDelta) == sign(measuredDelta) && ...
                abs(predictedDelta - measuredDelta) <= tolerance && ...
                ~logical(r.variantRefitted);
        end
    end
end
