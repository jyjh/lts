classdef PlantValidator
    %PLANTVALIDATOR Whole-run vehicle-plant validation without state resets.

    methods (Static)
        function report = validate(profile, governed, track, varargin)
            parser = inputParser;
            parser.addParameter('Dt', 0.001, @(x) isnumeric(x) && isscalar(x) && x > 0);
            parser.addParameter('WheelSolveIterations', 2, @isnumeric);
            parser.addParameter('MeasurementUncertainty', ...
                lts.validation.PlantValidator.defaultUncertainty(), @isstruct);
            parser.addParameter('TrendThreshold', 0.30, @isnumeric);
            parser.parse(varargin{:});
            opts = parser.Results;
            if ~isa(profile, 'lts.correlation.CorrelationReplayProfile')
                profile = lts.correlation.CorrelationReplayProfile.fromCsv(profile);
            end
            evalOptions = struct('Dt', opts.Dt, 'ExcludeInitialS', 0.1, ...
                'WheelSolveIterations', opts.WheelSolveIterations);
            [score, stateLog] = ...
                lts.correlation.CorrelationTuningEvaluator.simulateAndScore( ...
                    profile, governed.config, track, evalOptions);
            metrics = lts.validation.PlantValidator.metrics( ...
                stateLog, profile, opts.MeasurementUncertainty);
            trends = lts.validation.PlantValidator.residualTrends( ...
                stateLog, profile, opts.TrendThreshold);
            metricValues = [metrics.normalizedRmse];
            available = isfinite(metricValues);
            metricPass = any(available) && all(metricValues(available) <= 2);
            trendPass = isempty(trends.flagged);
            report = struct( ...
                'schema', "lts.validation.plant-report.v1", ...
                'mode', "whole-run-single-initial-state", ...
                'calibrationId', governed.calibrationId, ...
                'certification', governed.certification, ...
                'scoreDiagnostic', score, ...
                'metrics', metrics, ...
                'residualTrends', trends, ...
                'energyBalance', ...
                    lts.validation.PlantValidator.energyBalance( ...
                        stateLog, profile, governed.config.totalMass), ...
                'passed', metricPass && trendPass && score.status == "ok", ...
                'initializationCount', 1, ...
                'stateResetCount', 0, ...
                'notes', "Short reinitialized windows are diagnostic only.");
        end
    end

    methods (Static, Access = private)
        function uncertainty = defaultUncertainty()
            uncertainty = struct('positionM', 0.5, 'speedMps', 0.5, ...
                'yawRateRadps', 0.05, 'accelMps2', 0.980665, ...
                'wheelSpeedMps', 1.0);
        end

        function metrics = metrics(stateLog, profile, uncertainty)
            definitions = { ...
                'position', {'x', 'y'}, {'x', 'y'}, uncertainty.positionM; ...
                'speed', {'speed'}, {'speed'}, uncertainty.speedMps; ...
                'yaw_rate', {'yawRate'}, {'yawRate'}, uncertainty.yawRateRadps; ...
                'longitudinal_accel', {'ax'}, {'longAccelG'}, uncertainty.accelMps2; ...
                'lateral_accel', {'ay'}, {'latAccelG'}, uncertainty.accelMps2; ...
                'wheel_speed_fl', {'tireSpeed_FL'}, {'wheelSpeedFL'}, uncertainty.wheelSpeedMps; ...
                'wheel_speed_fr', {'tireSpeed_FR'}, {'wheelSpeedFR'}, uncertainty.wheelSpeedMps};
            metrics = repmat(struct('name', "", 'rmse', NaN, ...
                'uncertainty', NaN, 'normalizedRmse', NaN, 'sampleCount', 0), ...
                size(definitions, 1), 1);
            for i = 1:size(definitions, 1)
                name = string(definitions{i, 1});
                simFields = definitions{i, 2};
                refFields = definitions{i, 3};
                sigma = definitions{i, 4};
                if name == "position"
                    [sx, rx] = lts.validation.PlantValidator.aligned( ...
                        stateLog, profile, simFields{1}, refFields{1}, 1);
                    [sy, ry] = lts.validation.PlantValidator.aligned( ...
                        stateLog, profile, simFields{2}, refFields{2}, 1);
                    n = min([numel(sx), numel(rx), numel(sy), numel(ry)]);
                    if n > 0
                        residual = hypot((sx(1:n) - sx(1)) - (rx(1:n) - rx(1)), ...
                            (sy(1:n) - sy(1)) - (ry(1:n) - ry(1)));
                    else
                        residual = [];
                    end
                else
                    scale = 1;
                    if contains(name, "accel")
                        scale = 9.80665;
                    end
                    [sim, ref] = lts.validation.PlantValidator.aligned( ...
                        stateLog, profile, simFields{1}, refFields{1}, scale);
                    residual = sim - ref;
                end
                residual = residual(isfinite(residual));
                rmse = NaN;
                if ~isempty(residual)
                    rmse = sqrt(mean(residual.^2));
                end
                metrics(i) = struct('name', string(name), 'rmse', rmse, ...
                    'uncertainty', sigma, 'normalizedRmse', rmse / sigma, ...
                    'sampleCount', numel(residual));
            end
        end

        function trends = residualTrends(stateLog, profile, threshold)
            [simSpeed, refSpeed, time] = lts.validation.PlantValidator.aligned( ...
                stateLog, profile, 'speed', 'speed', 1);
            residual = simSpeed - refSpeed;
            inputs = {'speed', 'throttle', 'brake', 'steer'};
            values = {refSpeed, profile.throttle, profile.brake, profile.steer};
            coefficients = nan(numel(inputs), 1);
            flagged = strings(0, 1);
            for i = 1:numel(inputs)
                input = values{i};
                if numel(input) ~= numel(time)
                    input = interp1(profile.time, double(input(:)), time, 'linear', NaN);
                end
                valid = isfinite(input) & isfinite(residual);
                if nnz(valid) >= 3 && std(input(valid)) > eps && std(residual(valid)) > eps
                    c = corrcoef(input(valid), residual(valid));
                    coefficients(i) = c(1, 2);
                    if abs(coefficients(i)) >= threshold
                        flagged(end + 1, 1) = inputs{i}; %#ok<AGROW>
                    end
                end
            end
            trends = struct('inputs', string(inputs), ...
                'correlation', coefficients, ...
                'threshold', threshold, 'flagged', flagged);
        end

        function balance = energyBalance(stateLog, profile, mass)
            balance = struct('status', "unavailable", 'relativeClosureError', NaN);
            if ~isfield(stateLog, 'time') || ~isfield(stateLog, 'speed') || ...
                    isempty(stateLog.time) || isempty(profile.packVoltageV) || ...
                    isempty(profile.packCurrentA)
                return;
            end
            time = double(stateLog.time(:));
            packPower = double(profile.packVoltageV(:)) .* ...
                double(profile.packCurrentA(:));
            pack = interp1(profile.time, packPower, ...
                time, 'linear', NaN);
            valid = isfinite(pack) & isfinite(time);
            if nnz(valid) < 2
                return;
            end
            electrical = trapz(time(valid), pack(valid));
            speed = double(stateLog.speed(:));
            kineticChange = 0.5 * mass * (speed(end)^2 - speed(1)^2);
            lossForce = zeros(size(time));
            lossFields = {'F_drag', 'rollResistance', 'F_brake'};
            for i = 1:numel(lossFields)
                if isfield(stateLog, lossFields{i}) && ...
                        numel(stateLog.(lossFields{i})) == numel(time)
                    lossForce = lossForce + abs(double(stateLog.(lossFields{i})(:)));
                end
            end
            mechanicalLoss = trapz(time, lossForce .* abs(speed));
            closure = electrical - kineticChange - mechanicalLoss;
            balance.status = "approximate";
            balance.packEnergyJ = electrical;
            balance.kineticEnergyChangeJ = kineticChange;
            balance.modeledDissipationJ = mechanicalLoss;
            balance.closureErrorJ = closure;
            balance.relativeClosureError = abs(closure) / max(abs(electrical), 1);
            balance.note = "Approximate translational closure; rotational and thermal storage are not included.";
        end

        function [sim, ref, time] = aligned(stateLog, profile, simField, refField, refScale)
            sim = [];
            ref = [];
            time = [];
            if ~isfield(stateLog, 'controlTime')
                if ~isfield(stateLog, 'time')
                    return;
                end
                time = double(stateLog.time(:));
            else
                time = double(stateLog.controlTime(:));
            end
            if ~isfield(stateLog, simField) || ~isprop(profile, refField)
                return;
            end
            sim = double(stateLog.(simField)(:));
            reference = double(profile.(refField)(:)) * refScale;
            if numel(profile.time) < 2 || numel(reference) ~= numel(profile.time)
                sim = [];
                ref = [];
                time = [];
                return;
            end
            ref = interp1(profile.time, reference, time, 'linear', NaN);
            n = min([numel(sim), numel(ref), numel(time)]);
            sim = sim(1:n);
            ref = ref(1:n);
            time = time(1:n);
        end
    end
end
