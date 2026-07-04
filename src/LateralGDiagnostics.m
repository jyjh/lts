classdef LateralGDiagnostics
    % LATERALGDIAGNOSTICS Helpers for lateral-acceleration sanity checks.
    %
    % The raw lateral accelerometer is useful, but it should be checked
    % against kinematic references before it is used as a correlation target:
    %   ay_yaw = speed * yawRate / g
    %   ay_steer ~= speed^2 * tan(roadWheelSteer) / wheelbase / g

    properties (Constant)
        g = 9.80665
    end

    methods (Static)
        function report = assessSignals(time, rawLatG, speedMps, yawRateRadps, steerRad, wheelbase)
            if nargin < 6 || isempty(wheelbase) || ~isfinite(wheelbase) || wheelbase <= 0
                wheelbase = 1.5;
            end

            [time, rawLatG, speedMps, yawRateRadps, steerRad] = ...
                LateralGDiagnostics.trimToCommonLength( ...
                time, rawLatG, speedMps, yawRateRadps, steerRad);

            yawLatG = speedMps .* yawRateRadps ./ LateralGDiagnostics.g;
            steerLatG = speedMps.^2 .* tan(steerRad) ./ ...
                wheelbase ./ LateralGDiagnostics.g;

            validYaw = isfinite(rawLatG) & isfinite(yawLatG);
            activeYaw = validYaw & abs(rawLatG) > 0.5 & abs(yawLatG) > 0.2;
            signMismatch = activeYaw & sign(rawLatG) ~= sign(yawLatG);
            magnitudeMismatch = validYaw & abs(rawLatG) > 0.8 & ...
                abs(rawLatG) > max(1.5 * abs(yawLatG), abs(yawLatG) + 0.5);

            report = struct();
            report.time = time;
            report.rawLatG = rawLatG;
            report.yawLatG = yawLatG;
            report.steerLatG = steerLatG;
            report.speedMps = speedMps;
            report.sampleCount = numel(time);
            report.validYawSampleCount = nnz(validYaw);
            report.activeYawSampleCount = nnz(activeYaw);
            report.rawPeakAbsG = LateralGDiagnostics.maxAbs(rawLatG);
            report.rawP95AbsG = LateralGDiagnostics.percentileAbs(rawLatG, 95);
            report.yawPeakAbsG = LateralGDiagnostics.maxAbs(yawLatG);
            report.yawP95AbsG = LateralGDiagnostics.percentileAbs(yawLatG, 95);
            report.steerPeakAbsG = LateralGDiagnostics.maxAbs(steerLatG);
            report.steerP95AbsG = LateralGDiagnostics.percentileAbs(steerLatG, 95);
            report.signMismatchFraction = ...
                LateralGDiagnostics.safeFraction(nnz(signMismatch), nnz(activeYaw));
            report.magnitudeMismatchFraction = ...
                LateralGDiagnostics.safeFraction(nnz(magnitudeMismatch), nnz(validYaw));
            report.worstYawErrorG = LateralGDiagnostics.maxAbs(rawLatG - yawLatG);
            report.worstEvent = LateralGDiagnostics.worstEvent( ...
                time, rawLatG, yawLatG, steerLatG, speedMps);
            report.messages = LateralGDiagnostics.consistencyMessages(report);
        end

        function messages = consistencyMessages(report)
            messages = strings(0, 1);

            if report.validYawSampleCount < 3
                return;
            end

            if report.signMismatchFraction > 0.10
                messages(end + 1, 1) = sprintf( ...
                    ['Raw lateral accel disagrees in sign with speed*yawRate ' ...
                    'for %.1f%% of active samples. Check sensor axis/sign ' ...
                    'before fitting tire grip.'], ...
                    100 * report.signMismatchFraction);
            end

            if report.magnitudeMismatchFraction > 0.05
                messages(end + 1, 1) = sprintf( ...
                    ['Raw lateral accel is much larger than speed*yawRate ' ...
                    'for %.1f%% of samples. Treat raw accel peaks as suspect ' ...
                    'unless path/course data confirms them.'], ...
                    100 * report.magnitudeMismatchFraction);
            end

            if isfinite(report.rawPeakAbsG) && isfinite(report.yawPeakAbsG) && ...
                    report.rawPeakAbsG > max(1.3 * report.yawPeakAbsG, report.yawPeakAbsG + 0.4)
                messages(end + 1, 1) = sprintf( ...
                    ['Raw lateral peak %.2f g exceeds yaw-rate-derived peak ' ...
                    '%.2f g by a large margin.'], ...
                    report.rawPeakAbsG, report.yawPeakAbsG);
            end
        end

        function events = topMismatchEvents(time, rawLatG, yawLatG, steerLatG, speedMps, count, minSeparationSec)
            if nargin < 6 || isempty(count)
                count = 5;
            end
            if nargin < 7 || isempty(minSeparationSec)
                minSeparationSec = 0.25;
            end

            [time, rawLatG, yawLatG, steerLatG, speedMps] = ...
                LateralGDiagnostics.trimToCommonLength( ...
                time, rawLatG, yawLatG, steerLatG, speedMps);
            score = abs(rawLatG - yawLatG);
            score(~isfinite(score)) = -inf;

            events = repmat(LateralGDiagnostics.emptyEvent(), 0, 1);
            for idx = 1:count
                [bestScore, bestIdx] = max(score);
                if ~isfinite(bestScore) || bestScore < 0
                    break;
                end
                events(end + 1, 1) = LateralGDiagnostics.eventAt( ...
                    bestIdx, time, rawLatG, yawLatG, steerLatG, speedMps, bestScore); %#ok<AGROW>
                near = abs(time - time(bestIdx)) <= minSeparationSec;
                score(near) = -inf;
            end
        end
    end

    methods (Static, Access = private)
        function [varargout] = trimToCommonLength(varargin)
            lengths = cellfun(@numel, varargin);
            n = min(lengths);
            varargout = cell(size(varargin));
            for i = 1:numel(varargin)
                values = double(varargin{i}(:));
                if isempty(values)
                    values = NaN(n, 1);
                else
                    values = values(1:n);
                end
                varargout{i} = values;
            end
        end

        function value = maxAbs(values)
            values = abs(values(isfinite(values)));
            if isempty(values)
                value = NaN;
            else
                value = max(values);
            end
        end

        function value = percentileAbs(values, pct)
            values = sort(abs(values(isfinite(values))));
            if isempty(values)
                value = NaN;
                return;
            end

            rank = 1 + (numel(values) - 1) * pct / 100;
            lo = floor(rank);
            hi = ceil(rank);
            if lo == hi
                value = values(lo);
            else
                value = values(lo) * (hi - rank) + values(hi) * (rank - lo);
            end
        end

        function value = safeFraction(numerator, denominator)
            if denominator <= 0
                value = 0;
            else
                value = numerator / denominator;
            end
        end

        function event = worstEvent(time, rawLatG, yawLatG, steerLatG, speedMps)
            score = abs(rawLatG - yawLatG);
            score(~isfinite(score)) = -inf;
            [bestScore, idx] = max(score);
            if isempty(idx) || ~isfinite(bestScore) || bestScore < 0
                event = LateralGDiagnostics.emptyEvent();
                return;
            end
            event = LateralGDiagnostics.eventAt( ...
                idx, time, rawLatG, yawLatG, steerLatG, speedMps, bestScore);
        end

        function event = eventAt(idx, time, rawLatG, yawLatG, steerLatG, speedMps, score)
            event = struct( ...
                'time', time(idx), ...
                'rawLatG', rawLatG(idx), ...
                'yawLatG', yawLatG(idx), ...
                'steerLatG', steerLatG(idx), ...
                'speedMps', speedMps(idx), ...
                'scoreG', score);
        end

        function event = emptyEvent()
            event = struct( ...
                'time', NaN, ...
                'rawLatG', NaN, ...
                'yawLatG', NaN, ...
                'steerLatG', NaN, ...
                'speedMps', NaN, ...
                'scoreG', NaN);
        end
    end
end
