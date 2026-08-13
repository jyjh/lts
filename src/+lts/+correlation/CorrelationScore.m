classdef CorrelationScore
    %CORRELATIONSCORE Robust, horizon-weighted replay prediction score.

    methods (Static)
        function result = evaluate(stateLog, profile, varargin)
            parser = inputParser;
            parser.addParameter('ExcludeInitialS', 0.1, ...
                @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 0);
            parser.addParameter('FinalDriveRatio', 3.36, ...
                @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0);
            parser.addParameter('WheelRadiusM', 0.2032, ...
                @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x > 0);
            parser.parse(varargin{:});
            opts = parser.Results;

            time = lts.correlation.CorrelationScore.field(stateLog, 'controlTime', ...
                lts.correlation.CorrelationScore.field(stateLog, 'time', []));
            time = time(:);
            if isempty(time)
                result = lts.correlation.CorrelationScore.failedResult("empty_state_log");
                return;
            end
            horizon = max(time);
            mask = isfinite(time) & time >= opts.ExcludeInitialS;
            horizonWeight = 0.25 + 0.75 * min(1, max(0, time ./ max(horizon, eps)));

            groups = struct();
            groups.gps_trace = lts.correlation.CorrelationScore.traceLoss( ...
                lts.correlation.CorrelationScore.field(stateLog, 'x', []), ...
                lts.correlation.CorrelationScore.field(stateLog, 'y', []), ...
                profile.x, profile.y, profile.time, time, mask, horizonWeight, 1.0);
            groups.speed = lts.correlation.CorrelationScore.channelLoss( ...
                lts.correlation.CorrelationScore.field(stateLog, 'speed', []), ...
                profile.speed, profile.time, time, mask, horizonWeight, 1.0);
            groups.yaw_rate = lts.correlation.CorrelationScore.channelLoss( ...
                lts.correlation.CorrelationScore.field(stateLog, 'yawRate', []), ...
                profile.yawRate, profile.time, time, mask, horizonWeight, 0.20);

            lateral = [ ...
                lts.correlation.CorrelationScore.channelLoss(lts.correlation.CorrelationScore.field(stateLog, 'ay', []) / 9.80665, ...
                    profile.latAccelG, profile.time, time, mask, horizonWeight, 0.15), ...
                lts.correlation.CorrelationScore.channelLoss(lts.correlation.CorrelationScore.field(stateLog, 'frontAxleAy', []) / 9.80665, ...
                    profile.frontLatAccelG, profile.time, time, mask, horizonWeight, 0.15), ...
                lts.correlation.CorrelationScore.channelLoss(lts.correlation.CorrelationScore.field(stateLog, 'rearAxleAy', []) / 9.80665, ...
                    profile.rearLatAccelG, profile.time, time, mask, horizonWeight, 0.15)];
            if isfinite(lateral(1))
                groups.lateral_accel = lateral(1);
            else
                groups.lateral_accel = ...
                    lts.correlation.CorrelationScore.meanFinite(lateral(2:3));
            end

            groups.longitudinal_accel = lts.correlation.CorrelationScore.channelLoss( ...
                lts.correlation.CorrelationScore.field(stateLog, 'ax', []) / 9.80665, ...
                profile.longAccelG, profile.time, time, mask, horizonWeight, 0.15);

            rearCarrierRef = profile.motorRpm / opts.FinalDriveRatio * ...
                (2 * pi / 60) * opts.WheelRadiusM;
            rearCarrierSim = 0.5 * ( ...
                lts.correlation.CorrelationScore.field(stateLog, 'tireSpeed_RL', []) + ...
                lts.correlation.CorrelationScore.field(stateLog, 'tireSpeed_RR', []));
            wheels = [ ...
                lts.correlation.CorrelationScore.channelLoss(lts.correlation.CorrelationScore.field(stateLog, 'tireSpeed_FL', []), ...
                    profile.wheelSpeedFL, profile.time, time, mask, horizonWeight, 0.75), ...
                lts.correlation.CorrelationScore.channelLoss(lts.correlation.CorrelationScore.field(stateLog, 'tireSpeed_FR', []), ...
                    profile.wheelSpeedFR, profile.time, time, mask, horizonWeight, 0.75), ...
                lts.correlation.CorrelationScore.channelLoss(rearCarrierSim, rearCarrierRef, ...
                    profile.time, time, mask, horizonWeight, 0.75)];
            groups.wheel_speed = lts.correlation.CorrelationScore.meanFinite(wheels);

            % GPS-derived trace, speed, and body accelerations carry 75% of
            % the objective. Wheel speed remains an independent driveline
            % diagnostic rather than the vehicle-speed authority.
            names = {'gps_trace', 'speed', 'yaw_rate', 'lateral_accel', ...
                'longitudinal_accel', 'wheel_speed'};
            fixedWeights = [0.30, 0.20, 0.15, 0.15, 0.10, 0.10];
            values = nan(size(fixedWeights));
            for i = 1:numel(names)
                values(i) = groups.(names{i});
            end
            valid = isfinite(values);
            if ~any(valid)
                result = lts.correlation.CorrelationScore.failedResult("no_scorable_channels");
                return;
            end
            weights = fixedWeights(valid);
            weights = weights / sum(weights);
            total = sum(weights .* values(valid));
            result = struct( ...
                'score', total, ...
                'status', "ok", ...
                'sampleCount', nnz(mask), ...
                'gpsTrace', groups.gps_trace, ...
                'speed', groups.speed, ...
                'yawRate', groups.yaw_rate, ...
                'lateralAccel', groups.lateral_accel, ...
                'longitudinalAccel', groups.longitudinal_accel, ...
                'wheelSpeed', groups.wheel_speed);
        end

        function result = failedResult(status)
            result = struct('score', Inf, 'status', string(status), ...
                'sampleCount', 0, 'gpsTrace', NaN, 'speed', NaN, 'yawRate', NaN, ...
                'lateralAccel', NaN, 'longitudinalAccel', NaN, ...
                'wheelSpeed', NaN);
        end
    end

    methods (Static, Access = private)
        function loss = channelLoss(sim, reference, referenceTime, queryTime, ...
                commonMask, horizonWeight, scale)
            sim = sim(:);
            reference = reference(:);
            if isempty(sim) || numel(sim) ~= numel(queryTime) || ...
                    isempty(reference) || numel(reference) ~= numel(referenceTime)
                loss = NaN;
                return;
            end
            validReference = isfinite(referenceTime) & isfinite(reference);
            if nnz(validReference) < 2
                loss = NaN;
                return;
            end
            ref = interp1(referenceTime(validReference), reference(validReference), ...
                queryTime, 'linear', NaN);
            valid = commonMask & isfinite(sim) & isfinite(ref) & isfinite(horizonWeight);
            if nnz(valid) < 2
                loss = NaN;
                return;
            end
            residual = (sim(valid) - ref(valid)) / scale;
            absResidual = abs(residual);
            huber = 0.5 * residual.^2;
            outside = absResidual > 1;
            huber(outside) = absResidual(outside) - 0.5;
            weights = horizonWeight(valid);
            loss = sum(weights .* huber) / sum(weights);
            % Keep a finite but numerically runaway scalar channel (most
            % commonly wheel angular speed at a long horizon) from defeating
            % the fixed group weights. GPS trace loss is intentionally not
            % capped and therefore remains the authoritative long-horizon
            % discriminator.
            loss = min(loss, 50);
        end

        function value = meanFinite(values)
            values = values(isfinite(values));
            if isempty(values)
                value = NaN;
            else
                value = mean(values);
            end
        end

        function loss = traceLoss(simX, simY, referenceX, referenceY, ...
                referenceTime, queryTime, commonMask, horizonWeight, scale)
            simX = simX(:);
            simY = simY(:);
            referenceX = referenceX(:);
            referenceY = referenceY(:);
            if numel(simX) ~= numel(queryTime) || numel(simY) ~= numel(queryTime) || ...
                    numel(referenceX) ~= numel(referenceTime) || ...
                    numel(referenceY) ~= numel(referenceTime)
                loss = NaN;
                return;
            end
            referenceValid = isfinite(referenceTime) & ...
                isfinite(referenceX) & isfinite(referenceY);
            if nnz(referenceValid) < 3
                loss = NaN;
                return;
            end
            refX = interp1(referenceTime(referenceValid), referenceX(referenceValid), ...
                queryTime, 'linear', NaN);
            refY = interp1(referenceTime(referenceValid), referenceY(referenceValid), ...
                queryTime, 'linear', NaN);
            joint = isfinite(simX) & isfinite(simY) & isfinite(refX) & isfinite(refY);
            first = find(joint, 1, 'first');
            if isempty(first)
                loss = NaN;
                return;
            end
            simX = simX - simX(first);
            simY = simY - simY(first);
            refX = refX - refX(first);
            refY = refY - refY(first);
            headingWindow = joint & queryTime <= queryTime(first) + 0.5;
            last = find(headingWindow, 1, 'last');
            if isempty(last) || last == first || ...
                    hypot(simX(last), simY(last)) < 0.05 || ...
                    hypot(refX(last), refY(last)) < 0.05
                loss = NaN;
                return;
            end
            simHeading = atan2(simY(last), simX(last));
            refHeading = atan2(refY(last), refX(last));
            simForward = simX .* cos(simHeading) + simY .* sin(simHeading);
            simLeft = -simX .* sin(simHeading) + simY .* cos(simHeading);
            refForward = refX .* cos(refHeading) + refY .* sin(refHeading);
            refLeft = -refX .* sin(refHeading) + refY .* cos(refHeading);
            valid = commonMask & joint & isfinite(horizonWeight);
            if nnz(valid) < 2
                loss = NaN;
                return;
            end
            residual = hypot(simForward(valid) - refForward(valid), ...
                simLeft(valid) - refLeft(valid)) ./ scale;
            huber = 0.5 .* residual.^2;
            outside = residual > 1;
            huber(outside) = residual(outside) - 0.5;
            weights = horizonWeight(valid);
            loss = sum(weights .* huber) / sum(weights);
        end

        function value = field(s, name, defaultValue)
            if isstruct(s) && isfield(s, name)
                value = s.(name);
            else
                value = defaultValue;
            end
        end
    end
end
