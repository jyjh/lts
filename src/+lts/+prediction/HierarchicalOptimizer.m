classdef HierarchicalOptimizer
    %HIERARCHICALOPTIMIZER Reduced GGV plan with transient feasibility feedback.

    methods (Static)
        function result = optimize(config, track, varargin)
            parser = inputParser;
            parser.addParameter('Dt', 0.001, @(x) isnumeric(x) && isscalar(x) && x > 0);
            parser.addParameter('LineOffsetFractions', [0.45 0.65 0.80], @isnumeric);
            parser.addParameter('UsageSchedule', [0.98 0.93 0.88], @isnumeric);
            % A flying-lap seed avoids making the optimizer result depend on
            % the simulator's numerically delicate near-zero slip regime.
            parser.addParameter('InitialSpeed', 5.0, @(x) isnumeric(x) && isscalar(x));
            parser.parse(varargin{:});
            opts = parser.Results;

            candidates = struct('lapTime', {}, 'feasible', {}, ...
                'lineOffsetFraction', {}, 'usage', {}, ...
                'minimumTrackMargin', {}, 'stateLog', {}, 'status', {});
            bestIndex = NaN;
            bestTime = Inf;
            for usage = opts.UsageSchedule(:).'
                for offset = opts.LineOffsetFractions(:).'
                    candidate = lts.prediction.HierarchicalOptimizer.runCandidate( ...
                        config, track, opts.Dt, opts.InitialSpeed, usage, offset);
                    candidates(end + 1) = candidate; %#ok<AGROW>
                    if candidate.feasible && candidate.lapTime < bestTime
                        bestTime = candidate.lapTime;
                        bestIndex = numel(candidates);
                    end
                end
                if isfinite(bestIndex)
                    break;
                end
            end
            if isempty(candidates)
                error('lts_prediction_HierarchicalOptimizer:NoCandidates', ...
                    'Optimizer candidate schedules must not be empty.');
            elseif isfinite(bestIndex)
                winner = candidates(bestIndex);
            else
                finiteIndex = find(isfinite([candidates.lapTime]), 1, 'first');
                if isempty(finiteIndex)
                    winner = candidates(1);
                else
                    winner = candidates(finiteIndex);
                end
            end
            result = struct('schema', "lts.prediction.optimizer-result.v1", ...
                'lapTime', winner.lapTime, 'feasible', winner.feasible, ...
                'status', winner.status, ...
                'lineOffsetFraction', winner.lineOffsetFraction, ...
                'usage', winner.usage, ...
                'minimumTrackMargin', winner.minimumTrackMargin, ...
                'stateLog', winner.stateLog, 'candidates', candidates, ...
                'sectorTimes', lts.prediction.HierarchicalOptimizer.sectorTimes( ...
                    winner.stateLog, 3));
        end
    end

    methods (Static, Access = private)
        function candidate = runCandidate(config, track, dt, initialSpeed, usage, offset)
            config = config; %#ok<NASGU>
            stateLog = struct();
            lapTime = NaN;
            margin = NaN;
            try
                evalc('vehicle = lts.vehicle.VehicleManager.fromConfig(config, track, dt);');
                driver = lts.driver.DriverModel(vehicle);
                driver.corneringUsage = usage;
                driver.brakingUsage = usage;
                driver.driveUsage = usage;
                driver.racingLineOffsetFraction = offset;
                simulator = lts.simulation.Simulator(vehicle, driver, dt);
                simulator.verbose = false;
                points = track.getTrackPoints();
                headings = track.getHeading();
                initial = lts.simulation.VehicleState('s', 0, ... %#ok<NASGU>
                    'x', points(1, 1), 'y', points(1, 2), ...
                    'yaw', headings(1), 'speed', initialSpeed);
                evalc('[stateLog, lapTime] = simulator.simulate(initial, track);');
                margin = NaN;
                if isfield(stateLog, 'lateralError') && ~isempty(stateLog.lateralError)
                    margin = track.getTrackWidth() / 2 - max(abs(stateLog.lateralError));
                end
                finiteState = isfinite(lapTime) && isstruct(stateLog) && ...
                    isfield(stateLog, 'speed') && all(isfinite(stateLog.speed));
                feasible = finiteState && (isnan(margin) || margin >= -1e-6);
                if feasible
                    status = "feasible";
                elseif finiteState
                    status = "track_constraint";
                else
                    status = "incomplete";
                end
            catch err
                stateLog = struct();
                lapTime = NaN;
                margin = NaN;
                feasible = false;
                status = "error:" + string(err.identifier);
            end
            candidate = struct('lapTime', lapTime, 'feasible', feasible, ...
                'lineOffsetFraction', offset, 'usage', usage, ...
                'minimumTrackMargin', margin, 'stateLog', stateLog, ...
                'status', status);
        end

        function sectors = sectorTimes(stateLog, count)
            sectors = nan(1, count);
            if ~isstruct(stateLog) || ~isfield(stateLog, 'time') || ...
                    ~isfield(stateLog, 's') || numel(stateLog.time) < 2
                return;
            end
            s = double(stateLog.s(:));
            time = double(stateLog.time(:));
            valid = isfinite(s) & isfinite(time);
            s = s(valid);
            time = time(valid);
            [s, uniqueIndex] = unique(s, 'stable');
            time = time(uniqueIndex);
            if numel(s) < 2 || s(end) <= s(1)
                return;
            end
            boundaries = linspace(s(1), s(end), count + 1);
            sectors = diff(interp1(s, time, boundaries, 'linear'));
        end
    end
end
