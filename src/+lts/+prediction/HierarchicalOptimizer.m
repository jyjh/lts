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
            % Parallel=false (default) preserves the tier-by-tier early-exit:
            % once any candidate in the highest-remaining usage tier is
            % feasible, lower tiers are skipped. Set Parallel=true to evaluate
            % the full UsageSchedule x LineOffsetFractions grid with parfor;
            % this drops the early-exit but parallelizes the (independent)
            % full-lap simulations. Each candidate is a fresh vehicle + sim, so
            % they are embarrassingly parallel.
            parser.addParameter('Parallel', false, ...
                @(x) islogical(x) || (isnumeric(x) && isscalar(x)));
            parser.parse(varargin{:});
            opts = parser.Results;

            candidates = struct('lapTime', {}, 'feasible', {}, ...
                'lineOffsetFraction', {}, 'usage', {}, ...
                'minimumTrackMargin', {}, 'stateLog', {}, 'status', {});
            bestIndex = NaN;
            bestTime = Inf;
            if logical(opts.Parallel)
                candidates = lts.prediction.HierarchicalOptimizer. ...
                    runCandidatesParallel(config, track, opts);
                bestIndex = lts.prediction.HierarchicalOptimizer. ...
                    pickBestCandidate(candidates);
            else
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
                vehicle = lts.vehicle.VehicleManager.fromConfig( ...
                    config, track, dt, 'Verbose', false);
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
                [stateLog, lapTime] = simulator.simulate(initial, track);
                % Feasibility margin = smallest per-step distance to a track
                % edge over the lap. stateLog.trackLimitMargin is already the
                % per-side margin (left or right half-width minus the lateral
                % error, whichever side the car is on), so its min is the
                % tightest excursion. Fall back to a symmetric estimate from
                % the representative track width if the channel is absent.
                margin = NaN;
                if isfield(stateLog, 'trackLimitMargin') && ...
                        ~isempty(stateLog.trackLimitMargin) && ...
                        any(isfinite(stateLog.trackLimitMargin))
                    margin = min(stateLog.trackLimitMargin(isfinite(stateLog.trackLimitMargin)));
                elseif isfield(stateLog, 'lateralError') && ~isempty(stateLog.lateralError)
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
                % Surface the message + stack frame so a candidate failure is
                % diagnosable instead of just an opaque identifier.
                stackFrame = "";
                if ~isempty(err.stack)
                    stackFrame = " @ " + string(err.stack(1).name);
                end
                status = "error:" + string(err.identifier) + stackFrame;
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

        function candidates = runCandidatesParallel(config, track, opts)
            % RUNCANDIDATESPARALLEL Evaluate the full grid with parfor.
            %   Builds the UsageSchedule x LineOffsetFractions candidate grid
            %   as a preallocated struct array and fills it in any order
            %   (each candidate is an independent full-lap sim). Returns the
            %   candidates in usage-major order to match the serial path.
            usages = opts.UsageSchedule(:);
            offsets = opts.LineOffsetFractions(:);
            nU = numel(usages);
            nO = numel(offsets);
            nC = nU * nO;
            template = struct('lapTime', NaN, 'feasible', false, ...
                'lineOffsetFraction', 0.0, 'usage', 0.0, ...
                'minimumTrackMargin', NaN, 'stateLog', struct(), ...
                'status', "incomplete");
            candidates = repmat(template, nC, 1);
            % Broadcast the full (usage,offset) list so each parfor iteration
            % is a single flat, sliced assignment (MATLAB requires the sliced
            % index to appear directly in the parfor body, not a nested loop).
            % Ordering is usage-major: all offsets for usage 1, then usage 2,
            % ... matching the serial path's nested for-loop.
            usageList = repmat(usages, nO, 1);            % [u1..u1, u2..u2, ...]
            offsetList = repmat(offsets, nU, 1);           % [o1..o_nO, o1..o_nO, ...]
            parfor k = 1:nC
                candidates(k) = lts.prediction.HierarchicalOptimizer.runCandidate( ...
                    config, track, opts.Dt, opts.InitialSpeed, ...
                    usageList(k), offsetList(k));
            end
        end

        function bestIndex = pickBestCandidate(candidates)
            % PICKBESTCANDIDATE Index of the fastest feasible candidate, or
            %   NaN if none is feasible. Ties broken by first occurrence to
            %   match the serial path's "> bestTime" strict comparison.
            bestIndex = NaN;
            bestTime = Inf;
            for k = 1:numel(candidates)
                if candidates(k).feasible && candidates(k).lapTime < bestTime
                    bestTime = candidates(k).lapTime;
                    bestIndex = k;
                end
            end
        end
    end
end
