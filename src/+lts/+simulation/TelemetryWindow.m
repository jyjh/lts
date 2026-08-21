classdef TelemetryWindow
    methods (Static)
        function [stateLog, lapTime, recordedSteps] = apply( ...
                stateLog, recordStartS, recordEndS)
            lapTime = NaN;
            recordedSteps = 0;
            if isempty(stateLog.time)
                return;
            end

            tolerance = max(1e-6, 1e-9 * max(abs(recordEndS), 1));
            maxS = max(stateLog.s);
            completed = maxS >= recordEndS - tolerance;
            minMargin = NaN;
            if isfield(stateLog, 'trackLimitMargin') && ~isempty(stateLog.trackLimitMargin)
                minMargin = min(stateLog.trackLimitMargin);
            end
            keep = stateLog.s >= recordStartS - 1e-9 & ...
                stateLog.s <= recordEndS + 1e-9;
            fields = fieldnames(stateLog);
            for i = 1:numel(fields)
                stateLog.(fields{i}) = stateLog.(fields{i})(keep);
            end

            recordedSteps = nnz(keep);
            if recordedSteps == 0
                warning('lts_simulation_Simulator:NoRecordedTelemetry', ...
                    ['No samples in the recorded window. Max simulated s was %.1f m; ' ...
                    'minimum track margin was %.3f m.'], maxS, minMargin);
                return;
            end

            if recordStartS > 0
                stateLog.time = stateLog.time - stateLog.time(1);
                if isfield(stateLog, 'controlTime')
                    stateLog.controlTime = stateLog.controlTime - stateLog.controlTime(1);
                end
                for field = {'s', 'controlS', 'refS'}
                    name = field{1};
                    if isfield(stateLog, name)
                        stateLog.(name) = max(0, stateLog.(name) - recordStartS);
                    end
                end
            end

            if completed
                lapTime = stateLog.time(end);
            else
                warning('lts_simulation_Simulator:IncompleteRecordedTelemetry', ...
                    'The recorded window is incomplete, so no lap time is reported.');
            end
        end
    end
end
