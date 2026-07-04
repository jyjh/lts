classdef TelemetryWindow
    % TELEMETRYWINDOW Trims simulated telemetry to the recorded lap window.

    methods (Static)
        function [stateLog, lapTime, recordedSteps] = apply(stateLog, recordStartS, recordEndS)
            if isempty(stateLog.time)
                lapTime = 0;
                recordedSteps = 0;
                return;
            end

            trimDiagnostics = lts.simulation.TelemetryWindow.trimDiagnostics(stateLog);
            keep = stateLog.s >= recordStartS - 1e-9 & ...
                stateLog.s <= recordEndS + 1e-9;
            fields = fieldnames(stateLog);
            for i = 1:numel(fields)
                stateLog.(fields{i}) = stateLog.(fields{i})(keep);
            end

            recordedSteps = nnz(keep);
            if recordedSteps == 0
                warning('lts_simulation_Simulator:NoRecordedTelemetry', ...
                    ['No telemetry samples fell inside the recorded lap window ' ...
                    '(%.1f m to %.1f m). Simulation ended before the timed lap ' ...
                    'started or completed. Max simulated s was %.1f m, final ' ...
                    'speed was %.1f km/h, final lateral error was %.3f m, ' ...
                    'and minimum track margin was %.3f m.'], ...
                    recordStartS, recordEndS, ...
                    trimDiagnostics.maxS, ...
                    trimDiagnostics.finalSpeedKmh, ...
                    trimDiagnostics.finalLateralError, ...
                    trimDiagnostics.minTrackMargin);
                lapTime = 0;
                return;
            end

            if recordStartS > 0
                stateLog.time = stateLog.time - stateLog.time(1);
                if isfield(stateLog, 'controlTime')
                    stateLog.controlTime = stateLog.controlTime - stateLog.controlTime(1);
                end

                distanceFields = {'s', 'controlS', 'refS'};
                for i = 1:numel(distanceFields)
                    field = distanceFields{i};
                    if isfield(stateLog, field)
                        stateLog.(field) = max(0, stateLog.(field) - recordStartS);
                    end
                end
            end

            lapTime = stateLog.time(end);
        end

        function diagnostics = trimDiagnostics(stateLog)
            diagnostics.maxS = NaN;
            diagnostics.finalSpeedKmh = NaN;
            diagnostics.finalLateralError = NaN;
            diagnostics.minTrackMargin = NaN;

            if isfield(stateLog, 's') && ~isempty(stateLog.s)
                diagnostics.maxS = max(stateLog.s);
            end
            if isfield(stateLog, 'speedKmh') && ~isempty(stateLog.speedKmh)
                diagnostics.finalSpeedKmh = stateLog.speedKmh(end);
            elseif isfield(stateLog, 'speed') && ~isempty(stateLog.speed)
                diagnostics.finalSpeedKmh = stateLog.speed(end) * 3.6;
            end
            if isfield(stateLog, 'lateralError') && ~isempty(stateLog.lateralError)
                diagnostics.finalLateralError = stateLog.lateralError(end);
            end
            if isfield(stateLog, 'trackLimitMargin') && ...
                    ~isempty(stateLog.trackLimitMargin)
                diagnostics.minTrackMargin = min(stateLog.trackLimitMargin);
            end
        end
    end
end
