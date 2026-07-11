classdef TelemetryReplayDriver < handle
    % TELEMETRYREPLAYDRIVER Supplies measured controls to lts.simulation.Simulator.
    %
    % This class intentionally does not add path-following feedback, input
    % slew, or pedal exclusivity. The lts.simulation.Simulator owns any remaining safety
    % clamping requested by the caller.
    %
    % In correlation mode the "driver" is the data log: throttle, brake,
    % pressure, and steer are interpolated by time or distance and handed to
    % the same physics step used for synthetic laps.

    properties
        profile
        replayDomain = "distance"  % "distance" or "time"
        inputDt = 0.001

        % Let the replay channel map define road-wheel steering scale.
        maxSteeringAngle = inf
        steeringRampTime = 0
        replayDomainIsTime = false
        replayDurationDenom = 1
        replayDistanceDenom = 1
    end

    methods
        function obj = TelemetryReplayDriver(profile, varargin)
            parser = inputParser;
            parser.addParameter('ReplayDomain', 'distance', @(x) ischar(x) || isstring(x));
            parser.parse(varargin{:});

            if isa(profile, 'lts.correlation.CorrelationReplayProfile')
                obj.profile = profile;
            else
                obj.profile = lts.correlation.CorrelationReplayProfile.fromCsv(profile);
            end
            obj.replayDomain = lower(string(parser.Results.ReplayDomain));
            obj.validateReplayDomain();
            obj.replayDomainIsTime = obj.replayDomain == "time";
            obj.replayDurationDenom = max(obj.profile.duration(), eps);
            obj.replayDistanceDenom = max(obj.profile.totalDistance(), eps);
        end

        function obj = prepareForSimulation(obj, ~, ~, dt)
            if nargin >= 4 && isfinite(dt) && dt > 0
                obj.inputDt = dt;
            end
        end

        function input = computeInput(obj, state, observation)
            if obj.replayDomainIsTime
                input = obj.profile.sampleByTime(state.time);
                input.replayDomain = 'time';
                input.replayProgress = lts.util.saturate(...
                    input.sourceTime / obj.replayDurationDenom);
            else
                s = state.s;
                if nargin >= 3 && isstruct(observation) && ...
                        isfield(observation, 's') && isfinite(observation.s)
                    s = observation.s;
                end
                input = obj.profile.sampleByDistance(s);
                input.replayDomain = 'distance';
                input.replayProgress = lts.util.saturate(...
                    input.sourceDistance / obj.replayDistanceDenom);
            end
        end
    end

    methods (Access = private)
        function validateReplayDomain(obj)
            valid = obj.replayDomain == "distance" || obj.replayDomain == "time";
            if ~valid
                error('lts_correlation_TelemetryReplayDriver:InvalidReplayDomain', ...
                    'ReplayDomain must be "distance" or "time".');
            end
        end

    end
end
