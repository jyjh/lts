classdef TelemetryReplayDriver < handle
    % TELEMETRYREPLAYDRIVER Supplies measured controls to Simulator.
    %
    % This class intentionally does not add path-following feedback, input
    % slew, or pedal exclusivity. The Simulator owns any remaining safety
    % clamping requested by the caller.

    properties
        profile
        replayDomain = "distance"  % "distance" or "time"
        inputDt = 0.001

        % Let the replay channel map define road-wheel steering scale.
        maxSteeringAngle = inf
        steeringRampTime = 0
    end

    methods
        function obj = TelemetryReplayDriver(profile, varargin)
            parser = inputParser;
            parser.addParameter('ReplayDomain', 'distance', @(x) ischar(x) || isstring(x));
            parser.parse(varargin{:});

            if isa(profile, 'CorrelationReplayProfile')
                obj.profile = profile;
            else
                obj.profile = CorrelationReplayProfile.fromCsv(profile);
            end
            obj.replayDomain = lower(string(parser.Results.ReplayDomain));
            obj.validateReplayDomain();
        end

        function obj = prepareForSimulation(obj, ~, ~, dt)
            if nargin >= 4 && isfinite(dt) && dt > 0
                obj.inputDt = dt;
            end
        end

        function input = computeInput(obj, state, observation)
            switch obj.replayDomain
                case "distance"
                    s = state.s;
                    if nargin >= 3 && isstruct(observation) && ...
                            isfield(observation, 's') && isfinite(observation.s)
                        s = observation.s;
                    end
                    input = obj.profile.sampleByDistance(s);
                    input = obj.addReplayProgress(input, "distance");
                case "time"
                    input = obj.profile.sampleByTime(state.time);
                    input = obj.addReplayProgress(input, "time");
                otherwise
                    error('TelemetryReplayDriver:InvalidReplayDomain', ...
                        'ReplayDomain must be "distance" or "time".');
            end
        end
    end

    methods (Access = private)
        function validateReplayDomain(obj)
            valid = obj.replayDomain == "distance" || obj.replayDomain == "time";
            if ~valid
                error('TelemetryReplayDriver:InvalidReplayDomain', ...
                    'ReplayDomain must be "distance" or "time".');
            end
        end

        function input = addReplayProgress(obj, input, domain)
            input.replayDomain = char(domain);
            switch domain
                case "time"
                    denom = max(obj.profile.duration(), eps);
                    numerator = input.sourceTime;
                case "distance"
                    denom = max(obj.profile.totalDistance(), eps);
                    numerator = input.sourceDistance;
                otherwise
                    denom = 1;
                    numerator = 0;
            end
            input.replayProgress = max(0, min(1, numerator / denom));
        end
    end
end
