classdef CorrelationStateInitializer
    % CORRELATIONSTATEINITIALIZER Builds VehicleState from imported telemetry.

    methods (Static)
        function state = fromReplayProfile(profile, track, vehicleManager, varargin)
            parser = inputParser;
            parser.addParameter('UseLoggedPosition', true, @(x) islogical(x) || isnumeric(x));
            parser.addParameter('UseLoggedYawRate', true, @(x) islogical(x) || isnumeric(x));
            parser.parse(varargin{:});

            if ~isa(profile, 'CorrelationReplayProfile')
                profile = CorrelationReplayProfile.fromCsv(profile);
            end

            speed = profile.speed(1);
            throttle = profile.throttle(1);
            brake = profile.brake(1);
            steer = profile.steer(1);

            yaw = NaN;
            heading = NaN;
            if profile.hasYaw()
                yaw = profile.yaw(1);
                heading = yaw;
            elseif nargin >= 2 && ~isempty(track)
                trackHeading = track.getHeading();
                if ~isempty(trackHeading)
                    yaw = trackHeading(1);
                    heading = yaw;
                end
            end

            state = VehicleState( ...
                's', 0, ...
                'speed', speed, ...
                'vx', speed, ...
                'vy', 0, ...
                'yaw', yaw, ...
                'heading', heading, ...
                'throttle', throttle, ...
                'brake', brake, ...
                'steer', steer);

            if profile.hasPosition() && logical(parser.Results.UseLoggedPosition)
                state.x = profile.x(1);
                state.y = profile.y(1);
            end

            if logical(parser.Results.UseLoggedYawRate) && ...
                    ~isempty(profile.yawRate) && isfinite(profile.yawRate(1))
                state.yawRate = profile.yawRate(1);
            end

            if nargin >= 3 && ~isempty(vehicleManager)
                state.vehicleManager = vehicleManager;
            end
        end
    end
end
