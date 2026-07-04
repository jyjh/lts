classdef CorrelationStateInitializer
    % CORRELATIONSTATEINITIALIZER Builds lts.simulation.VehicleState from imported telemetry.
    %
    % Replays should start from measured speed/yaw/position when available;
    % otherwise they fall back to a neutral free-space state. This keeps
    % correlation runs from inheriting synthetic track-start assumptions.

    methods (Static)
        function state = fromReplayProfile(profile, track, vehicleManager, varargin)
            parser = inputParser;
            parser.addParameter('UseLoggedPosition', true, @(x) islogical(x) || isnumeric(x));
            parser.addParameter('UseLoggedYawRate', true, @(x) islogical(x) || isnumeric(x));
            parser.addParameter('UseLoggedWheelSpeeds', true, @(x) islogical(x) || isnumeric(x));
            parser.parse(varargin{:});

            if ~isa(profile, 'lts.correlation.CorrelationReplayProfile')
                profile = lts.correlation.CorrelationReplayProfile.fromCsv(profile);
            end

            speed = profile.speed(1);
            throttle = profile.throttle(1);
            brake = profile.brake(1);
            steer = profile.steer(1);

            [vx, vy, speed] = lts.correlation.CorrelationStateInitializer.initialVelocity(profile, speed);

            yaw = 0;
            if profile.hasYaw()
                yaw = profile.yaw(1);
            end

            x = 0;
            y = 0;
            if profile.hasPosition() && logical(parser.Results.UseLoggedPosition)
                x = profile.x(1);
                y = profile.y(1);
            end

            state = lts.simulation.VehicleState( ...
                's', 0, ...
                'speed', speed, ...
                'vx', vx, ...
                'vy', vy, ...
                'x', x, ...
                'y', y, ...
                'yaw', yaw, ...
                'throttle', throttle, ...
                'brake', brake, ...
                'steer', steer);

            if logical(parser.Results.UseLoggedYawRate) && ...
                    ~isempty(profile.yawRate) && isfinite(profile.yawRate(1))
                state.yawRate = profile.yawRate(1);
            end

            if nargin >= 3 && ~isempty(vehicleManager)
                state.vehicleManager = vehicleManager;
                if logical(parser.Results.UseLoggedWheelSpeeds)
                    lts.correlation.CorrelationStateInitializer.seedWheelSpeeds( ...
                        profile, vehicleManager, speed);
                end
            end
        end
    end

    methods (Static, Access = private)
        function [vx, vy, speed] = initialVelocity(profile, speed)
            speed = max(0, speed);
            if profile.hasVelocity()
                vx = profile.vx(1);
                vy = profile.vy(1);
                speed = hypot(vx, vy);
                return;
            end

            if profile.hasBodySlip()
                beta = profile.bodySlip(1);
                vx = speed * cos(beta);
                vy = speed * sin(beta);
                return;
            end

            vx = speed;
            vy = 0;
        end

        function seedWheelSpeeds(profile, vehicleManager, vehicleSpeed)
            if ~profile.hasInitialWheelSpeeds() || isempty(vehicleManager) || ...
                    isempty(vehicleManager.tire)
                return;
            end

            wheelSpeeds = profile.initialWheelSpeeds();
            corners = {'FL', 'FR', 'RL', 'RR'};
            values = [wheelSpeeds.FL, wheelSpeeds.FR, ...
                wheelSpeeds.RL, wheelSpeeds.RR];
            valid = isfinite(values) & values >= 0;
            fallbackSpeed = max(vehicleSpeed, 0);
            if any(valid)
                fallbackSpeed = median(values(valid));
            end

            for i = 1:numel(corners)
                cornerName = corners{i};
                wheelSpeed = values(i);
                if ~isfinite(wheelSpeed) || wheelSpeed < 0
                    wheelSpeed = fallbackSpeed;
                end

                cornerState = vehicleManager.tire.(cornerName);
                cornerState.angularVelocity = max(wheelSpeed, 0) / ...
                    max(cornerState.wheelRadius, eps);
            end

            if ~isempty(vehicleManager.powertrain) && ...
                    ismethod(vehicleManager.powertrain, 'updateStateFromDrivenWheels')
                vehicleManager.powertrain.updateStateFromDrivenWheels( ...
                    [vehicleManager.tire.RL.angularVelocity, ...
                    vehicleManager.tire.RR.angularVelocity]);
            end
        end
    end
end
