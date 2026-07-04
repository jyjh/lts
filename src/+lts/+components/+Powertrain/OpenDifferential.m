classdef OpenDifferential < lts.components.Powertrain.DifferentialComponent
    % OPENDIFFERENTIAL Open (free) differential
    %
    % Torque is split 50/50 between the two driven wheels and the carrier
    % speed is the mean of the two wheel speeds, so the wheels may rotate at
    % different speeds. This is the differential model that reproduces the
    % simulator's original driveline behavior (mean-speed carrier + equal
    % torque split), and is the default.
    %
    % In a corner the inside (unloaded) wheel can spin up freely because it
    % receives the same torque as the loaded wheel but has less grip — the
    % classic open-differential wheelspin limitation.

    methods
        function obj = OpenDifferential()
        end

        function out = solveDrive(~, totalWheelTorque, omegaL, omegaR, ~, ~)
            totalWheelTorque = max(0, totalWheelTorque);
            out.TL = 0.5 * totalWheelTorque;
            out.TR = 0.5 * totalWheelTorque;
            % Carrier speed is the mean; allow speed differentiation.
            out.carrierOmega = 0.5 * (omegaL + omegaR);
        end

        function locked = locksWheels(~)
            locked = false;
        end

        function name = getName(~)
            name = 'OpenDifferential';
        end
    end
end
