classdef LockedDifferential < components.Powertrain.DifferentialComponent
    % LOCKEDDIFFERENTIAL Locked differential / spool
    %
    % The two driven wheels are mechanically locked together and rotate at a
    % single common speed. The combined wheel inertia (2*I) is accelerated by
    % the total axle torque, and the resulting speed is assigned to both
    % wheels. Torque is split 50/50 (load-proportional split is not modeled
    % for a spool; in reality the contact patches self-distribute, but a
    % 50/50 split is the standard simplification).
    %
    % A spool prevents inside-wheel wheelspin in slow corners (common on FSAE
    % cars that run a solid rear), at the cost of tire scrub and push in
    % tight corners.

    properties
        % If true, integrate the combined-rotor speed forward by dt instead
        % of simply averaging the incoming wheel speeds. Forward integration
        % keeps the locked speed consistent with the applied torque when the
        % wheels are accelerating (e.g. launch). Default true.
        integrateSpeed = true
    end

    methods
        function obj = LockedDifferential()
        end

        function out = solveDrive(obj, totalWheelTorque, omegaL, omegaR, wheelInertia, dt)
            totalWheelTorque = max(0, totalWheelTorque);
            omegaL = max(omegaL, 0);
            omegaR = max(omegaR, 0);
            I = max(wheelInertia, eps);

            if obj.integrateSpeed && dt > 0
                % Treat the two wheels as one combined rotor of inertia 2*I
                % driven by the total axle torque, starting from the mean.
                omegaMean = 0.5 * (omegaL + omegaR);
                omegaLocked = omegaMean + (totalWheelTorque / (2 * I)) * dt;
                omegaLocked = max(omegaLocked, 0);
            else
                omegaLocked = 0.5 * (omegaL + omegaR);
            end

            out.TL = 0.5 * totalWheelTorque;
            out.TR = 0.5 * totalWheelTorque;
            out.carrierOmega = omegaLocked;
        end

        function locked = locksWheels(~)
            locked = true;
        end

        function name = getName(~)
            name = 'LockedDifferential';
        end
    end
end
