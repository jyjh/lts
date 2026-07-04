classdef LockedDifferential < lts.components.Powertrain.DifferentialComponent
    % LOCKEDDIFFERENTIAL Locked differential / spool
    %
    % The two driven wheels are mechanically locked together and rotate at a
    % single common speed. Torque is split 50/50 between them (load-proportional
    % split is not modeled for a spool; in reality the contact patches
    % self-distribute, but a 50/50 split is the standard simplification).
    %
    % The speed constraint is enforced by lts.simulation.Simulator, which assigns
    % out.carrierOmega (the mean of the incoming wheel speeds) to both driven
    % wheels after the per-corner wheel solver has run. The spool does NOT
    % forward-integrate the combined-rotor speed itself: doing so after the
    % per-corner solver already advanced each wheel with T_drive + tire Fx +
    % brake would double-count the drive impulse and discard tire/brake
    % resistance. Letting the per-corner solver own all dynamics keeps the
    % torque/impulse accounting identical to an open diff, differing only in
    % the speed-lock overwrite.
    %
    % A spool prevents inside-wheel wheelspin in slow corners (common on FSAE
    % cars that run a solid rear), at the cost of tire scrub and push in
    % tight corners.

    methods
        function obj = LockedDifferential()
        end

        function out = solveDrive(obj, totalWheelTorque, omegaL, omegaR, ~, ~)
            totalWheelTorque = max(0, totalWheelTorque);
            % Carrier speed is the mean of the incoming wheel speeds; the
            % speed-lock overwrite happens in lts.simulation.Simulator after this call. The
            % per-corner wheel solver handles all drive/brake/tire dynamics.
            out.TL = 0.5 * totalWheelTorque;
            out.TR = 0.5 * totalWheelTorque;
            out.carrierOmega = 0.5 * (omegaL + omegaR);
        end

        function locked = locksWheels(~)
            locked = true;
        end

        function name = getName(~)
            name = 'LockedDifferential';
        end
    end
end
