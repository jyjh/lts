classdef (Abstract) DifferentialComponent
    % DIFFERENTIALCOMPONENT Abstract interface for driven-axle differentials
    %
    % A differential maps a total commanded driven-axle wheel torque and the
    % two driven wheels' angular velocities into a per-wheel torque split and
    % a single carrier (output-shaft) angular velocity that the powertrain
    % samples for motor speed.
    %
    % Concrete implementations:
    %   - OpenDifferential         50/50 torque split, carrier = mean(omega)
    %   - LockedDifferential       spool: equal rear wheel speed
    %   - ClutchLSDDifferential    torque-biasing limited-slip
    %
    % The solve is called once per wheel-contact iteration in Simulator.step,
    % so the returned carrier speed stays consistent with the converged wheel
    % speeds (and therefore the motor RPM / rev limiter).

    methods (Abstract)
        % out = solveDrive(totalWheelTorque, omegaL, omegaR, wheelInertia, dt)
        %
        %   totalWheelTorque - Total torque commanded to the axle [Nm] (>=0 drive)
        %   omegaL, omegaR   - Left/right driven-wheel angular velocity [rad/s]
        %   wheelInertia     - Per-wheel rotational inertia [kg*m^2]
        %   dt               - Timestep [s]
        %
        % Returns a struct with:
        %   .TL, .TR        - Per-wheel drive torque [Nm] (>=0)
        %   .carrierOmega   - Differential carrier angular velocity [rad/s]
        %                     (the speed the powertrain samples for motor RPM)
        out = solveDrive(obj, totalWheelTorque, omegaL, omegaR, wheelInertia, dt)

        % True if the differential mechanically locks the driven wheels to a
        % common speed (e.g. a spool). When true, Simulator assigns
        % out.carrierOmega to both driven wheels after each wheel update.
        locked = locksWheels(obj)
    end

    methods
        function name = getName(obj)
            name = 'DifferentialComponent';
        end
    end
end
