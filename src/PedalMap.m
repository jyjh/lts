classdef PedalMap
    % PEDALMAP Stateless physics-based longitudinal pedal map.
    %
    % Maps a required longitudinal acceleration onto a [throttle, brake] pedal
    % pair in [0, 1], mutually exclusive, covering the four regimes a real
    % driver uses:
    %
    %   - WOT when the required drive force meets/exceeds full-throttle,
    %   - partial throttle to maintain speed against drag/rolling (cruise),
    %   - coast (both pedals zero) when drag alone provides the decel,
    %   - gradual brake proportional to the decel beyond coast.
    %
    % This is a pure, stateless utility extracted from the original driver
    % planner so both the lap planner and the live driver share one map. It
    % carries no VehicleManager and no driver state; callers supply the
    % capability numbers evaluated at the relevant speed.

    methods (Static)
        function [throttle, brake] = compute(axRef, F_drive_full, F_resistance, mass, brakeForceAccel)
            % COMPUTE Map a required longitudinal accel onto pedal commands.
            %
            %   [throttle, brake] = PedalMap.compute( ...
            %       axRef, F_drive_full, F_resistance, mass, brakeForceAccel)
            %
            % Inputs:
            %   axRef           - required longitudinal accel [m/s^2] (+ = drive)
            %   F_drive_full    - full-throttle wheel force, traction-capped [N]
            %   F_resistance    - drag + rolling resistance at this speed [N]
            %   mass            - vehicle mass [kg]
            %   brakeForceAccel - decel per unit brake command [m/s^2]
            %
            % Pedals are mutually exclusive (never both > 0), clamped to [0,1].
            mass = max(mass, eps);
            brakeForceAccel = max(brakeForceAccel, eps);

            % Force the wheels must apply at the contact patch to net axRef,
            % i.e. invert  ax = (F_drive - F_resistance) / mass.
            F_req = axRef * mass + F_resistance;

            % Coast deadband: a real driver lifts rather than holding a few
            % percent throttle or dabbing a few percent brake. When the required
            % force magnitude is below this fraction of the drive/brake scale,
            % snap to coast. The threshold is small enough (a few %) that it
            % only suppresses negligible pedal commands, never genuine cruise
            % throttle (which on this car is ~10%+ to overcome drag).
            coastFraction = 0.03;

            throttle = 0;
            brake = 0;
            if F_req <= 0
                % No drive force needed: the car must hold speed or slow down.
                requiredDecel = max(0, -axRef);
                coastDecel = F_resistance / mass;
                brakeForceTotal = brakeForceAccel * mass;
                if requiredDecel <= coastDecel
                    % Drag/rolling resistance alone covers the decel -> coast.
                    throttle = 0;
                    brake = 0;
                elseif brakeForceTotal > 0 && ...
                        (requiredDecel - coastDecel) < coastFraction * brakeForceAccel
                    % Required brake is negligible -> coast.
                    throttle = 0;
                    brake = 0;
                else
                    % Hydraulic brake fills the gap beyond coast, gradually.
                    brake = (requiredDecel - coastDecel) / brakeForceAccel;
                    brake = max(0, min(1, brake));
                end
            elseif F_drive_full <= 0
                % No tractive capability recorded (e.g. at/over rev limit) but
                % drive was requested: ask for WOT and let the powertrain
                % return whatever it can.
                throttle = 1;
            else
                throttle = F_req / F_drive_full;
                throttle = max(0, min(1, throttle));
                % Negligible throttle (below a few % of full) -> coast.
                if throttle < coastFraction
                    throttle = 0;
                end
            end
        end
    end
end
