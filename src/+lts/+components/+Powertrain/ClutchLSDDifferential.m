classdef ClutchLSDDifferential < lts.components.Powertrain.DifferentialComponent
    % CLUTCHLSDDIFFERENTIAL Clutch-pack (plate) limited-slip differential
    %
    % Behaves like an open differential at the carrier (carrier speed is the
    % mean of the two wheel speeds, so wheels may differentiate) but biases
    % torque toward the slower-rotating (typically the more heavily loaded,
    % outside) wheel via a clutch pack. The bias (locking) torque is:
    %
    %   T_lock = preload                 (static clutch preload)
    %          + ramp * T_total          (torque-sensitive ramp, 1:1 here)
    %          + speedGain*(w_fast-w_slow)  (speed-sensitive viscous term)
    %
    % The clutch torque is an internal equal-and-opposite axle torque: it
    % remains active off-throttle while TL + TR always equals the externally
    % applied driveline torque. Load-dependent bias is capped by biasRatio.
    % This is a simplified but representative model of the plate-style LSDs
    % (and Torsten-type units) commonly run on FSAE cars.
    %
    % Defaults are a mild, stable setup; tune preload/ramp/biasRatio to suit.

    properties
        % Static clutch pack preload torque [Nm]. It resists relative wheel
        % motion even at zero externally applied torque.
        preload = 20

        % Torque-sensitive ramp coefficient [dimensionless]. Fraction of the
        % total axle torque added as locking torque. 1.0 = 1:1 ramp.
        ramp = 0.5

        % Speed-sensitive (viscous) locking gain [N*m*s/rad]. Adds locking
        % torque proportional to the wheel speed difference so the clutch
        % actually resists relative wheelspin rather than only statically
        % re-biasing torque. A small nonzero default keeps the LSD from
        % limit-cycling; set 0 for a pure torque-sensitive plate LSD.
        speedGain = 2.0

        % Maximum torque bias ratio T_slow / T_fast [-]. A typical 1.5-way
        % clutch LSD runs ~1.5-3.0. inf disables the cap.
        biasRatio = 2.0

        % Maximum fraction of relative wheel speed corrected by one explicit
        % torque evaluation. Keeping this below one makes the simulator's
        % fixed-point wheel iterations contract instead of chatter.
        relativeSpeedDamping = 0.5
    end

    methods
        function obj = ClutchLSDDifferential(varargin)
            % CLUTCHLSDDIFFERENTIAL Optional name-value overrides for the
            %   tuning parameters: 'preload','ramp','speedGain','biasRatio'.
            if mod(nargin, 2) ~= 0
                error('ClutchLSDDifferential:BadArgs', ...
                    'Arguments must be name-value pairs.');
            end
            for i = 1:2:nargin
                if isprop(obj, varargin{i})
                    obj.(varargin{i}) = varargin{i + 1};
                end
            end
        end

        function out = solveDrive(obj, totalWheelTorque, omegaL, omegaR, wheelInertia, dt)
            out = obj.solveDriveline( ...
                max(0, totalWheelTorque), 0, omegaL, omegaR, wheelInertia, dt);
        end

        function out = solveDriveline(obj, driveWheelTorque, coastdownWheelTorque, ...
                omegaL, omegaR, wheelInertia, dt)
            driveWheelTorque = max(0, driveWheelTorque);
            totalWheelTorque = driveWheelTorque + coastdownWheelTorque;
            base = 0.5 * totalWheelTorque;
            dw = omegaR - omegaL;

            % Ramp and viscous capacity are load-dependent. During nonzero
            % drive/coast, cap that part at the configured torque-bias ratio.
            % Preload remains an independent clutch capacity, so it can act as
            % a zero-net internal torque during true off-throttle coast.
            dynamicCapacity = max(0, obj.ramp) * ...
                (abs(driveWheelTorque) + abs(coastdownWheelTorque)) + ...
                max(0, obj.speedGain) * abs(dw);
            if abs(totalWheelTorque) > eps && isfinite(obj.biasRatio)
                b = max(1, obj.biasRatio);
                biasRatioCapacity = 0.5 * abs(totalWheelTorque) * ...
                    (b - 1) / (b + 1);
                dynamicCapacity = min(dynamicCapacity, biasRatioCapacity);
            end
            clutchCapacity = max(0, obj.preload) + dynamicCapacity;

            biasMagnitude = obj.boundedBiasMagnitude( ...
                clutchCapacity, dw, wheelInertia, dt);
            biasTorque = sign(dw) * biasMagnitude;

            out.TL = base + biasTorque;
            out.TR = base - biasTorque;
            out.carrierOmega = 0.5 * (omegaL + omegaR);
            out.lockTorqueDifference = 2 * biasMagnitude;
        end

        function locked = locksWheels(~)
            locked = false;
        end

        function name = getName(~)
            name = 'ClutchLSDDifferential';
        end
    end

    methods (Access = private)
        function magnitude = boundedBiasMagnitude(obj, capacity, dw, wheelInertia, dt)
            if abs(dw) <= eps || capacity <= 0
                magnitude = 0;
                return;
            end

            magnitude = capacity;
            if isfinite(wheelInertia) && wheelInertia > 0 && ...
                    isfinite(dt) && dt > 0
                damping = obj.relativeSpeedDamping;
                if ~isfinite(damping)
                    damping = 0.5;
                end
                damping = lts.util.clamp(damping, 0, 0.95);
                % Equal-and-opposite bias B changes relative acceleration by
                % 2B/I. This impulse cap limits the one-evaluation correction
                % to damping*|dw| and keeps fixed-point iteration contractive.
                impulseCapacity = 0.5 * damping * wheelInertia * abs(dw) / dt;
                magnitude = min(magnitude, impulseCapacity);
            end
        end
    end
end
