classdef ClutchLSDDifferential < components.Powertrain.DifferentialComponent
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
    % capped by a maximum torque bias ratio (T_slow / T_fast <= biasRatio).
    % This is a simplified but representative model of the plate-style LSDs
    % (and Torsten-type units) commonly run on FSAE cars.
    %
    % Defaults are a mild, stable setup; tune preload/ramp/biasRatio to suit.

    properties
        % Static clutch pack preload torque [Nm]. Always biases toward the
        % slower wheel even at zero applied torque.
        preload = 20

        % Torque-sensitive ramp coefficient [dimensionless]. Fraction of the
        % total axle torque added as locking torque. 1.0 = 1:1 ramp.
        ramp = 0.5

        % Speed-sensitive (viscous) locking gain [N*m*s/rad]. Adds locking
        % torque proportional to the wheel speed difference.
        speedGain = 0.0

        % Maximum torque bias ratio T_slow / T_fast [-]. A typical 1.5-way
        % clutch LSD runs ~1.5-3.0. inf disables the cap.
        biasRatio = 2.0
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

        function out = solveDrive(obj, totalWheelTorque, omegaL, omegaR, ~, ~)
            totalWheelTorque = max(0, totalWheelTorque);
            omegaL = max(omegaL, 0);
            omegaR = max(omegaR, 0);

            base = 0.5 * totalWheelTorque;

            % Identify slower wheel (receives extra torque) and compute the
            % raw locking torque from preload + ramp + speed terms.
            if omegaL <= omegaR
                slowerSide = 'L';
                dw = omegaR - omegaL;
            else
                slowerSide = 'R';
                dw = omegaL - omegaR;
            end

            Tlock = obj.preload + obj.ramp * totalWheelTorque + obj.speedGain * dw;
            Tlock = max(0, Tlock);

            TL = base;
            TR = base;
            if slowerSide == 'L'
                TL = TL + Tlock;
                TR = TR - Tlock;
            else
                TR = TR + Tlock;
                TL = TL - Tlock;
            end

            % Enforce the maximum bias ratio on the conserved (un-clamped)
            % torques, THEN clamp to non-negative. Applying the cap before the
            % clamp keeps TL + TR == totalWheelTorque: the cap pulls the
            % over-biased (fast) side up toward maxSide/biasRatio and takes
            % the excess from the slow side, preserving the sum. Clamping
            % first would zero a negative fast side and create torque.
            [TL, TR] = obj.applyBiasRatio(TL, TR);
            TL = max(0, TL);
            TR = max(0, TR);

            out.TL = TL;
            out.TR = TR;
            out.carrierOmega = 0.5 * (omegaL + omegaR);
        end

        function locked = locksWheels(~)
            locked = false;
        end

        function name = getName(~)
            name = 'ClutchLSDDifferential';
        end
    end

    methods (Access = private)
        function [TL, TR] = applyBiasRatio(obj, TL, TR)
            % Cap |T_slow/T_fast| at biasRatio without changing the total.
            if ~isfinite(obj.biasRatio) || obj.biasRatio <= 0
                return;
            end
            total = TL + TR;
            if total <= eps
                return;
            end
            maxSide = max(TL, TR);
            minSide = min(TL, TR);
            cappedMin = maxSide / obj.biasRatio;
            if minSide < cappedMin
                % Rescale the lesser side up to the cap and pull the excess
                % from the greater side so the total is preserved.
                minSide = cappedMin;
                if TL <= TR
                    TL = minSide;
                    TR = total - TL;
                else
                    TR = minSide;
                    TL = total - TR;
                end
            end
        end
    end
end
