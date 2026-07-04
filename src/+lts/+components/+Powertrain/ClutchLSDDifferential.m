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
        % torque proportional to the wheel speed difference so the clutch
        % actually resists relative wheelspin rather than only statically
        % re-biasing torque. A small nonzero default keeps the LSD from
        % limit-cycling; set 0 for a pure torque-sensitive plate LSD.
        speedGain = 2.0

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
            % omegaL/omegaR may be negative when reverse rotation is enabled;
            % the carrier mean and slower-wheel identification both work with
            % signed speeds, so no clamp here.

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
            % Cap the locking torque so it can never exceed the open-diff base
            % torque. Without this, a high preload relative to a low commanded
            % total torque (e.g. preload=20 at T_total=10) drives the fast-side
            % torque negative, which inverts the bias and breaks TL+TR ==
            % T_total after the non-negative clamp. Capping at base - eps keeps
            % both sides strictly non-negative before any further processing.
            Tlock = min(Tlock, max(base - eps, 0));

            TL = base;
            TR = base;
            if slowerSide == 'L'
                TL = TL + Tlock;
                TR = TR - Tlock;
            else
                TR = TR + Tlock;
                TL = TL - Tlock;
            end

            % Enforce the maximum bias ratio on the now-guaranteed-non-negative
            % torques. applyBiasRatio rescales the lesser (fast) side up to
            % maxSide/biasRatio and pulls the excess from the slow side, which
            % preserves TL + TR == totalWheelTorque exactly.
            [TL, TR] = obj.applyBiasRatio(TL, TR);

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
            % APPLYBIASRATIO Cap T_high / T_low at biasRatio, preserving total.
            %   When the requested split exceeds the bias ratio, redistribute
            %   to the maximum allowed split. For total torque T and bias
            %   ratio b, the capped split is T_high = T*b/(b+1),
            %   T_low = T/(b+1) (the closed-form solution of
            %   T_high/T_low = b with T_high + T_low = T). The earlier
            %   incremental rescale was wrong: it computed the floor from the
            %   pre-transfer max side, so raising the low side and pulling
            %   from the high side overshot and collapsed the bias.
            if ~isfinite(obj.biasRatio) || obj.biasRatio <= 0
                return;
            end
            total = TL + TR;
            if total <= eps
                return;
            end
            ratio = max(TL, TR) / max(min(TL, TR), eps);
            if ratio <= obj.biasRatio
                return;   % within the allowed bias
            end
            b = obj.biasRatio;
            highSide = total * b / (b + 1);
            lowSide  = total / (b + 1);
            if TL >= TR
                TL = highSide;
                TR = lowSide;
            else
                TR = highSide;
                TL = lowSide;
            end
        end
    end
end
