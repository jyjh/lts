classdef DrexlerRampPlateDifferential < lts.components.Powertrain.DifferentialComponent
    % DREXLERRAMPPLATEDIFFERENTIAL Drexler-style ramp-plate mechanical LSD
    %
    % Models a calibrated clutch/ramp differential with separate acceleration
    % and deceleration ramp angles. The oil is carried as metadata only; the
    % Motul 75W-140 data sheet confirms LSD suitability but does not publish a
    % clutch friction coefficient usable by this model.

    properties
        % Ramp angles [deg]. Lower angle gives stronger ramp action for a
        % given rampTorqueScale because the simple ramp multiplier is cot(a).
        accelRampAngleDeg = 30
        decelRampAngleDeg = 45

        % Measured static breakaway torque difference across the outputs [Nm].
        % NaN means the differential is intentionally not calibration-ready.
        preloadBreakawayTorqueNm = NaN

        % Calibrated multiplier from wheel-side driveline torque to output
        % torque-difference capacity. NaN prevents accidental activation.
        rampTorqueScale = NaN

        % Fluid metadata; not used in the equations.
        fluid = "Motul Gear Competition 75W-140"

        % Optional numerical smoothing near zero relative wheel speed [rad/s].
        % 0 keeps the ideal Coulomb-style lock response.
        slipSmoothingRadPerSec = 0
    end

    methods
        function obj = DrexlerRampPlateDifferential(varargin)
            if mod(nargin, 2) ~= 0
                error('DrexlerRampPlateDifferential:BadArgs', ...
                    'Arguments must be name-value pairs.');
            end
            for i = 1:2:nargin
                name = varargin{i};
                if ~isprop(obj, name)
                    error('DrexlerRampPlateDifferential:UnknownProperty', ...
                        'Unknown property "%s".', name);
                end
                obj.(name) = varargin{i + 1};
            end
            obj.fluid = string(obj.fluid);
        end

        function out = solveDrive(obj, totalWheelTorque, omegaL, omegaR, wheelInertia, dt)
            out = obj.solveDriveline( ...
                max(0, totalWheelTorque), 0, omegaL, omegaR, wheelInertia, dt);
        end

        function out = solveDriveline(obj, driveWheelTorque, coastdownWheelTorque, omegaL, omegaR, ~, ~)
            obj.validateCalibration();

            driveWheelTorque = max(0, driveWheelTorque);
            totalWheelTorque = driveWheelTorque + coastdownWheelTorque;
            base = 0.5 * totalWheelTorque;

            % Ramp plates create a torque-difference capacity proportional to
            % driveline load. Accel and decel use separate ramp angles; lower
            % angles have larger cot(angle) leverage and therefore lock harder.
            accelLock = driveWheelTorque * obj.rampMultiplier(obj.accelRampAngleDeg);
            decelLock = abs(coastdownWheelTorque) * obj.rampMultiplier(obj.decelRampAngleDeg);
            lockTorqueDifference = obj.preloadBreakawayTorqueNm + ...
                obj.rampTorqueScale * (accelLock + decelLock);

            % Bias torque opposes relative output speed: if the right wheel is
            % faster (dw > 0), torque is shifted to the left/slower wheel.
            dw = omegaR - omegaL;
            if abs(dw) <= eps
                biasSign = 0;
            else
                biasSign = sign(dw);
            end

            smoothing = max(0, obj.slipSmoothingRadPerSec);
            if smoothing > 0
                slipScale = min(1, abs(dw) / smoothing);
            else
                slipScale = 1;
            end

            biasTorque = 0.5 * lockTorqueDifference * slipScale;
            out.TL = base + biasSign * biasTorque;
            out.TR = base - biasSign * biasTorque;
            out.carrierOmega = 0.5 * (omegaL + omegaR);
            out.lockTorqueDifference = lockTorqueDifference * slipScale;
            out.accelRampAngleDeg = obj.accelRampAngleDeg;
            out.decelRampAngleDeg = obj.decelRampAngleDeg;
        end

        function validateCalibration(obj)
            if ~isfinite(obj.preloadBreakawayTorqueNm) || ...
                    obj.preloadBreakawayTorqueNm < 0 || ...
                    ~isfinite(obj.rampTorqueScale) || obj.rampTorqueScale < 0
                error('DrexlerRampPlateDifferential:Uncalibrated', ...
                    ['Drexler LSD requires finite non-negative ' ...
                    'preloadBreakawayTorqueNm and rampTorqueScale before use.']);
            end
            obj.validateRampAngle(obj.accelRampAngleDeg, 'accelRampAngleDeg');
            obj.validateRampAngle(obj.decelRampAngleDeg, 'decelRampAngleDeg');
        end

        function locked = locksWheels(~)
            locked = false;
        end

        function name = getName(~)
            name = 'DrexlerRampPlateDifferential';
        end
    end

    methods (Access = private)
        function multiplier = rampMultiplier(~, angleDeg)
            multiplier = 1 / tand(angleDeg);
        end

        function validateRampAngle(~, angleDeg, fieldName)
            if ~isfinite(angleDeg) || angleDeg <= 0 || angleDeg >= 90
                error('DrexlerRampPlateDifferential:InvalidRampAngle', ...
                    '%s must be finite and between 0 and 90 degrees.', fieldName);
            end
        end
    end
end
