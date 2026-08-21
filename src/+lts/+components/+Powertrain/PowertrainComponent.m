classdef (Abstract) PowertrainComponent
    % POWERTRAINCOMPONENT Abstract interface for powertrain models
    % Provides wheel torque computation and motor speed state tracking.
    %
    % Torque-control contract (concrete defaults, inherited by every
    % powertrain): resolveTorques and its per-mode helpers implement the
    % Simulator's torque-control modes ("throttle", "motor_torque_command",
    % "motor_torque_delivered") generically through the abstract interface
    % above plus optional hooks. Optional hooks are probed at runtime, so a
    % minimal powertrain only needs the abstract members:
    %   methods: computeCoastdownTorque(speed, throttle),
    %           getDeliveredTorqueDrivetrainEfficiency(),
    %           getRegenDrivetrainEfficiency(),
    %           getMotoringEfficiencyAtRPM(motorRPM),
    %           isRPMLimitActive(motorRPM)
    %   properties: regenEfficiency, rpmLimitRPM,
    %           rpmLimitHysteresisRPM, wheelRadius
    % Override resolveTorques (or any helper) to plug SoC/thermal/torque-map
    % behavior in without touching the integrator.
    %
    % Concrete implementations:
    %   - EMRAX228Powertrain — supported EMRAX 228 model from a MAT map

    properties (Abstract)
        state  % lts.components.Powertrain.PowertrainState
    end

    methods (Abstract)
        % Total driven-axle wheel torque [Nm] given vehicle speed and throttle [0-1]
        wheelTorque = computeDriveTorque(obj, speed, throttle)

        % Full-throttle wheel-equivalent drive force [N] at the given vehicle
        % speed [m/s], WITHOUT mutating powertrain state. Used by the lap-time
        % planner to probe tractive capability along the speed plan.
        F = computeMaxDriveForce(obj, speed)

        % Update motor speed from driven-wheel angular velocity [rad/s]
        updateStateFromDrivenWheels(obj, drivenWheelAngularVelocity)

        % Fallback update for callers that only have vehicle speed [m/s]
        updateStateFromVehicleSpeed(obj, vehicleSpeed)

        % Driven-wheel angular velocity corresponding to the motor RPM limit [rad/s]
        maxOmega = getMaxDrivenWheelAngularVelocity(obj)

        % Maximum engine torque [Nm] at given engine speed [rpm]
        maxTorque = getMaxTorque(obj, engineSpeed)

        % Total gear ratio (including final drive)
        totalRatio = getTotalGearRatio(obj)

        % Drivetrain efficiency [0-1]
        efficiency = getDrivetrainEfficiency(obj)
    end

    methods
        function [totalDriveTorque, totalCoastdownTorque] = resolveTorques( ...
                obj, mode, input, state, throttle, limitByPackPower, ...
                driveWheelRadius)
            % RESOLVETORQUES Resolve a torque-control mode into axle torques.
            %   [driveTorque, coastdownTorque] = obj.resolveTorques( ...
            %       mode, input, state, throttle, limitByPackPower, driveWheelRadius)
            %
            %   mode              "throttle", "motor_torque_command", or
            %                     "motor_torque_delivered" (alias forms
            %                     accepted; see normalizeTorqueControlMode)
            %   input             normalized driver/replay input struct;
            %                     direct modes read motorTorqueCommandNm,
            %                     motorTorqueDeliveredNm, regenTorqueNm,
            %                     packVoltageV, packCurrentA, motorRpm, and
            %                     targetSpeed channels
            %   state             vehicle state struct (uses state.speed)
            %   throttle          normalized throttle command [0-1]
            %   limitByPackPower  true to cap direct-mode motor torque by the
            %                     measured pack power (the Simulator's
            %                     limitMotorTorqueByPackPower policy)
            %   driveWheelRadius  driven-tire rolling radius [m]; [] when the
            %                     caller has no tire model (the optional
            %                     wheelRadius property is used instead)
            %
            %   Returns totalDriveTorque >= 0 for the differential drive path
            %   and signed totalCoastdownTorque <= 0 for coast/regen drag.
            %   Motor speed must already be current: callers track it via
            %   updateStateFromDrivenWheels before resolving. Direct modes
            %   record their limiting telemetry on obj.state
            %   (requestedMotorTorque, motorTorquePowerLimitNm, pack
            %   channels) for the Simulator's stateLog.
            %
            %   Error identifiers keep the historical
            %   lts_simulation_Simulator:* names so callers and tests observe
            %   identical failures after this logic moved out of the
            %   Simulator.
            if nargin < 6 || isempty(limitByPackPower)
                limitByPackPower = false;
            end
            if nargin < 7
                driveWheelRadius = [];
            end
            mode = obj.normalizeTorqueControlMode(mode);

            switch mode
                case "throttle"
                    totalDriveTorque = obj.computeDriveTorque(state.speed, throttle);

                    % Off-throttle motoring/regen drag on the driven axle
                    % (opt-in via the powertrain component; 0 when off). This
                    % is signed driveline torque so ramp-plate LSDs can use
                    % their decel ramps. Hydraulic brake torque is separate.
                    totalCoastdownTorque = 0;
                    if ismethod(obj, 'computeCoastdownTorque')
                        totalCoastdownTorque = obj.computeCoastdownTorque( ...
                            state.speed, throttle);
                    end

                case "motor_torque_command"
                    motorTorqueCommandNm = lts.util.fieldOr(input, ...
                        'motorTorqueCommandNm', NaN);
                    if ~isfinite(motorTorqueCommandNm)
                        error('lts_simulation_Simulator:MissingMotorTorqueCommand', ...
                            ['PowertrainMode "motor_torque_command" requires a finite ' ...
                            'motorTorqueCommandNm input from the replay profile.']);
                    end

                    motorTorqueRequestNm = obj.selectDirectMotorTorqueRequest( ...
                        motorTorqueCommandNm, input);
                    wheelTorque = obj.applyMotorTorqueCommand( ...
                        motorTorqueRequestNm, throttle, input, ...
                        limitByPackPower, driveWheelRadius);
                    totalDriveTorque = max(0, wheelTorque);
                    totalCoastdownTorque = min(0, wheelTorque);

                case "motor_torque_delivered"
                    deliveredMotorTorqueNm = lts.util.fieldOr(input, ...
                        'motorTorqueDeliveredNm', NaN);
                    if ~isfinite(deliveredMotorTorqueNm)
                        error('lts_simulation_Simulator:MissingMotorTorqueDelivered', ...
                            ['PowertrainMode "motor_torque_delivered" requires a finite ' ...
                            'motorTorqueDeliveredNm input from the replay profile.']);
                    end
                    wheelTorque = obj.applyDeliveredMotorTorque( ...
                        deliveredMotorTorqueNm, throttle, input, driveWheelRadius);
                    totalDriveTorque = max(0, wheelTorque);
                    totalCoastdownTorque = min(0, wheelTorque);

                otherwise
                    error('lts_simulation_Simulator:InvalidPowertrainMode', ...
                        ['PowertrainMode must be "throttle", "motor_torque_command", ' ...
                        'or "motor_torque_delivered".']);
            end
        end

        function wheelTorque = applyDeliveredMotorTorque( ...
                obj, deliveredMotorTorqueNm, throttle, input, driveWheelRadius)
            % APPLYDELIVEREDMOTORTORQUE Apply measured (delivered) shaft torque.
            %   Reflects a delivered motor torque through the gear ratio with
            %   the delivered-torque drivetrain efficiency (losses reverse
            %   for negative/regen torque) and records the replay telemetry
            %   channels on obj.state. No power/RPM limiting is applied: the
            %   measurement is downstream of the controller.
            if nargin < 5
                driveWheelRadius = [];
            end
            ratio = obj.getTotalGearRatio();
            efficiency = obj.getDrivetrainEfficiency();
            if ismethod(obj, 'getDeliveredTorqueDrivetrainEfficiency')
                efficiency = obj.getDeliveredTorqueDrivetrainEfficiency();
            end
            if deliveredMotorTorqueNm >= 0
                wheelTorque = deliveredMotorTorqueNm * ratio * efficiency;
            else
                wheelTorque = deliveredMotorTorqueNm * ratio / max(efficiency, eps);
            end

            driveForce = 0;
            if ~isempty(driveWheelRadius)
                driveForce = wheelTorque / max(driveWheelRadius, eps);
            end
            requestedMotorTorqueNm = lts.util.fieldOr(input, ...
                'motorTorqueCommandNm', deliveredMotorTorqueNm);
            packVoltageV = lts.util.fieldOr(input, 'packVoltageV', NaN);
            packCurrentA = lts.util.fieldOr(input, 'packCurrentA', NaN);
            obj.state.updateOutputs( ...
                throttle, deliveredMotorTorqueNm, wheelTorque, driveForce, ...
                efficiency, false);
            obj.state.requestedMotorTorque = requestedMotorTorqueNm;
            obj.state.motorTorquePowerLimitNm = NaN;
            obj.state.motorTorquePowerLimitActive = false;
            obj.state.packVoltageV = packVoltageV;
            obj.state.packCurrentA = packCurrentA;
            obj.state.packPowerW = packVoltageV * packCurrentA;
        end

        function wheelTorque = applyMotorTorqueCommand( ...
                obj, motorTorqueCommandNm, throttle, input, limitByPackPower, ...
                driveWheelRadius)
            % APPLYMOTORTORQUECOMMAND Apply a commanded motor torque request.
            %   Runs the command through pack-power and rev-limiter limiting,
            %   reflects it through the gear ratio with the correct loss
            %   direction (motoring multiplies by efficiency, regen divides
            %   by it), and records the limiting telemetry channels on
            %   obj.state.
            if nargin < 4 || isempty(input)
                input = struct();
            end
            if nargin < 5 || isempty(limitByPackPower)
                limitByPackPower = false;
            end
            if nargin < 6
                driveWheelRadius = [];
            end

            ratio = obj.getTotalGearRatio();
            efficiency = obj.motoringDrivetrainEfficiency(input);
            regenEfficiency = obj.regenDrivetrainEfficiency();
            [appliedMotorTorqueNm, powerLimitNm, packVoltageV, ...
                packCurrentA, packPowerW, powerLimitActive] = ...
                obj.limitMotorTorqueCommandByPackPower( ...
                motorTorqueCommandNm, input, limitByPackPower, driveWheelRadius);
            [appliedMotorTorqueNm, rpmLimitActive] = ...
                obj.limitDirectMotorTorqueByRpm( ...
                motorTorqueCommandNm, appliedMotorTorqueNm);
            if appliedMotorTorqueNm >= 0
                wheelTorque = appliedMotorTorqueNm * ratio * efficiency;
                stateEfficiency = efficiency;
            else
                % Reverse the loss direction for regen: wheel braking power
                % must exceed the mechanical/electrical power reaching the
                % motor/pack, not shrink below it as motoring torque does.
                wheelTorque = appliedMotorTorqueNm * ratio / max(regenEfficiency, eps);
                stateEfficiency = regenEfficiency;
            end

            driveForce = 0;
            if ~isempty(driveWheelRadius)
                driveForce = wheelTorque / max(driveWheelRadius, eps);
            end

            if ~isempty(obj.state)
                obj.state.updateOutputs( ...
                    throttle, appliedMotorTorqueNm, wheelTorque, driveForce, ...
                    stateEfficiency, rpmLimitActive);
                obj.state.requestedMotorTorque = motorTorqueCommandNm;
                obj.state.motorTorquePowerLimitNm = powerLimitNm;
                obj.state.motorTorquePowerLimitActive = powerLimitActive;
                obj.state.packVoltageV = packVoltageV;
                obj.state.packCurrentA = packCurrentA;
                obj.state.packPowerW = packPowerW;
            end
        end

        function efficiency = regenDrivetrainEfficiency(obj)
            % REGENDRIVETRAINEFFICIENCY Drivetrain efficiency for regen [0-1].
            %   Prefers the optional getRegenDrivetrainEfficiency hook, then
            %   the optional regenEfficiency property, then the scalar
            %   motoring efficiency. Regen power flows wheel to motor, so
            %   this efficiency governs the wheel-side torque reflection.
            efficiency = obj.getDrivetrainEfficiency();
            if ismethod(obj, 'getRegenDrivetrainEfficiency')
                efficiency = obj.getRegenDrivetrainEfficiency();
            elseif isprop(obj, 'regenEfficiency') && ...
                    isfinite(obj.regenEfficiency)
                efficiency = obj.regenEfficiency;
            end
            efficiency = lts.util.saturate(efficiency);
        end

        function efficiency = motoringDrivetrainEfficiency(obj, input)
            % MOTORINGDRIVETRAINEFFICIENCY Drivetrain efficiency for motoring [0-1].
            %   Uses the optional RPM-dependent motoring efficiency curve
            %   when provided; the logged motorRpm input channel is
            %   preferred, then the live motor state.
            if nargin < 2 || isempty(input)
                input = struct();
            end
            efficiency = obj.getDrivetrainEfficiency();
            if ~ismethod(obj, 'getMotoringEfficiencyAtRPM')
                efficiency = lts.util.saturate(efficiency);
                return;
            end
            motorRPM = lts.util.fieldOr(input, 'motorRpm', NaN);
            if ~isfinite(motorRPM) && ~isempty(obj.state)
                motorRPM = obj.state.motorRPM;
            end
            efficiency = obj.getMotoringEfficiencyAtRPM(motorRPM);
        end

        function motorTorqueRequestNm = selectDirectMotorTorqueRequest( ...
                ~, motorTorqueCommandNm, input)
            % SELECTDIRECTMOTORTORQUEREQUEST Pick the direct-mode request.
            %   The throttle-regen channel is a candidate/request and can be
            %   nonzero during motoring. Let it override the calculated
            %   command only when the logged pack power confirms actual
            %   charging.
            if nargin < 3 || isempty(input)
                input = struct();
            end
            motorTorqueRequestNm = motorTorqueCommandNm;
            if motorTorqueCommandNm < 0
                return;
            end

            regenTorqueNm = lts.util.fieldOr(input, 'regenTorqueNm', NaN);
            if ~isfinite(regenTorqueNm) || regenTorqueNm >= 0
                return;
            end

            packVoltageV = lts.util.fieldOr(input, 'packVoltageV', NaN);
            packCurrentA = lts.util.fieldOr(input, 'packCurrentA', NaN);
            if ~isfinite(packVoltageV) || ~isfinite(packCurrentA) || packVoltageV <= 0
                return;
            end

            if packVoltageV * packCurrentA < -100
                motorTorqueRequestNm = regenTorqueNm;
            end
        end

        function [appliedMotorTorqueNm, powerLimitNm, packVoltageV, ...
                packCurrentA, packPowerW, powerLimitActive] = ...
                limitMotorTorqueCommandByPackPower(obj, requestedMotorTorqueNm, ...
                input, limitByPackPower, driveWheelRadius)
            % LIMITMOTORTORQUECOMMANDBYPACKPOWER Cap a torque command by pack power.
            %   Measured DC pack power P = V*I bounds the electrical power at
            %   the motor shaft: |T| <= |P| / omega_motor. Only applied when
            %   limitByPackPower is true and logged pack voltage/current are
            %   available. powerLimitNm is the signed torque bound (NaN when
            %   not computable); powerLimitActive reports whether the bound
            %   actually reduced the request.
            if nargin < 4 || isempty(limitByPackPower)
                limitByPackPower = false;
            end
            if nargin < 5
                driveWheelRadius = [];
            end
            appliedMotorTorqueNm = requestedMotorTorqueNm;
            powerLimitNm = NaN;
            packVoltageV = lts.util.fieldOr(input, 'packVoltageV', NaN);
            packCurrentA = lts.util.fieldOr(input, 'packCurrentA', NaN);
            packPowerW = NaN;
            powerLimitActive = false;

            if ~isfinite(packVoltageV) || ~isfinite(packCurrentA) || packVoltageV <= 0
                return;
            end

            packPowerW = packVoltageV * packCurrentA;
            if ~limitByPackPower
                return;
            end
            if ~isfinite(packPowerW) || requestedMotorTorqueNm == 0
                return;
            end

            motorOmega = obj.motorAngularVelocityForPowerLimit( ...
                input, driveWheelRadius);
            if ~isfinite(motorOmega)
                return;
            end
            motorOmega = max(abs(motorOmega), 1.0);

            if requestedMotorTorqueNm > 0
                powerLimitNm = max(0, packPowerW) / motorOmega;
                appliedMotorTorqueNm = min(requestedMotorTorqueNm, powerLimitNm);
            else
                powerLimitNm = min(0, packPowerW) / motorOmega;
                appliedMotorTorqueNm = max(requestedMotorTorqueNm, powerLimitNm);
            end

            toleranceNm = max(1e-9, 1e-6 * abs(requestedMotorTorqueNm));
            powerLimitActive = abs(appliedMotorTorqueNm - requestedMotorTorqueNm) > toleranceNm;
        end

        function [appliedMotorTorqueNm, rpmLimitActive] = ...
                limitDirectMotorTorqueByRpm(obj, requestedMotorTorqueNm, ...
                appliedMotorTorqueNm)
            % LIMITDIRECTMOTORTORQUEBYRPM Cut positive torque at the RPM cap.
            %   When the rev limiter is active the applied motor torque is
            %   clamped to <= 0 so regen/coastdown requests still pass
            %   through. Negative requests are never RPM-limited. Uses the
            %   optional isRPMLimitActive hook when provided, else the
            %   rpmLimitRPM / rpmLimitHysteresisRPM properties.
            rpmLimitActive = false;
            if requestedMotorTorqueNm <= 0
                return;
            end

            if isempty(obj.state)
                return;
            end

            motorRPM = obj.state.motorRPM;
            if ~isfinite(motorRPM)
                return;
            end

            if ismethod(obj, 'isRPMLimitActive')
                rpmLimitActive = obj.isRPMLimitActive(motorRPM);
            elseif isprop(obj, 'rpmLimitRPM')
                limitRPM = obj.rpmLimitRPM;
                if isfinite(limitRPM) && limitRPM > 0
                    releaseRPM = limitRPM;
                    if obj.state.rpmLimitActive && ...
                            isprop(obj, 'rpmLimitHysteresisRPM')
                        releaseRPM = limitRPM - max(0, obj.rpmLimitHysteresisRPM);
                    end
                    rpmLimitActive = motorRPM >= releaseRPM;
                end
            end

            if rpmLimitActive
                appliedMotorTorqueNm = min(appliedMotorTorqueNm, 0);
            end
        end

        function motorOmega = motorAngularVelocityForPowerLimit( ...
                obj, input, driveWheelRadius)
            % MOTORANGULARVELOCITYFORPOWERLIMIT Motor speed for the pack-power bound.
            %   Preference order: logged motor RPM, logged target speed
            %   reflected through the gear ratio, then the live motor state
            %   tracked from the driven wheels.
            if nargin < 3
                driveWheelRadius = [];
            end
            motorOmega = NaN;
            loggedMotorRpm = lts.util.fieldOr(input, 'motorRpm', NaN);
            if isfinite(loggedMotorRpm)
                motorOmega = loggedMotorRpm * 2 * pi / 60;
                return;
            end

            replaySpeed = lts.util.fieldOr(input, 'targetSpeed', NaN);
            if isfinite(replaySpeed) && replaySpeed > 0
                ratio = obj.getTotalGearRatio();
                wheelRadius = NaN;
                if ~isempty(driveWheelRadius)
                    wheelRadius = driveWheelRadius;
                elseif isprop(obj, 'wheelRadius')
                    wheelRadius = obj.wheelRadius;
                end
                if isfinite(ratio) && ratio > 0 && isfinite(wheelRadius) && wheelRadius > 0
                    motorOmega = replaySpeed / wheelRadius * ratio;
                    return;
                end
            end

            if ~isempty(obj.state)
                motorOmega = obj.state.motorAngularVelocity;
                if isfinite(motorOmega)
                    return;
                end
            end
        end
    end

    methods (Access = protected)
        function mode = normalizeTorqueControlMode(~, mode)
            % NORMALIZETORQUECONTROLMODE Canonicalize a torque-control mode.
            %   Accepts the historical alias spellings used by replay
            %           configurations ("motor-torque", "calculated_cmd",
            %           "motor_iq", "delivered-torque", ...). Mirrors
            %           lts.simulation.Simulator.validatePowertrainMode so
            %           both entry points agree on the mode vocabulary.
            mode = lower(string(mode));
            mode = strrep(mode, "-", "_");
            if mode == "motor_torque" || mode == "motor_command" || ...
                    mode == "calculated_cmd" || mode == "calculated_command"
                mode = "motor_torque_command";
            end
            if mode == "motor_iq" || mode == "measured_torque" || ...
                    mode == "delivered_torque"
                mode = "motor_torque_delivered";
            end
            if mode ~= "throttle" && mode ~= "motor_torque_command" && ...
                    mode ~= "motor_torque_delivered"
                error('lts_simulation_Simulator:InvalidPowertrainMode', ...
                    ['PowertrainMode must be "throttle", "motor_torque_command", ' ...
                    'or "motor_torque_delivered".']);
            end
        end
    end
end
