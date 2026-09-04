classdef BrakeForcePolicy
    % Converts brake inputs to axle forces.
    %
    % The commanded-force capacity is the tire grip limit of the critical
    % axle under the current brake bias (gripLimitedCapacity): a race brake
    % system is sized to reach lockup, so the physical ceiling on useful
    % commanded force is tire grip, not hydraulics. The wheel solve beyond
    % it is still free to lock a wheel (torque > grip), and the planner and
    % driver models reference the same capacity, so commanded braking,
    % planned braking, and the telemetry normalization all share one
    % physical reference.

    methods (Static)
        function brakeForces = compute(input, frontNormalLoad, rearNormalLoad, vehicleManager, brakeMode)
            brakeCommand = lts.util.saturate(lts.util.fieldOr(input, 'brake', 0));
            frontNormalLoad = max(frontNormalLoad, 0);
            rearNormalLoad = max(rearNormalLoad, 0);
            brakeForceCapacity = lts.simulation.BrakeForcePolicy.gripLimitedCapacity( ...
                vehicleManager, frontNormalLoad, rearNormalLoad);
            mode = lts.simulation.BrakeForcePolicy.validateMode(brakeMode);

            brakeForces = struct( ...
                'requestedCommand', brakeCommand, ...
                'effectiveCommand', brakeCommand, ...
                'frontForce', 0, ...
                'rearForce', 0, ...
                'frontPressureBar', NaN, ...
                'rearPressureBar', NaN, ...
                'pressureModeActive', false);

            if mode == "pressure"
                frontPressureBar = lts.util.fieldOr(input, 'brakePressureFrontBar', NaN);
                rearPressureBar = lts.util.fieldOr(input, 'brakePressureRearBar', NaN);
                if ~isfinite(frontPressureBar) || ~isfinite(rearPressureBar)
                    error('lts_simulation_Simulator:MissingBrakePressureInput', ...
                        ['BrakeMode "pressure" requires finite brakePressureFrontBar ' ...
                        'and brakePressureRearBar inputs from the replay profile.']);
                end

                frontForcePerBar = vehicleManager.brakePressureFrontForcePerBar;
                rearForcePerBar = vehicleManager.brakePressureRearForcePerBar;
                if ~isfinite(frontForcePerBar) || frontForcePerBar < 0 || ...
                        ~isfinite(rearForcePerBar) || rearForcePerBar < 0
                    error('lts_simulation_Simulator:MissingBrakePressureCalibration', ...
                        ['BrakeMode "pressure" requires vehicle brakePressure ' ...
                        'frontForcePerBar and rearForcePerBar calibration.']);
                end

                frontPressureBar = max(0, frontPressureBar);
                rearPressureBar = max(0, rearPressureBar);
                brakeForces.frontPressureBar = frontPressureBar;
                brakeForces.rearPressureBar = rearPressureBar;
                brakeForces.frontForce = frontPressureBar * frontForcePerBar;
                brakeForces.rearForce = rearPressureBar * rearForcePerBar;
                if brakeForceCapacity > eps
                    brakeForces.effectiveCommand = min(1, ...
                        (brakeForces.frontForce + brakeForces.rearForce) / brakeForceCapacity);
                else
                    brakeForces.effectiveCommand = ...
                        double(brakeForces.frontForce + brakeForces.rearForce > 0);
                end
                brakeForces.pressureModeActive = true;
                return;
            end

            brakeBiasFront = lts.util.saturate(vehicleManager.brakeBiasFront);
            brakeForceMag = brakeCommand * brakeForceCapacity;
            brakeForces.frontForce = brakeForceMag * brakeBiasFront;
            brakeForces.rearForce = brakeForceMag * (1 - brakeBiasFront);
        end

        function capacity = gripLimitedCapacity(vehicleManager, frontNormalLoad, rearNormalLoad)
            % GRIPLIMITEDCAPACITY Total brake force at the grip ceiling
            %   capacity = BrakeForcePolicy.gripLimitedCapacity( ...
            %               vehicleManager, frontNormalLoad, rearNormalLoad)
            %
            %   With the applied total split by brakeBiasFront, both axles
            %   reach their longitudinal grip limit (peak |Fx|/Fz x axle
            %   load, from the current quasi-static corner loads) at this
            %   total force. The critical (grip-hungriest under the bias)
            %   axle sets the ceiling: command 1.0 puts exactly that axle at
            %   its lockup threshold, the way a driver modulates a non-ABS
            %   car at the limit, while the other axle stays below its
            %   limit by however much the bias is away from ideal.
            bias = lts.util.saturate(vehicleManager.brakeBiasFront);
            frontNormalLoad = max(frontNormalLoad, 0);
            rearNormalLoad = max(rearNormalLoad, 0);
            capacity = inf;
            if bias > eps
                capacity = min(capacity, ...
                    vehicleManager.tire.getPeakLongitudinalFriction(frontNormalLoad / 2) ...
                    * frontNormalLoad / bias);
            end
            if bias < 1
                capacity = min(capacity, ...
                    vehicleManager.tire.getPeakLongitudinalFriction(rearNormalLoad / 2) ...
                    * rearNormalLoad / (1 - bias));
            end
            capacity = max(capacity, 0);
        end

        function mode = validateMode(mode)
            mode = lower(string(mode));
            if mode ~= "ratio" && mode ~= "pressure"
                error('lts_simulation_Simulator:InvalidBrakeMode', ...
                    'BrakeMode must be "ratio" or "pressure".');
            end
        end
    end
end
