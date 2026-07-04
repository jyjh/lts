classdef BrakeForcePolicy
    % BRAKEFORCEPOLICY Converts brake inputs into commanded axle forces.

    methods (Static)
        function brakeForces = compute(input, totalNormalLoad, vehicleManager, brakeMode)
            brakeCommand = max(0, min(1, lts.util.fieldOr(input, 'brake', 0)));
            totalNormalLoad = max(0, totalNormalLoad);
            brakeForceCapacity = max(0, vehicleManager.brakeForceCoefficient) * totalNormalLoad;
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

            brakeBiasFront = max(0, min(1, vehicleManager.brakeBiasFront));
            brakeForceMag = brakeCommand * brakeForceCapacity;
            brakeForces.frontForce = brakeForceMag * brakeBiasFront;
            brakeForces.rearForce = brakeForceMag * (1 - brakeBiasFront);
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
