classdef SimulatorZeroPowertrain < components.Powertrain.PowertrainComponent
    properties
        state = components.Powertrain.PowertrainState()
    end

    methods
        function wheelTorque = computeDriveTorque(obj, ~, throttle)
            wheelTorque = 0;
            obj.state.updateOutputs(throttle, 0, 0, 0, 1, false);
        end

        function F = computeMaxDriveForce(~, ~)
            F = 0;
        end

        function updateStateFromDrivenWheels(obj, drivenWheelAngularVelocity)
            obj.state.updateFromDrivenWheels(drivenWheelAngularVelocity, 1);
        end

        function updateStateFromVehicleSpeed(obj, vehicleSpeed)
            obj.state.updateFromVehicleSpeed(vehicleSpeed, 1, 1);
        end

        function maxOmega = getMaxDrivenWheelAngularVelocity(~)
            maxOmega = inf;
        end

        function maxTorque = getMaxTorque(~, ~)
            maxTorque = 0;
        end

        function totalRatio = getTotalGearRatio(~)
            totalRatio = 1;
        end

        function efficiency = getDrivetrainEfficiency(~)
            efficiency = 1;
        end
    end
end
