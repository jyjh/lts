classdef SimulatorZeroPowertrain < lts.components.Powertrain.PowertrainComponent
    properties
        state = lts.components.Powertrain.PowertrainState()
        totalRatio = 1
        efficiency = 1
        regenEfficiency = NaN
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
            obj.state.updateFromDrivenWheels(drivenWheelAngularVelocity, obj.totalRatio);
        end

        function updateStateFromVehicleSpeed(obj, vehicleSpeed)
            obj.state.updateFromVehicleSpeed(vehicleSpeed, 1, obj.totalRatio);
        end

        function maxOmega = getMaxDrivenWheelAngularVelocity(~)
            maxOmega = inf;
        end

        function maxTorque = getMaxTorque(~, ~)
            maxTorque = 0;
        end

        function totalRatio = getTotalGearRatio(obj)
            totalRatio = obj.totalRatio;
        end

        function efficiency = getDrivetrainEfficiency(obj)
            efficiency = obj.efficiency;
        end
    end
end
