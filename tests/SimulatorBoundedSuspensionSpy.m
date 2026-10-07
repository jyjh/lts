classdef SimulatorBoundedSuspensionSpy < SimulatorChassisOnlySuspensionSpy
    properties
        integrationLimit = .00025
    end
    methods
        function obj = SimulatorBoundedSuspensionSpy()
            obj@SimulatorChassisOnlySuspensionSpy(600);
        end
        function step = getMaxIntegrationStep(obj)
            step = obj.integrationLimit;
        end
    end
end
