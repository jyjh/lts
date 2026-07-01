classdef SimulatorChassisOnlySuspensionSpy < handle
    properties
        algebraicCalls = 0
        chassisCalls = 0
        loadPerCorner
    end

    methods
        function obj = SimulatorChassisOnlySuspensionSpy(loadPerCorner)
            obj.loadPerCorner = loadPerCorner;
        end

        function loads = estimateCornerLoads(obj, ~, ~, ~, ~)
            loads = obj.staticLoads();
        end

        function loads = computeCornerLoads(obj, ~, ~, ~, ~, ~)
            obj.algebraicCalls = obj.algebraicCalls + 1;
            error('SimulatorChassisOnlySuspensionSpy:AlgebraicPathCalled', ...
                'Chassis mode must not call computeCornerLoads.');
        end

        function loads = computeCornerLoadsFromChassis(obj, ~, ~, ~)
            obj.chassisCalls = obj.chassisCalls + 1;
            loads = obj.staticLoads();
        end

        function kin = getCornerKinematics(~)
            zero = struct('camberAngle', 0, 'toeAngle', 0, 'steerAngle', 0);
            kin = struct('FL', zero, 'FR', zero, 'RL', zero, 'RR', zero);
        end

        function loads = staticLoads(obj)
            loads = struct( ...
                'FL', obj.loadPerCorner, ...
                'FR', obj.loadPerCorner, ...
                'RL', obj.loadPerCorner, ...
                'RR', obj.loadPerCorner);
        end
    end
end
