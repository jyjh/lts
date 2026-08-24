classdef SimulatorChassisOnlySuspensionSpy < handle
    % Minimal suspension stand-in for Simulator tests: returns fixed
    % per-corner loads through the chassis-driven path only.
    properties
        chassisCalls = 0
        loadPerCorner
    end

    methods
        function obj = SimulatorChassisOnlySuspensionSpy(loadPerCorner)
            obj.loadPerCorner = loadPerCorner;
        end

        function loads = computeCornerLoadsFromChassis(obj, ~, ~, ~, ~) %#ok<INUSD>
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
