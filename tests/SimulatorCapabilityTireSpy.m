classdef SimulatorCapabilityTireSpy < handle
    properties
        RL = struct('wheelRadius', 0.25)
        slipOffset = 0
    end
    methods
        function kappa = computeSlipRatioFromKinematics(obj, corner, speed)
            kappa = obj.slipOffset + (corner.angularVelocity * corner.wheelRadius - speed) / speed;
        end
    end
end
