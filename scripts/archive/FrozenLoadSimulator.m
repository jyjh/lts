classdef FrozenLoadSimulator < lts.simulation.Simulator
    % FROZENLOADSIMulator Diagnostic subclass for load-path ablations.
    %   Mode 'static'      : tire sees static loads; suspension not advanced.
    %   Mode 'advanceOnly' : suspension/quarter-car advances normally, but
    %                        the tire force evaluation still sees static
    %                        loads (isolates the instantaneous Fz->Fx gain).
    properties
        mode = 'static'
    end
    methods
        function obj = FrozenLoadSimulator(vehicleManager, driverModel, dt)
            obj@lts.simulation.Simulator(vehicleManager, driverModel, dt);
        end
    end
    methods
        function loads = getCurrentCornerLoads(obj, steer)
            vm = obj.vehicleManager;
            W = vm.totalMass * 9.81;
            fF = vm.staticFrontWeight;
            staticLoads = struct( ...
                'FL', 0.5 * W * fF, 'FR', 0.5 * W * fF, ...
                'RL', 0.5 * W * (1 - fF), 'RR', 0.5 * W * (1 - fF));
            switch obj.mode
                case 'static'
                    loads = staticLoads;
                otherwise  % 'advanceOnly'
                    loads = getCurrentCornerLoads@lts.simulation.Simulator( ...
                        obj, steer);
                    tireLoads = loads; %#ok<NASGU>
                    loads = staticLoads;
            end
        end
    end
end
