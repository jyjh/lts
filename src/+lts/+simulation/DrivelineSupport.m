classdef DrivelineSupport
    % Wheel inertia and driven-axle helpers.

    methods (Static)
        function inertia = wheelInertia(vehicleManager)
            baseI = 0.5;
            if ~isempty(vehicleManager.tire) && isprop(vehicleManager.tire, 'wheelInertia')
                baseI = vehicleManager.tire.wheelInertia;
            end

            reflectedRotorInertia = 0;
            if ~isempty(vehicleManager.powertrain) && ...
                    isa(vehicleManager.powertrain, 'lts.components.Powertrain.PowertrainComponent') && ...
                    ismethod(vehicleManager.powertrain, 'getReflectedRotorInertia')
                reflectedRotorInertia = ...
                    vehicleManager.powertrain.getReflectedRotorInertia();
            end

            % Rotor inertia belongs at the differential carrier.
            inertia = struct('FL', baseI, 'FR', baseI, ...
                'RL', baseI, 'RR', baseI, ...
                'reflectedRotorInertia', reflectedRotorInertia);
        end

        function out = solveDifferential(vehicleManager, totalDriveTorque, totalCoastdownTorque, ...
                omegaL, omegaR, wheelInertia, dt)
            if ~isempty(vehicleManager.differential) && ...
                    isa(vehicleManager.differential, 'lts.components.Powertrain.DifferentialComponent')
                out = vehicleManager.differential.solveDriveline( ...
                    totalDriveTorque, totalCoastdownTorque, ...
                    omegaL, omegaR, wheelInertia, dt);
            else
                totalDriveTorque = max(0, totalDriveTorque);
                totalWheelTorque = totalDriveTorque + totalCoastdownTorque;
                out.TL = 0.5 * totalWheelTorque;
                out.TR = 0.5 * totalWheelTorque;
                out.carrierOmega = 0.5 * (omegaL + omegaR);
            end
        end

        function locked = locksWheels(vehicleManager)
            locked = false;
            if ~isempty(vehicleManager.differential) && ...
                    isa(vehicleManager.differential, 'lts.components.Powertrain.DifferentialComponent') && ...
                    ismethod(vehicleManager.differential, 'locksWheels')
                locked = vehicleManager.differential.locksWheels();
            end
        end

        function initializeWheelSpeeds(vehicleManager, vehicleSpeed)
            if vehicleSpeed <= 0
                return;
            end

            tire = vehicleManager.tire;
            lts.simulation.DrivelineSupport.initializeCornerWheelSpeed(tire.FL, vehicleSpeed);
            lts.simulation.DrivelineSupport.initializeCornerWheelSpeed(tire.FR, vehicleSpeed);
            lts.simulation.DrivelineSupport.initializeCornerWheelSpeed(tire.RL, vehicleSpeed);
            lts.simulation.DrivelineSupport.initializeCornerWheelSpeed(tire.RR, vehicleSpeed);
            vehicleManager.powertrain.updateStateFromDrivenWheels( ...
                [tire.RL.angularVelocity, tire.RR.angularVelocity]);
        end

        function initializeCornerWheelSpeed(cornerState, vehicleSpeed)
            if cornerState.angularVelocity > 0
                return;
            end

            cornerState.angularVelocity = max(vehicleSpeed, 0) / ...
                max(cornerState.wheelRadius, eps);
        end
    end
end
