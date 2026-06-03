classdef DriverModel
    % DRIVERMODEL Decides throttle and brake inputs based on vehicle state and track
    % Extracted from VehicleManager to be a swappable, testable component.
    %
    % Usage:
    %   driver = DriverModel(vehicleManager);
    %   [throttle, brake] = driver.computeInputs(speed, curKappa, curMu, s, arcLen, curvature, mu);
    
    properties
        % Reference to VehicleManager for component access
        vehicleManager
        
        % Tuneable driver parameters
        brakingLookahead = 1.0   % Lookahead factor for braking distance
        lookaheadTime    = 2.0   % Seconds ahead to look
        minLookaheadDist = 10    % Minimum lookahead distance [m]
        hysteresis       = 0.02  % 2% speed hysteresis band
        maintainThrottle = 0.5   % Partial throttle when maintaining speed
    end
    
    methods
        function obj = DriverModel(vehicleManager)
            % DRIVERMODEL Construct with a VehicleManager reference
            %   DriverModel(vehicleManager)
            obj.vehicleManager = vehicleManager;
        end
        
        function [throttle, brake] = computeInputs(obj, speed, curKappa, curMu, s, arcLen, curvature, mu)
            % COMPUTEINPUTS Decide throttle and brake for the current situation
            %   [throttle, brake] = computeInputs(speed, curKappa, curMu, s, arcLen, curvature, mu)
            %
            %   speed     - current vehicle speed [m/s]
            %   curKappa  - current track curvature [1/m]
            %   curMu     - current surface friction
            %   s         - current distance along track [m]
            %   arcLen    - arc-length parameterization of track
            %   curvature - full curvature array
            %   mu        - full friction array
            
            throttle = 0;
            brake = 0;
            
            vm = obj.vehicleManager;
            
            % Compute max lateral accel from tire grip
            tempState = VehicleState('speed', speed, 'ax', 0, 'pitchAngle', 0, 'rideHeight', 0);
            tempState.vehicleManager = vm;
            W = vm.totalMass * 9.81;
            F_downforce = vm.aero.computeDownforce(tempState);
            totalNormalLoad = W + F_downforce;
            peakMu = vm.tire.getPeakFriction(totalNormalLoad / 4);
            maxLateralAccel = peakMu * 9.81;
            
            % Current max cornering speed
            if abs(curKappa) > 1e-6
                vMaxCurrent = sqrt(maxLateralAccel / abs(curKappa));
            else
                vMaxCurrent = vm.maxSpeed;
            end
            
            % Look ahead for upcoming curvature
            lookAheadDist = speed * obj.lookaheadTime * obj.brakingLookahead;
            lookAheadDist = max(lookAheadDist, obj.minLookaheadDist);
            
            % Find look-ahead region
            idx = find(arcLen <= s, 1, 'last');
            idxEnd = find(arcLen <= s + lookAheadDist, 1, 'last');
            if isempty(idxEnd)
                idxEnd = numel(curvature);
            end
            
            % Compute max speed in look-ahead window
            upcomingKappa = curvature(idx:min(idxEnd, numel(curvature)));
            if ~isempty(upcomingKappa)
                maxUpcomingKappa = max(abs(upcomingKappa));
                if maxUpcomingKappa > 1e-6
                    vMaxAhead = sqrt(maxLateralAccel / maxUpcomingKappa);
                else
                    vMaxAhead = vm.maxSpeed;
                end
            else
                vMaxAhead = vm.maxSpeed;
            end
            
            % Decision logic
            vTarget = min(vMaxCurrent, min(vMaxAhead, vm.maxSpeed));
            
            % Compute required deceleration to reach vTarget
            if speed > vTarget * (1 + obj.hysteresis)
                % Need to brake
                if lookAheadDist > 0
                    reqDecel = (speed^2 - vTarget^2) / (2 * lookAheadDist);
                else
                    reqDecel = maxLateralAccel;
                end
                reqDecel = max(0, reqDecel);
                
                % Brake intensity [0-1]
                brake = reqDecel / maxLateralAccel;
                brake = max(0, min(1, brake));
                throttle = 0;
            elseif speed < vTarget * (1 - obj.hysteresis)
                % Can accelerate
                throttle = 1.0;
                brake = 0;
            else
                % Maintain speed - partial throttle to overcome drag
                throttle = obj.maintainThrottle;
                brake = 0;
            end
        end
    end
end