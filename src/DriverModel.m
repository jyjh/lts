classdef DriverModel
    % DRIVERMODEL Decides throttle and brake inputs based on vehicle state
    % Extracted from VehicleManager to be a swappable, testable component.
    %
    % Usage:
    %   driver = DriverModel(vehicleManager);
    %   [throttle, brake] = driver.computeInputs(state);
    
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
        
        function [throttle, brake] = computeInputs(obj, state)
            % COMPUTEINPUTS Decide throttle and brake for the current situation
            %   [throttle, brake] = computeInputs(state)
            %
            %   state - VehicleState object with current vehicle state.
            %           Expects the following properties to be set:
            %             speed     - current vehicle speed [m/s]
            %             s         - current distance along track [m]
            %             curvature - current track curvature [1/m]
            %             mu        - current surface friction
            %           The state must have vehicleManager set for access
            %           to components and track data.
            
            throttle = 0;
            brake = 0;
            
            vm = obj.vehicleManager;
            
            speed    = state.speed;
            s        = state.s;
            curKappa = state.curvature;
            
            % Get track arrays from VehicleManager's track
            track     = vm.track;
            curvature = track.getCurvature();
            trackPts  = track.getTrackPoints();
            
            % Compute arc-length parameterization
            dx = diff(trackPts(:,1));
            dy = diff(trackPts(:,2));
            arcLen = [0; cumsum(sqrt(dx.^2 + dy.^2))];
            
            % Compute max lateral accel from tire grip
            aeroForces = vm.aero.computeForces(state);
            F_downforce = aeroForces.Fz_front + aeroForces.Fz_rear;
            W = vm.totalMass * 9.81;
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