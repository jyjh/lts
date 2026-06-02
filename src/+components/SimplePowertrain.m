classdef SimplePowertrain < components.PowertrainComponent
    % SIMPLEPOWERTRAIN Single-gear powertrain with basic torque curve
    % Uses a lookup table for engine torque and fixed gear ratio
    
    properties
        maxEngineTorque   = 55       % Peak engine torque [Nm] (typical FSAE single cylinder)
        torqueCurveRPM    = []       % RPM breakpoints for torque curve
        torqueCurveNm     = []       % Torque values [Nm] at breakpoints
        totalGearRatio    = 12.0     % Overall gear ratio (gear * final drive)
        wheelRadius       = 0.2286   % Tire rolling radius [m] (13" wheel with tire)
        drivetrainEfficiency = 0.90  % Drivetrain efficiency [0-1]
        maxEngineRPM      = 12000    % Redline [rpm]
        idleRPM           = 2000     % Idle speed [rpm]
    end
    
    methods
        function obj = SimplePowertrain(varargin)
            % SIMPLEPOWERTRAIN Construct with optional name-value pairs
            if nargin > 0
                for i = 1:2:nargin
                    if isprop(obj, varargin{i})
                        obj.(varargin{i}) = varargin{i+1};
                    end
                end
            end
            
            % Default torque curve if not specified (flat-ish with peak around 8000 rpm)
            if isempty(obj.torqueCurveRPM)
                obj.torqueCurveRPM = [0 2000 4000 6000 8000 10000 12000];
                obj.torqueCurveNm  = [0  35   45   52   55    50     40] * (obj.maxEngineTorque / 55);
            end
        end
        
        function F_drive = computeDriveForce(obj, speed, throttle)
            % Compute drive force at wheels
            throttle = max(0, min(1, throttle));  % Clamp [0,1]
            
            % Compute engine RPM from vehicle speed
            wheelRPM = speed / (2 * pi * obj.wheelRadius) * 60;
            engineRPM = wheelRPM * obj.totalGearRatio;
            
            % Engine braking if below idle (no drive force)
            if engineRPM < obj.idleRPM && throttle == 0
                F_drive = 0;
                return;
            end
            
            % Clamp engine RPM
            engineRPM = max(obj.idleRPM, min(engineRPM, obj.maxEngineRPM));
            
            % Interpolate torque from curve
            torque = interp1(obj.torqueCurveRPM, obj.torqueCurveNm, engineRPM, 'linear', 0);
            
            % Drive force at wheels
            F_drive = torque * obj.totalGearRatio * obj.drivetrainEfficiency * throttle / obj.wheelRadius;
        end
        
        function torque = getMaxTorque(obj, engineSpeed)
            % Interpolate max torque from curve
            if engineSpeed < obj.idleRPM || engineSpeed > obj.maxEngineRPM
                torque = 0;
            else
                torque = interp1(obj.torqueCurveRPM, obj.torqueCurveNm, engineSpeed, 'linear', 0);
            end
        end
        
        function ratio = getTotalGearRatio(obj)
            ratio = obj.totalGearRatio;
        end
        
        function eff = getDrivetrainEfficiency(obj)
            eff = obj.drivetrainEfficiency;
        end
    end
end