classdef SimpleSuspension < components.SuspensionComponent
    % SIMPLESUSPENSION Simple suspension model with fixed geometry
    % Computes load transfer using track width, CG height, and wheelbase
    
    properties
        trackWidth         = 1.2    % Track width [m]
        wheelbase          = 1.55   % Wheelbase [m]
        cgHeight           = 0.28   % Center of gravity height [m]
        rollStiffDist      = 0.55   % Front roll stiffness distribution [0-1]
        staticFrontWeight  = 0.48   % Static front weight distribution [0-1]
    end
    
    methods
        function obj = SimpleSuspension(varargin)
            % SIMPLESUSPENSION Construct with optional name-value pairs
            if nargin > 0
                for i = 1:2:nargin
                    if isprop(obj, varargin{i})
                        obj.(varargin{i}) = varargin{i+1};
                    end
                end
            end
        end
        
        function latTransfer = computeLatLoadTransfer(obj, ay, totalMass)
            % Total lateral load transfer = m * ay * cgHeight / trackWidth
            % Split between front and rear by roll stiffness distribution
            totalLatTransfer = totalMass * abs(ay) * obj.cgHeight / obj.trackWidth;
            
            latTransfer.front = totalLatTransfer * obj.rollStiffDist;
            latTransfer.rear  = totalLatTransfer * (1 - obj.rollStiffDist);
            
            % Sign convention: positive ay means load transfers to outside
            % Front and rear transfer are always positive magnitudes
            % The calling code handles which side gains/loses load
        end
        
        function longTransfer = computeLongLoadTransfer(obj, ax, totalMass)
            % Longitudinal load transfer = m * ax * cgHeight / wheelbase
            % Positive ax (acceleration) transfers load to rear
            totalLongTransfer = totalMass * ax * obj.cgHeight / obj.wheelbase;
            
            % Positive totalLongTransfer = load goes to rear
            longTransfer.front = -totalLongTransfer;  % Front loses load
            longTransfer.rear  =  totalLongTransfer;  % Rear gains load
        end
        
        function dist = getRollStiffnessDistribution(obj)
            dist = obj.rollStiffDist;
        end
        
        function dist = getStaticWeightDistribution(obj)
            dist = obj.staticFrontWeight;
        end
    end
end