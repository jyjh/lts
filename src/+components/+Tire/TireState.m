classdef TireState < handle
    % TIRESTATE Mutable per-corner tire state
    %
    % Holds the dynamic state variables for one tire (corner) of the car.
    % Uses handle inheritance so that PacejkaTire can mutate state in-place
    % across timesteps, mirroring the SuspensionState pattern.
    %
    % All forces are in the tire's local coordinate system.
    % Slip angle and slip ratio are the inputs that produced the forces.
    
    properties
        % --- Inputs (set each timestep) ---
        
        % Normal force at tire contact patch [N] (from suspension)
        normalForce     = 0
        
        % Slip angle [rad] (positive = left turn for this tire)
        slipAngle       = 0

        % Raw target slip angle before relaxation [rad]
        targetSlipAngle = 0

        % Relaxed force-producing slip angle [rad]
        relaxedSlipAngle = 0
        
        % Slip ratio [-1 to 1] (positive = driving, negative = braking)
        slipRatio       = 0

        % Raw target slip ratio before relaxation [-1 to 1]
        targetSlipRatio = 0

        % Relaxed force-producing slip ratio [-1 to 1]
        relaxedSlipRatio = 0

        % True once relaxation states have been initialized
        slipStateInitialized = false
        
        % Inclination (camber) angle [rad] (positive = top tilted outward)
        camberAngle     = 0
        
        % --- Wheel rotational state ---
        
        % Wheel angular velocity [rad/s] (positive = rolling forward)
        angularVelocity = 0
        
        % Effective tire rolling radius [m]
        wheelRadius     = 0.241935
        
        % --- Outputs (computed each timestep) ---
        
        % Lateral force Fy [N] (positive = left)
        Fy              = 0
        
        % Longitudinal force Fx [N] (positive = driving)
        Fx              = 0
        
        % Overturning moment Mx [Nm]
        Mx              = 0
        
        % Rolling resistance moment My [Nm]
        My              = 0
        
        % Aligning torque Mz [Nm]
        Mz              = 0
        
        % Peak lateral friction coefficient at current load
        peakMu          = 0

        % Peak longitudinal friction coefficient at current load
        peakMuLong      = 0

        % Combined tire force limit in the current force direction and usage
        frictionLimit   = 0
        frictionUsage   = 0
    end
    
    methods
        function obj = TireState()
            % TIRESTATE Construct with zero initial conditions
            obj.normalForce     = 0;
            obj.slipAngle       = 0;
            obj.targetSlipAngle = 0;
            obj.relaxedSlipAngle = 0;
            obj.slipRatio       = 0;
            obj.targetSlipRatio = 0;
            obj.relaxedSlipRatio = 0;
            obj.slipStateInitialized = false;
            obj.camberAngle     = 0;
            obj.angularVelocity = 0;
            % wheelRadius keeps its default effective rolling radius
            obj.Fy              = 0;
            obj.Fx              = 0;
            obj.Mx              = 0;
            obj.My              = 0;
            obj.Mz              = 0;
            obj.peakMu          = 0;
            obj.peakMuLong      = 0;
            obj.frictionLimit   = 0;
            obj.frictionUsage   = 0;
        end

        function set.normalForce(obj, value)
            obj.normalForce = utils.nonnegativeScalarOrDefault(value, 0);
        end

        function set.slipAngle(obj, value)
            obj.slipAngle = utils.scalarOrDefault(value, 0);
        end

        function set.targetSlipAngle(obj, value)
            obj.targetSlipAngle = utils.scalarOrDefault(value, 0);
        end

        function set.relaxedSlipAngle(obj, value)
            obj.relaxedSlipAngle = utils.scalarOrDefault(value, 0);
        end

        function set.slipRatio(obj, value)
            obj.slipRatio = utils.unitSignedScalarOrDefault(value, 0);
        end

        function set.targetSlipRatio(obj, value)
            obj.targetSlipRatio = utils.unitSignedScalarOrDefault(value, 0);
        end

        function set.relaxedSlipRatio(obj, value)
            obj.relaxedSlipRatio = utils.unitSignedScalarOrDefault(value, 0);
        end

        function set.slipStateInitialized(obj, value)
            obj.slipStateInitialized = utils.logicalScalarOrDefault(value, false);
        end

        function set.camberAngle(obj, value)
            obj.camberAngle = utils.scalarOrDefault(value, 0);
        end

        function set.angularVelocity(obj, value)
            obj.angularVelocity = utils.nonnegativeScalarOrDefault(value, 0);
        end

        function set.wheelRadius(obj, value)
            obj.wheelRadius = utils.positiveScalarOrDefault(value, 0.241935);
        end

        function set.Fy(obj, value)
            obj.Fy = utils.scalarOrDefault(value, 0);
        end

        function set.Fx(obj, value)
            obj.Fx = utils.scalarOrDefault(value, 0);
        end

        function set.Mx(obj, value)
            obj.Mx = utils.scalarOrDefault(value, 0);
        end

        function set.My(obj, value)
            obj.My = utils.scalarOrDefault(value, 0);
        end

        function set.Mz(obj, value)
            obj.Mz = utils.scalarOrDefault(value, 0);
        end

        function set.peakMu(obj, value)
            obj.peakMu = utils.nonnegativeScalarOrDefault(value, 0);
        end

        function set.peakMuLong(obj, value)
            obj.peakMuLong = utils.nonnegativeScalarOrDefault(value, 0);
        end

        function set.frictionLimit(obj, value)
            obj.frictionLimit = utils.nonnegativeScalarOrDefault(value, 0);
        end

        function set.frictionUsage(obj, value)
            obj.frictionUsage = utils.unitScalarOrDefault(value, 0);
        end
        
        function reset(obj)
            % RESET Reset all dynamic state to zero
            obj.normalForce     = 0;
            obj.slipAngle       = 0;
            obj.targetSlipAngle = 0;
            obj.relaxedSlipAngle = 0;
            obj.slipRatio       = 0;
            obj.targetSlipRatio = 0;
            obj.relaxedSlipRatio = 0;
            obj.slipStateInitialized = false;
            obj.camberAngle     = 0;
            obj.angularVelocity = 0;
            obj.Fy              = 0;
            obj.Fx              = 0;
            obj.Mx              = 0;
            obj.My              = 0;
            obj.Mz              = 0;
            obj.peakMu          = 0;
            obj.peakMuLong      = 0;
            obj.frictionLimit   = 0;
            obj.frictionUsage   = 0;
        end
    end
end
