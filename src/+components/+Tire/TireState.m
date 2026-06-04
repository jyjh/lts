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
        
        % Slip ratio [-1 to 1] (positive = driving, negative = braking)
        slipRatio       = 0
        
        % Inclination (camber) angle [rad] (positive = top tilted outward)
        camberAngle     = 0
        
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
        
        % Peak friction coefficient at current load
        peakMu          = 0
    end
    
    methods
        function obj = TireState()
            % TIRESTATE Construct with zero initial conditions
            obj.normalForce = 0;
            obj.slipAngle   = 0;
            obj.slipRatio   = 0;
            obj.camberAngle = 0;
            obj.Fy          = 0;
            obj.Fx          = 0;
            obj.Mx          = 0;
            obj.My          = 0;
            obj.Mz          = 0;
            obj.peakMu      = 0;
        end
        
        function reset(obj)
            % RESET Reset all state to zero
            obj.normalForce = 0;
            obj.slipAngle   = 0;
            obj.slipRatio   = 0;
            obj.camberAngle = 0;
            obj.Fy          = 0;
            obj.Fx          = 0;
            obj.Mx          = 0;
            obj.My          = 0;
            obj.Mz          = 0;
            obj.peakMu      = 0;
        end
    end
end