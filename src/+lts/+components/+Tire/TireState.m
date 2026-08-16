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

        % --- Transient (relaxation) slip state ---
        % Lagged slip quantities fed to the Magic Formula. These track the
        % steady-state kinematic slip through a first-order contact-patch
        % lag (tire relaxation). See PacejkaTire.applyRelaxation.
        % slipAngle/slipRatio above hold the lagged (transient) values used
        % for the force evaluation; the steady-state kinematic inputs are
        % stored here for the next step's relaxation update.
        ssSlipAngle  = 0     % Steady-state (kinematic) slip angle [rad]
        ssSlipRatio  = 0     % Steady-state (kinematic) slip ratio [-]
        
        % Inclination (camber) angle [rad] (positive = top tilted outward)
        camberAngle     = 0

        % Lagged normal load fed to the Magic Formula [N]. Tracks
        % normalForce through the contact-patch load-response lag (see
        % PacejkaTire.normalLoadRelaxationLength). NaN until seeded by the
        % first evaluation; normalForce above stays instantaneous.
        relaxedNormalLoad = NaN
        
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
        
        % Peak friction coefficient at current load
        peakMu          = 0

        % Last raw peak-mu lookup used by this corner. PacejkaTire owns the
        % key format; the state keeps the hot per-corner hit path local.
        peakMuCacheKey  = NaN
        rawPeakMu       = NaN
    end
    
    methods
        function obj = TireState()
            % TIRESTATE Construct with zero initial conditions
            obj.normalForce     = 0;
            obj.slipAngle       = 0;
            obj.slipRatio       = 0;
            obj.ssSlipAngle     = 0;
            obj.ssSlipRatio     = 0;
            obj.camberAngle     = 0;
            obj.relaxedNormalLoad = NaN;
            obj.angularVelocity = 0;
            % wheelRadius keeps its default (0.241935 m); lts.vehicle.VehicleManager.fromConfig
            % overrides it from config.tire.wheelRadius when building via a config.
            obj.Fy              = 0;
            obj.Fx              = 0;
            obj.Mx              = 0;
            obj.My              = 0;
            obj.Mz              = 0;
            obj.peakMu          = 0;
            obj.peakMuCacheKey  = NaN;
            obj.rawPeakMu       = NaN;
        end
        
        function reset(obj)
            % RESET Reset all dynamic state to zero
            obj.normalForce     = 0;
            obj.slipAngle       = 0;
            obj.slipRatio       = 0;
            obj.ssSlipAngle     = 0;
            obj.ssSlipRatio     = 0;
            obj.camberAngle     = 0;
            obj.relaxedNormalLoad = NaN;
            obj.angularVelocity = 0;
            obj.Fy              = 0;
            obj.Fx              = 0;
            obj.Mx              = 0;
            obj.My              = 0;
            obj.Mz              = 0;
            obj.peakMu          = 0;
            obj.peakMuCacheKey  = NaN;
            obj.rawPeakMu       = NaN;
        end
    end
end
