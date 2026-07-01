function cfg = baseline()
    % BASELINE Reference FSAE car configuration
    %
    % Returns a fully-populated VehicleConfig describing the current baseline
    % car. Every value is spelled out explicitly (rather than relying on the
    % VehicleConfig defaults) so this file also serves as:
    %   1. documentation of the baseline car, and
    %   2. a copy/paste template for new car configs.
    %
    % To define a new car:
    %   1. Copy this file to +vehicles/<yourcar>.m.
    %   2. Edit the values below.
    %   3. In run_simulation.m:  config = vehicles.<yourcar>();
    %
    % Units: SI throughout (m, kg, N, s, rad, Pa).

    cfg = VehicleConfig();

    %% ====================================================================
    %  VEHICLE-LEVEL CONSTANTS
    %  Mass includes the driver. x forward, y left, z up; CG height from
    %  the ground. Pitch/roll inertia are derived in SimpleChassis from
    %  mass + geometry; only yaw inertia is specified here.
    %  ====================================================================
    cfg.totalMass            = 264;      % Total mass with driver [kg]
    cfg.wheelbase            = 1.558;    % Wheelbase [m]
    cfg.trackWidth           = 1.21;     % Track width [m]
    cfg.cgHeight             = 0.3;      % CG height above ground [m]
    cfg.yawInertia           = 130;      % Yaw inertia about CG [kg*m^2]
    cfg.airDensity           = 1.225;    % Air density [kg/m^3]
    cfg.staticFrontWeight    = 0.5038;     % Static front weight distribution [0-1]
    cfg.brakeBiasFront       = 0.60;     % Brake force fraction to front axle [0-1] TODO
    cfg.brakeForceCoefficient = 0.70;    % Brake force capacity as fraction of normal load (no ABS) TODO
    cfg.maxSpeed             = 80;       % Soft speed limiter [m/s] (~288 km/h)
    cfg.unsprungMass         = 25;       % Per-corner unsprung mass [kg] TODO

    %% ====================================================================
    %  AERODYNAMICS
    %  Each element is positioned at (xPosition, zPosition) — xPosition > 0
    %  forward of CG, < 0 behind — and produces downforce F = 0.5*rho*ClA*V^2
    %  and drag F = 0.5*rho*CdA*V^2. Wings use a linear ride-height model
    %  (heightSensitivity = fractional ClA change per cm); the floor uses an
    %  exponential ground-effect model (heightExponent + stallHeight).
    %  pitchSensitivityClA [1/rad]: fractional ClA change per rad of body
    %  pitch (nose-up positive); negative = loses downforce nose-up.
    %  ====================================================================

    % Front wing: ahead of front axle, very pitch/height sensitive.
    cfg.frontWing = struct( ...
        'xPosition', 0.9, ...            % 0.9 m forward of CG (ahead of front axle)
        'zPosition', 0.08, ...           % 8 cm above reference plane
        'ClA', 1.6, ...                  % Downforce coefficient * area [m^2]
        'CdA', 0.35, ...                 % Drag coefficient * area [m^2]
        'pitchSensitivityClA', -5.0, ... % Loses downforce when nose pitches up [1/rad]
        'heightSensitivity', 0.3);       % Fractional ClA change per cm of height

    % Rear wing: behind rear axle, moderate pitch sensitivity.
    cfg.rearWing = struct( ...
        'xPosition', -0.85, ...          % 0.85 m behind CG (behind rear axle)
        'zPosition', 0.45, ...           % 45 cm above reference plane (high-mounted)
        'ClA', 2.1, ...                  % Highest downforce element [m^2]
        'CdA', 1.15, ...                 % Highest drag element [m^2]
        'pitchSensitivityClA', 3.0, ...  % Gains downforce when nose pitches up [1/rad]
        'heightSensitivity', 0.15);      % Fractional ClA change per cm of height

    % Underbody floor / diffuser: near CG, extremely height sensitive.
    cfg.underbody = struct( ...
        'xPosition', 0.0, ...            % At CG
        'zPosition', 0.035, ...          % 3.5 cm (nominal floor height)
        'ClA', 0.4, ...                  % Moderate downforce [m^2]
        'CdA', 0.10, ...                 % Very low drag [m^2]
        'pitchSensitivityClA', -8.0, ... % Very pitch-sensitive (ground effect) [1/rad]
        'stallHeight', 0.015, ...        % Downforce collapses below 1.5 cm [m]
        'heightExponent', 0.6);          % Ground-effect curve exponent (higher = steeper)

    %% ====================================================================
    %  SUSPENSION
    %  Per-axle spring/damper (front and rear corners share params within an
    %  axle). Wheel rate = springRate * motionRatio^2. The front/rear elastic
    %  load-transfer split is derived from springs + anti-roll bars unless
    %  rollStiffnessOverride is set.
    %  ====================================================================

    cfg.suspension.front = struct( ...
        'springRate', 43780, ...         % Heave spring rate [N/m]
        'dampingCoeff', 3000, ...        % Compression (bump) damping [N*s/m] TODO
        'reboundCoeff', 4500);           % Rebound (droop) damping [N*s/m] TODO

    cfg.suspension.rear = struct( ...
        'springRate', 39400, ...         % Heave spring rate [N/m] (slightly softer)
        'dampingCoeff', 2800, ...        % Compression (bump) damping [N*s/m] TODO
        'reboundCoeff', 4200);           % Rebound (droop) damping [N*s/m] TODO

    cfg.suspension.motionRatio    = 1;     % Installation motion ratio (wheel<->spring)
    cfg.suspension.bumpStopLength = 0.025;    % Free travel before bump stop engages [m] TODO
    cfg.suspension.bumpStopRate   = 200000;   % Bump stop stiffness [N/m] TODO
    cfg.suspension.tireSpringRate = 200000;   % Vertical tire stiffness [N/m] TODO

    % Suspension geometry: per-axle lookup tables indexed by wheel travel
    % [m] (bump/compression = positive, rebound = negative). Values are
    % linearly interpolated across travelGrid and extrapolated past the ends.
    %   camberCurve/toeCurve in [rad] (camber positive = top-outward,
    %                                  toe positive = toe-left)
    %   motionRatioCurve [-] referenced to the wheel
    %   rollCenterHeight [m] above ground (drives the geometric load transfer)
    % Vehicle-level wheelbase/track/weight are pulled from the VehicleManager
    % at construction, so they are not duplicated here. TODO
    cfg.suspension.geometry.front = struct( ...
        'travelGrid',       [-0.05 0 0.05], ...
        'camberCurve',      [0.5 0 -1.5] * pi / 180, ...   % gains neg. camber in bump
        'toeCurve',         [-0.05 0 0.05] * pi / 180, ... % toes out in bump
        'motionRatioCurve', [0.93 0.95 0.97], ...
        'rollCenterHeight', 0.030);                        % slightly above ground
    cfg.suspension.geometry.rear = struct( ...
        'travelGrid',       [-0.05 0 0.05], ...
        'camberCurve',      [0.25 0 -0.8] * pi / 180, ...  % less camber gain than front
        'toeCurve',         [0.05 0 -0.05] * pi / 180, ... % toes in in bump
        'motionRatioCurve', [0.94 0.95 0.96], ...
        'rollCenterHeight', 0.045);                        % a bit higher = stable platform
    % Steering model. steerInput is treated as a road-wheel angle by default.
    %   ackermann: 0 = parallel steer, 1 = ideal Ackermann.
    cfg.suspension.geometry.steering = struct( ...
        'steeringRatio',      4.856, ...
        'ackermann',          0.8872, ...
        'maxWheelSteerAngle', 0.6, ...                      % [rad] (~34 deg) road-wheel cap
        'rearSteerRatio',     0.0);

    % Anti-roll bars: described by end stiffness, motion ratio, and
    % drop-link lever arm; wheel-rate roll stiffness is added to the axle's
    % wheel springs to derive the F/R elastic load-transfer split.
    % Wheel-rate target: front ~25 kN/m, rear ~15 kN/m (stiffer front is the
    % common FSAE setup to suppress front roll and tune steady-state balance).
    cfg.suspension.frontArb = struct( ...
        'stiffness', 1800, ...           % [N/m] at the bar end
        'motionRatio', 0.95, ...         % wheel <-> bar end
        'leverArm', 0.26, ...            % [m] bar axis to drop-link
        'enabled', true);
    cfg.suspension.rearArb = struct( ...
        'stiffness', 1100, ...           % [N/m] at the bar end (softer than front)
        'motionRatio', 0.95, ...         % wheel <-> bar end
        'leverArm', 0.26, ...            % [m] bar axis to drop-link
        'enabled', true);

    % NaN = derive the F/R roll-stiffness split from springs + ARBs.
    % Set a value in [0,1] to force a fixed front fraction (legacy A/B).
    cfg.suspension.rollStiffnessOverride = NaN;
    % Opt-in twist coupling between chassis roll DOFs and load transfer.
    cfg.suspension.coupleChassisRollToLoadTransfer = false;

    %% ====================================================================
    %  CHASSIS (sprung-mass platform heave/pitch/roll)
    %  The sprung mass is a lumped heave/pitch/2xroll body. Heave uses
    %  translational units; pitch/roll/twist use rotational units. Pitch and
    %  roll inertia are derived from mass + geometry in SimpleChassis, so
    %  they are not configured here.
    %  ====================================================================
    cfg.chassis = struct( ...
        'heaveStiffness', 160000, ...    % [N/m]
        'heaveDamping', 12000, ...       % [N*s/m]
        'pitchStiffness', 90000, ...     % [N*m/rad]
        'pitchDamping', 6000, ...        % [N*m*s/rad]
        'rollStiffness', 55000, ...      % [N*m/rad] (legacy whole-car average)
        'rollDamping', 5000, ...         % [N*m*s/rad]
        'torsionalRigidity', 162518, ... % [N*m/rad] couples front vs rear roll (twist); ~4000 N*m/deg
        'torsionalDamping', 2000);       % [N*m*s/rad] damps the twist rate

    %% ====================================================================
    %  POWERTRAIN
    %  Single-speed EV (EMRAX 228). matFile = '' selects the default motor
    %  map in +Powertrain/; final drive is fixed in the map. Drivetrain is
    %  RWD (drive torque only on the rear axle).
    %    differential.type: 'open' | 'locked' (spool) | 'lsd'
    %      ('lsd' may carry preload, ramp, speedGain, biasRatio)
    %  ====================================================================
    cfg.powertrain = struct( ...
        'matFile', '', ...
        'efficiency', 0.92, ...          % Drivetrain efficiency [0-1]
        'differential', struct('type', 'open'));

    %% ====================================================================
    %  TIRE
    %  Pacejka Magic Formula (MF 6.1) via MFeval; tirFile lives in +Tire/.
    %  Grip is set entirely by the tire data (peak mu with load sensitivity)
    %  — there is no surface-friction cap, so the driver and tire agree.
    %  ====================================================================
    cfg.tire = struct( ...
        'tirFile', '43105_18x7.5_10_R25B_7.tir', ...
        'wheelInertia', 0.5, ...         % Wheel+tire+brake inertia per corner [kg*m^2]
        'relaxationLength', 0.30, ...    % Contact-patch slip lag [m] (0 = steady-state)
        'wheelRadius', 0.241935, ...     % Effective rolling radius [m]
        'rollingResistanceCoeff', 0.015, ... % Crr; per-wheel resistance torque T_rr = Crr*Fz*R [-]
        'bearingDragCoeff', 0);          % Viscous wheel-hub drag T_b = coeff*omega [N*m*s/rad] (0 = off)
end
