function cfg = baseline()
    % BASELINE Reference FSAE car configuration
    %
    % Returns a fully-populated lts.vehicle.VehicleConfig describing the current baseline
    % car. Every value is spelled out explicitly (rather than relying on the
    % lts.vehicle.VehicleConfig defaults) so this file also serves as:
    %   1. documentation of the baseline car, and
    %   2. a copy/paste template for new car configs.
    %
    % To define a new car:
    %   1. Copy this file to +lts/+vehicles/<yourcar>.m.
    %   2. Edit the values below.
    %   3. In lts.app.run_simulation:  config = lts.vehicles.<yourcar>();
    %
    % Units: SI throughout (m, kg, N, s, rad, Pa).

    cfg = lts.vehicle.VehicleConfig();

    %% ====================================================================
    %  VEHICLE-LEVEL CONSTANTS
    %  Mass includes the driver. x forward, y left, z up; CG height from
    %  the ground. Pitch/roll inertia are derived in SimpleChassis from
    %  mass + geometry; only yaw inertia is specified here.
    %  ====================================================================
    cfg.totalMass            = 256;      % Total mass with driver [kg] (Assume 68kg driver)
    cfg.wheelbase            = 1.558;    % Wheelbase [m]
    cfg.trackWidth           = 1.21;     % Track width [m]
    cfg.cgHeight             = 0.3;      % CG height above ground [m]
    cfg.yawInertia           = 130;      % Yaw inertia about CG [kg*m^2]
    cfg.airDensity           = 1.225;    % Air density [kg/m^3]
    cfg.staticFrontWeight    = 0.50;     % Static front weight distribution [0-1]
    cfg.brakeBiasFront       = 0.60;     % Brake force fraction to front axle [0-1]
    cfg.brakeForceCoefficient = 0.70;    % Brake force capacity as fraction of normal load (no ABS)
    cfg.maxSpeed             = 80;       % Soft speed limiter [m/s] (~288 km/h)
    cfg.unsprungMass         = 9;       % Per-corner unsprung mass [kg]

    %% ====================================================================
    %  AERODYNAMICS
    %  Whole-car aero is represented by one resultant at the center of
    %  pressure. xPosition > 0 is forward of CG, < 0 is behind. zPosition is
    %  kept at CG height so drag adds no artificial pitch moment.
    %  ====================================================================

    % Single whole-car resultant. Values preserve the previous three-element
    % total ClA/CdA and downforce-weighted center of pressure.
    cfg.aero = struct( ...
        'xPosition', -0.084146, ...      % Center of pressure, 44.6% front aero load
        'zPosition', cfg.cgHeight, ...   % Drag resultant at CG height
        'ClA', 4.10, ...                 % Downforce coefficient * area [m^2]
        'CdA', 1.60, ...                 % Drag coefficient * area [m^2]
        'pitchSensitivityClA', 0.0);     % Whole-car CSV aero has no pitch map

    %% ====================================================================
    %  SUSPENSION
    %  Per-axle spring/damper (front and rear corners share params within an
    %  axle). Wheel rate = springRate * motionRatio^2. The front/rear elastic
    %  load-transfer split is derived from springs + anti-roll bars unless
    %  rollStiffnessOverride is set.
    %  ====================================================================

    cfg.suspension.front = struct( ...
        'springRate', 45000, ...         % Heave spring rate [N/m]
        'dampingCoeff', 3000, ...        % Compression (bump) damping [N*s/m]
        'reboundCoeff', 4500);           % Rebound (droop) damping [N*s/m]

    cfg.suspension.rear = struct( ...
        'springRate', 42000, ...         % Heave spring rate [N/m] (slightly softer)
        'dampingCoeff', 2800, ...        % Compression (bump) damping [N*s/m]
        'reboundCoeff', 4200);           % Rebound (droop) damping [N*s/m]

    cfg.suspension.motionRatio    = 0.95;     % Installation motion ratio (wheel<->spring)
    cfg.suspension.bumpStopLength = 0.025;    % Free travel before bump stop engages [m]
    cfg.suspension.bumpStopRate   = 200000;   % Bump stop stiffness [N/m]
    cfg.suspension.tireSpringRate = 200000;   % Vertical tire stiffness [N/m]

    % Suspension geometry: per-axle lookup tables indexed by wheel travel
    % [m] (bump/compression = positive, rebound = negative). Values are
    % linearly interpolated across travelGrid and extrapolated past the ends.
    %   camberCurve/toeCurve in [rad] (camber positive = top-outward,
    %                                  toe positive = toe-left)
    %   motionRatioCurve [-] referenced to the wheel
    %   rollCenterHeight [m] above ground (drives the geometric load transfer)
    %   rollCenterLateral [m] signed lateral position at +1g, scaled by axle ay
    %   casterAngle/mechanicalTrail/scrubRadius/kingpinInclination define
    %   the steering axis and contact patch sweep under steer.
    % Vehicle-level wheelbase/track/weight are pulled from the lts.vehicle.VehicleManager
    % at construction, so they are not duplicated here.
    cfg.suspension.geometry.front = struct( ...
        'travelGrid',       [-0.05 0 0.05], ...
        'camberCurve',      [0.5 0 -1.5] * pi / 180, ...   % gains neg. camber in bump
        'toeCurve',         [-0.05 0 0.05] * pi / 180, ... % toes out in bump
        'motionRatioCurve', [0.93 0.95 0.97], ...
        'rollCenterHeight', 0.030, ...                     % slightly above ground
        'rollCenterLateral', 0, ...                         % not specified
        'casterAngle',      7.0 * pi / 180, ...            % positive caster
        'mechanicalTrail',  0.030, ...                     % [m]
        'scrubRadius',      0.018, ...                     % [m], positive outboard
        'kingpinInclination', 8.0 * pi / 180, ...          % positive top-inward
        'kingpinOffset',    0.018);                        % [m], scrub fallback/alias
    cfg.suspension.geometry.rear = struct( ...
        'travelGrid',       [-0.05 0 0.05], ...
        'camberCurve',      [0.25 0 -0.8] * pi / 180, ...  % less camber gain than front
        'toeCurve',         [0.05 0 -0.05] * pi / 180, ... % toes in in bump
        'motionRatioCurve', [0.94 0.95 0.96], ...
        'rollCenterHeight', 0.045, ...                     % a bit higher = stable platform
        'rollCenterLateral', 0, ...                         % not specified
        'casterAngle',      0, ...
        'mechanicalTrail',  0, ...
        'scrubRadius',      0, ...
        'kingpinInclination', 0, ...
        'kingpinOffset',    0);
    % Steering model. steerInput is treated as a road-wheel angle by default.
    %   ackermann: 0 = parallel steer, 1 = ideal Ackermann. 0.8872 is a
    %   typical FSAE value (mostly-Ackermann for tight corners).
    cfg.suspension.geometry.steering = struct( ...
        'steeringRatio',      1.0, ...
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
        'heaveStiffness', NaN, ...        % derive from suspension springs
        'heaveDamping', NaN, ...          % derive from suspension dampers
        'pitchStiffness', NaN, ...        % derive from suspension springs
        'pitchDamping', NaN, ...          % derive from suspension dampers
        'rollStiffness', 55000, ...      % [N*m/rad] (legacy whole-car average)
        'rollDamping', NaN, ...          % derive from suspension dampers
        'torsionalRigidity', 229183, ... % [N*m/rad] couples front vs rear roll (twist); ~4000 N*m/deg
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
        'motorRotorInertia', 0.07, ...   % Motor rotor inertia [kg*m^2], reflected as I*ratio^2 to rear wheels
        'throttleMapInput', [0.00 0.15 0.35 0.60 0.80 1.00], ...
        'throttleMapOutput', [0.00 0.02 0.10 0.28 0.58 1.00], ...
        'differential', struct('type', 'lsd'));

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
