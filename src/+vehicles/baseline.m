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
    %  ====================================================================
    cfg.totalMass            = 256;      % Total mass with driver [kg]
    cfg.wheelbase            = 1.558;    % Wheelbase [m]
    cfg.trackWidth           = 1.21;     % Track width [m]
    cfg.cgHeight             = 0.3;      % CG height [m]
    cfg.yawInertia           = 130;      % Yaw inertia about CG [kg*m^2]
    cfg.airDensity           = 1.225;    % Air density [kg/m^3]
    cfg.staticFrontWeight    = 0.50;     % Static front weight distribution [0-1]
    cfg.brakeBiasFront       = 0.60;     % Brake force to front axle [0-1]
    cfg.brakeForceCoefficient = 0.70;    % Brake capacity as fraction of normal load
    cfg.maxSpeed             = 80;       % Speed limiter [m/s] (~288 km/h)
    cfg.unsprungMass         = 25;       % Per-corner unsprung mass [kg]

    %% ====================================================================
    %  AERODYNAMICS
    %  Each element is positioned independently and responds to vehicle
    %  pitch and ride height.
    %  ====================================================================

    % Front wing: ahead of front axle, very pitch/height sensitive.
    cfg.frontWing = struct( ...
        'xPosition', 0.9, ...            % 0.9 m forward of CG (ahead of front axle)
        'zPosition', 0.08, ...           % 8 cm above reference plane
        'ClA', 1.6, ...                  % Downforce coefficient * area
        'CdA', 0.35, ...                 % Drag coefficient * area
        'pitchSensitivityClA', -5.0, ... % Loses downforce when nose pitches up
        'heightSensitivity', 0.3);       % Sensitive to ride height

    % Rear wing: behind rear axle, moderate pitch sensitivity.
    cfg.rearWing = struct( ...
        'xPosition', -0.85, ...          % 0.85 m behind CG (behind rear axle)
        'zPosition', 0.45, ...           % 45 cm above reference plane (high-mounted)
        'ClA', 2.1, ...                  % Highest downforce element
        'CdA', 1.15, ...                 % Highest drag element
        'pitchSensitivityClA', 3.0, ...  % Gains downforce when nose pitches up
        'heightSensitivity', 0.15);      % Moderately sensitive

    % Underbody floor / diffuser: near CG, extremely height sensitive.
    cfg.underbody = struct( ...
        'xPosition', 0.0, ...            % At CG
        'zPosition', 0.035, ...          % 3.5 cm (nominal floor height)
        'ClA', 0.4, ...                  % Moderate downforce
        'CdA', 0.10, ...                 % Very low drag
        'pitchSensitivityClA', -8.0, ... % Very pitch-sensitive (ground effect)
        'stallHeight', 0.015, ...        % Stall below 1.5 cm
        'heightExponent', 0.6);          % Ground-effect sensitivity curve

    %% ====================================================================
    %  SUSPENSION
    %  Per-axle spring/damper (front and rear corners share params within an
    %  axle). The front/rear elastic load-transfer split is derived from
    %  springs + anti-roll bars unless rollStiffnessOverride is set.
    %  ====================================================================

    cfg.suspension.front = struct( ...
        'springRate', 45000, ...         % Heave spring rate [N/m]
        'dampingCoeff', 3000, ...        % Compression damping [N*s/m]
        'reboundCoeff', 4500);           % Rebound damping [N*s/m]

    cfg.suspension.rear = struct( ...
        'springRate', 42000, ...
        'dampingCoeff', 2800, ...
        'reboundCoeff', 4200);

    cfg.suspension.motionRatio    = 0.95;     % Installation motion ratio
    cfg.suspension.bumpStopLength = 0.025;    % Bump stop travel [m]
    cfg.suspension.bumpStopRate   = 200000;   % Bump stop stiffness [N/m]
    cfg.suspension.tireSpringRate = 200000;   % Vertical tire stiffness [N/m]

    % SuspensionGeometry preset name.
    % Options: 'neutral', 'baseline', 'high-camber-gain', 'pro-ackermann'
    cfg.suspension.geometryPreset = 'baseline';

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
        'stiffness', 1100, ...
        'motionRatio', 0.95, ...
        'leverArm', 0.26, ...
        'enabled', true);

    % NaN = derive the F/R roll-stiffness split from springs + ARBs.
    % Set a value in [0,1] to force a fixed front fraction (legacy A/B).
    cfg.suspension.rollStiffnessOverride = NaN;
    % Opt-in twist coupling between chassis roll DOFs and load transfer.
    cfg.suspension.coupleChassisRollToLoadTransfer = false;

    %% ====================================================================
    %  CHASSIS (sprung-mass platform heave/pitch/roll)
    %  Pitch and roll inertia are derived from mass + geometry in
    %  SimpleChassis, so they are not configured here.
    %  ====================================================================
    cfg.chassis = struct( ...
        'heaveStiffness', 160000, ...    % [N/m]
        'heaveDamping', 12000, ...       % [N*s/m]
        'pitchStiffness', 90000, ...     % [N*m/rad]
        'pitchDamping', 6000, ...        % [N*m*s/rad]
        'rollStiffness', 55000, ...      % [N*m/rad] (legacy whole-car)
        'rollDamping', 5000, ...         % [N*m*s/rad]
        'torsionalRigidity', 229183, ... % [N*m/rad] (4000 N*m/deg)
        'torsionalDamping', 2000);       % [N*m*s/rad]

    %% ====================================================================
    %  POWERTRAIN
    %  matFile = '' selects the default EMRAX 228 map in +Powertrain/.
    %  differential.type: 'open' | 'locked' | 'lsd'
    %    ('lsd' may carry preload, ramp, speedGain, biasRatio)
    %  ====================================================================
    cfg.powertrain = struct( ...
        'matFile', '', ...
        'efficiency', 0.92, ...          % Drivetrain efficiency [0-1]
        'differential', struct('type', 'open'));

    %% ====================================================================
    %  TIRE
    %  Pacejka Magic Formula via MFeval; tirFile lives in +Tire/.
    %  ====================================================================
    cfg.tire = struct( ...
        'tirFile', '43105_18x7.5_10_R25B_7.tir', ...
        'wheelInertia', 0.5, ...         % Wheel+tire+brake inertia per corner [kg*m^2]
        'relaxationLength', 0.30, ...    % Tire relaxation length [m]
        'wheelRadius', 0.241935);        % Effective rolling radius [m]
end
