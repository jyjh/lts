classdef VehicleConfig
    % VEHICLECONFIG Per-vehicle physics configuration
    %
    % A plain value object that collects every tunable vehicle-physics
    % parameter in one place. One VehicleConfig == one car. Car definitions
    % live in the +vehicles package (e.g. vehicles.baseline) and return a
    % fully-populated VehicleConfig; VehicleManager.fromConfig turns that
    % config into a wired, ready-to-simulate VehicleManager.
    %
    % Sub-systems are nested structs so a car file can override a single
    % field (e.g. cfg.frontWing.ClA = 2.0) without re-declaring the rest.
    % The constructor pre-populates every field with the baseline values, so
    % a car file only needs to spell out the values it wants to change — but
    % vehicles.baseline spells them all out as documentation/template.
    %
    % Scope: vehicle PHYSICS only. Track selection, driver tuning, and the
    % simulation timestep stay in run_simulation.m (they are scenario/test
    % settings, not car properties).

    properties
        % ----------------------------------------------------------------
        % Vehicle-level constants
        %   Mass is car + driver; all lengths in [m]. CG height is measured
        %   from the ground. x is forward, y is left, z is up (SAE-ish).
        % ----------------------------------------------------------------
        totalMass     = 256      % Total mass with driver [kg]
        wheelbase     = 1.558    % Wheelbase [m]
        trackWidth    = 1.21     % Track width [m]
        cgHeight      = 0.3      % CG height above ground [m]
        yawInertia    = 130      % Yaw inertia about CG [kg*m^2]
        airDensity    = 1.225    % Air density [kg/m^3]
        staticFrontWeight    = 0.50 % Static front weight distribution [0-1]
        brakeBiasFront       = 0.60 % Brake force fraction to front axle [0-1]
        brakeForceCoefficient = 0.70 % Brake force capacity as a fraction of total normal load (no ABS)
        maxSpeed      = 80       % Soft speed limiter [m/s] (~288 km/h)
        unsprungMass  = 25       % Per-corner unsprung mass [kg]

        % ----------------------------------------------------------------
        % Sub-systems (nested structs, initialized in the constructor)
        % ----------------------------------------------------------------
        frontWing     % Front wing aero map (downforce/drag + pitch & height sensitivity)
        rearWing      % Rear wing aero map
        underbody     % Underbody floor / diffuser aero map (exponential ground effect)

        suspension    % Springs, damping, ARBs, bump stops, kinematic geometry
        chassis       % Sprung-mass heave/pitch/roll platform stiffness & damping

        powertrain    % Motor map, drivetrain efficiency, differential type
        tire          % Pacejka .tir data + wheel dynamics (inertia, relaxation, drag)
    end

    methods
        function obj = VehicleConfig()
            % VEHICLECONFIG Construct with baseline defaults.
            %   Every sub-system struct is pre-populated so individual
            %   fields can be overridden later (cfg.frontWing.ClA = 2.0).

            % --- Aero: front wing ---
            %   Aero elements are positioned at (xPosition, zPosition) where
            %   xPosition > 0 is forward of CG, < 0 is behind. Each element
            %   produces downforce F = 0.5*rho*ClA*V^2 and drag F = 0.5*rho*CdA*V^2.
            %   pitchSensitivityClA [1/rad]: fractional ClA change per rad of
            %     body pitch (nose-up positive); negative = loses DF nose-up.
            %   Wings use a LINEAR ride-height model:
            %     heightSensitivity = fractional ClA change per cm of height
            %     deviation from nominal (FrontWing/RearWing only).
            obj.frontWing = struct( ...
                'xPosition', 0.9, ...
                'zPosition', 0.08, ...
                'ClA', 1.6, ...
                'CdA', 0.35, ...
                'pitchSensitivityClA', -5.0, ...
                'heightSensitivity', 0.3);

            % --- Aero: rear wing ---
            obj.rearWing = struct( ...
                'xPosition', -0.85, ...
                'zPosition', 0.45, ...
                'ClA', 2.1, ...
                'CdA', 1.15, ...
                'pitchSensitivityClA', 3.0, ...
                'heightSensitivity', 0.15);

            % --- Aero: underbody floor / diffuser ---
            %   The floor uses an EXPONENTIAL ground-effect model (unlike the
            %   wings' linear one):
            %     heightFactor = (zPosition/effectiveZ)^heightExponent
            %     stallHeight [m]: below this the floor stalls (DF collapses).
            obj.underbody = struct( ...
                'xPosition', 0.0, ...
                'zPosition', 0.035, ...
                'ClA', 0.4, ...
                'CdA', 0.10, ...
                'pitchSensitivityClA', -8.0, ...
                'stallHeight', 0.015, ...
                'heightExponent', 0.6);

            % --- Suspension ---
            %   front/rear (shared within an axle):
            %     springRate   [N/m] heave spring (wheel rate = springRate*MR^2)
            %     dampingCoeff [N*s/m] compression (bump) damping
            %     reboundCoeff [N*s/m] rebound (droop) damping
            %   motionRatio     [-] installation MR (wheel<->spring); wheel rate scales by MR^2
            %   bumpStopLength  [m] free travel before the bump stop engages
            %   bumpStopRate    [N/m] bump-stop stiffness
            %   tireSpringRate  [N/m] vertical tire stiffness (quarter-car)
            %   geometry: suspension/steering kinematics (per-axle curves +
            %             steering model), see the geometry block below
            %   frontArb/rearArb: stiffness [N/m at bar end], motionRatio,
            %                     leverArm [m], enabled
            %   rollStiffnessOverride: NaN = derive from springs+ARBs, else [0-1]
            %   coupleChassisRollToLoadTransfer: opt-in twist coupling
            obj.suspension = struct( ...
                'front', struct('springRate', 45000, 'dampingCoeff', 3000, 'reboundCoeff', 4500), ...
                'rear',  struct('springRate', 42000, 'dampingCoeff', 2800, 'reboundCoeff', 4200), ...
                'motionRatio', 0.95, ...
                'bumpStopLength', 0.025, ...
                'bumpStopRate', 200000, ...
                'tireSpringRate', 200000, ...
                'frontArb', struct('stiffness', 1800, 'motionRatio', 0.95, 'leverArm', 0.26, 'enabled', true), ...
                'rearArb',  struct('stiffness', 1100, 'motionRatio', 0.95, 'leverArm', 0.26, 'enabled', true), ...
                'rollStiffnessOverride', NaN, ...
                'coupleChassisRollToLoadTransfer', false);

            % --- Suspension geometry (kinematics) ---
            %   Per-axle lookup tables indexed by wheel travel [m]
            %   (bump/compression = positive travel, rebound = negative). Values
            %   are linearly interpolated across travelGrid and extrapolated
            %   beyond the endpoints (3 points is the minimum useful resolution).
            %     travelGrid       [m] wheel-travel sample points (monotonic)
            %     camberCurve      [rad] camber vs travel (positive = top-outward)
            %     toeCurve         [rad] toe vs travel (positive = toe-left)
            %     motionRatioCurve [-] MR vs travel (referenced to the wheel)
            %     rollCenterHeight [m] above ground; drives the geometric
            %                          (instantaneous) lateral load transfer.
            %   Steering model: steerInput is treated as a road-wheel angle.
            %     ackermann: 0 = parallel steer, 1 = ideal Ackermann.
            %   (Vehicle-level wheelbase/track/weight are pulled from the
            %    VehicleManager at construction, not duplicated here.)
            obj.suspension.geometry = struct( ...
                'front', struct( ...
                    'travelGrid',       [-0.05 0 0.05], ...
                    'camberCurve',      [0.5 0 -1.5] * pi / 180, ...
                    'toeCurve',         [-0.05 0 0.05] * pi / 180, ...
                    'motionRatioCurve', [0.93 0.95 0.97], ...
                    'rollCenterHeight', 0.030), ...
                'rear', struct( ...
                    'travelGrid',       [-0.05 0 0.05], ...
                    'camberCurve',      [0.25 0 -0.8] * pi / 180, ...
                    'toeCurve',         [0.05 0 -0.05] * pi / 180, ...
                    'motionRatioCurve', [0.94 0.95 0.96], ...
                    'rollCenterHeight', 0.045), ...
                'steering', struct( ...
                    'steeringRatio',      1.0, ...
                    'ackermann',          0.8872, ...
                    'maxWheelSteerAngle', 0.6, ...
                    'rearSteerRatio',     0.0));

            % --- Chassis (sprung-mass platform heave/pitch/roll) ---
            %   The sprung mass is a lumped heave/pitch/2xroll body. Heave
            %   uses translational units; pitch/roll/twist use rotational
            %   units. Pitch/roll inertia are derived from mass + geometry in
            %   SimpleChassis, so they are not configured here.
            %     heave/pitch/roll Stiffness [N/m] / [N*m/rad]
            %     heave/pitch/roll Damping   [N*s/m] / [N*m*s/rad]
            %     torsionalRigidity [N*m/rad] couples front vs rear roll DOFs
            %       (twist angle); Inf = perfectly rigid tub. ~4000 N*m/deg.
            %     torsionalDamping [N*m*s/rad] damps the twist rate.
            obj.chassis = struct( ...
                'heaveStiffness', 160000, ...
                'heaveDamping', 12000, ...
                'pitchStiffness', 90000, ...
                'pitchDamping', 6000, ...
                'rollStiffness', 55000, ...
                'rollDamping', 5000, ...
                'torsionalRigidity', 229183, ...
                'torsionalDamping', 2000);

            % --- Powertrain ---
            %   Single-speed EV (EMRAX 228). The motor map is tractive-force
            %   vs RPM with a rev limiter; final drive is fixed in the map.
            %     matFile: '' uses the default EMRAX 228 map in +Powertrain/
            %     efficiency [0-1] drivetrain efficiency (gearbox + bearings)
            %     motorRotorInertia [kg*m^2] motor rotor inertia, reflected as
            %       I*ratio^2 onto the driven (rear) wheels (default 0.07).
            %     regenEnabled (false) opt-in regenerative braking at off-throttle
            %     motoringDragTorque [Nm] (0) opt-in motor coastdown drag
            %     regenTorqueLimitNm [Nm] (30) max regen torque, motor-side
            %     differential.type: 'open' | 'locked' (spool) | 'lsd'
            %       'lsd' may carry preload [N*m], ramp [-], speedGain [-],
            %       biasRatio [-] (torque-bias cap).
            %   Drivetrain is RWD (drive torque only on the rear axle).
            obj.powertrain = struct( ...
                'matFile', '', ...
                'efficiency', 0.92, ...
                'motorRotorInertia', 0.07, ...
                'regenEnabled', false, ...
                'motoringDragTorque', 0, ...
                'regenTorqueLimitNm', 30, ...
                'differential', struct('type', 'open'));

            % --- Tire ---
            %   Pacejka Magic Formula (MF 6.1) via MFeval. Grip is set entirely
            %   by the tire data (its peak mu with load sensitivity); there is
            %   no separate surface-friction cap, so the driver and the tire
            %   model agree on grip at the dry reference surface.
            %   surfaceMuReference is the track Mu value that maps to the
            %   unscaled tire data; procedural dry tracks use Mu = 1.2.
            %   tirFile lives in +Tire/.
            %     tirFile:  Pacejka .tir filename in +Tire/
            %     wheelInertia [kg*m^2] wheel+tire+brake rotating inertia/corner
            %     relaxationLength [m] contact-patch slip lag (0 = steady-state)
            %     wheelRadius [m] effective rolling radius
            %     rollingResistanceCoeff [-] Crr; per-wheel resistance torque
            %       T_rr = Crr*Fz*R (0 disables coast-down drag)
            %     bearingDragCoeff [N*m*s/rad] viscous wheel-hub drag
            %       T_b = coeff*omega (0 = off by default)
            obj.tire = struct( ...
                'tirFile', '43105_18x7.5_10_R25B_7.tir', ...
                'wheelInertia', 0.5, ...
                'relaxationLength', 0.30, ...
                'surfaceMuReference', 1.2, ...
                'wheelRadius', 0.241935, ...
                'rollingResistanceCoeff', 0.015, ...
                'bearingDragCoeff', 0);
        end
    end
end
