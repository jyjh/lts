classdef VehicleConfig
    % VEHICLECONFIG Per-vehicle physics configuration
    %
    % A plain value object that collects every tunable vehicle-physics
    % parameter in one place. One lts.vehicle.VehicleConfig == one car. Car definitions
    % live in the lts.vehicles package (e.g. lts.vehicles.baseline) and return a
    % fully-populated lts.vehicle.VehicleConfig; lts.vehicle.VehicleManager.fromConfig turns that
    % config into a wired, ready-to-simulate lts.vehicle.VehicleManager.
    %
    % Sub-systems are nested structs so a car file can override a single
    % field (e.g. cfg.aero.ClA = 4.2) without re-declaring the rest.
    % The constructor pre-populates every field with the baseline values, so
    % a car file only needs to spell out the values it wants to change — but
    % lts.vehicles.baseline spells them all out as documentation/template.
    %
    % Scope: vehicle PHYSICS only for normal simulation. Correlation overlays
    % may also carry replay-only calibration hints consumed by
    % lts.app.run_correlation; track selection, driver tuning, and timestep
    % still stay in the app layer.

    properties
        name = "VehicleConfig"
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
        brakePressure        % Pressure-based brake calibration [N/bar] for correlation replay
        maxSpeed      = 80       % Soft speed limiter [m/s] (~288 km/h)
        unsprungMass  = 25       % Per-corner unsprung mass [kg]

        % ----------------------------------------------------------------
        % Sub-systems (nested structs, initialized in the constructor)
        % ----------------------------------------------------------------
        aero          % Whole-car aero map (downforce/drag + center of pressure)

        suspension    % Springs, damping, ARBs, bump stops, kinematic geometry
        chassis       % Sprung-mass heave/pitch/roll platform stiffness & damping

        powertrain    % Motor map, drivetrain efficiency, differential type
        tire          % Pacejka .tir data + wheel dynamics (inertia, relaxation, drag)
        correlation   % Correlation replay scenario overrides
    end

    methods
        function obj = VehicleConfig()
            % VEHICLECONFIG Construct with baseline defaults.
            %   Every sub-system struct is pre-populated so individual
            %   fields can be overridden later (cfg.aero.ClA = 4.2).
            obj.name = "VehicleConfig";

            % --- Aero: whole car ---
            %   The aero resultant is positioned at (xPosition, zPosition) where
            %   xPosition > 0 is forward of CG, < 0 is behind. The component
            %   produces downforce F = 0.5*rho*ClA*V^2 and drag F = 0.5*rho*CdA*V^2.
            %   xPosition is the center of pressure relative to the CG. A
            %   front aero balance f maps to xPosition = wheelbase*(f-staticFrontWeight).
            %   zPosition is kept at CG height by default so drag adds no
            %   artificial pitch moment.
            obj.aero = struct( ...
                'xPosition', -0.084146, ...
                'zPosition', obj.cgHeight, ...
                'ClA', 4.10, ...
                'CdA', 1.60, ...
                'pitchSensitivityClA', 0.0);

            % --- Suspension ---
            %   front/rear (shared within an axle):
            %     springRate   [N/m] heave spring (wheel rate = springRate*MR^2)
            %     dampingCoeff [N*s/m] low-speed compression (bump) slope
            %     reboundCoeff [N*s/m] low-speed rebound (droop) slope
            %   motionRatio     [-] installation MR (wheel<->spring); wheel rate scales by MR^2
            %   bumpStopLength  [m] free travel before the bump stop engages
            %   bumpStopRate    [N/m] bump-stop stiffness
            %   tireSpringRate  [N/m] vertical tire stiffness (quarter-car)
            %   dampingKneeSpeed      [m/s] wheel-domain shaft speed at which the
            %     damper slope breaks from low-speed to high-speed. Inf => linear.
            %   dampingHighSpeedRatio [-] high-speed damper slope / low-speed slope.
            %     1.0 => linear damper. Typical racing dampers ~0.2-0.3; the
            %     default 0.3 brings an over-damped low-speed setup (~300-400%
            %     critical) close to critically damped at high shaft speed.
            %   geometry: suspension/steering kinematics (per-axle curves +
            %             steering model), see the geometry block below
            %   frontArb/rearArb: torsional stiffness [N*m/rad], motionRatio,
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
                'dampingKneeSpeed', 0.05, ...
                'dampingHighSpeedRatio', 0.3, ...
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
            %     rollCenterLateral [m] signed lateral roll-center position
            %                          at +1g; dynamics scale it by current
            %                          axle lateral acceleration.
            %     casterAngle [rad] positive when steering-axis top leans rearward
            %     mechanicalTrail [m] positive contact patch behind kingpin ground point
            %     scrubRadius [m] positive contact patch outboard of kingpin ground point
            %     kingpinInclination [rad] positive when steering-axis top leans inward
            %     kingpinOffset [m] alias/fallback for scrubRadius when scrub is omitted
            %   Steering model: steerInput is treated as a road-wheel angle.
            %     ackermann: 0 = parallel, 1 = ideal, >1 = over-Ackermann.
            %   (Vehicle-level wheelbase/track/weight are pulled from the
            %    lts.vehicle.VehicleManager at construction, not duplicated here.)
            obj.suspension.geometry = struct( ...
                'front', struct( ...
                    'travelGrid',       [-0.05 0 0.05], ...
                    'camberCurve',      [0.5 0 -1.5] * pi / 180, ...
                    'toeCurve',         [-0.05 0 0.05] * pi / 180, ...
                    'motionRatioCurve', [0.93 0.95 0.97], ...
                    'rollCenterHeight', 0.030, ...
                    'rollCenterLateral', 0, ...
                    'casterAngle',      7.0 * pi / 180, ...
                    'mechanicalTrail',  0.030, ...
                    'scrubRadius',      0.018, ...
                    'kingpinInclination', 8.0 * pi / 180, ...
                    'kingpinOffset',    0.018), ...
                'rear', struct( ...
                    'travelGrid',       [-0.05 0 0.05], ...
                    'camberCurve',      [0.25 0 -0.8] * pi / 180, ...
                    'toeCurve',         [0.05 0 -0.05] * pi / 180, ...
                    'motionRatioCurve', [0.94 0.95 0.96], ...
                    'rollCenterHeight', 0.045, ...
                    'rollCenterLateral', 0, ...
                    'casterAngle',      0, ...
                    'mechanicalTrail',  0, ...
                    'scrubRadius',      0, ...
                    'kingpinInclination', 0, ...
                    'kingpinOffset',    0), ...
                'steering', struct( ...
                    'steeringRatio',      1.0, ...
                    'ackermann',          0.8872, ...
                    'maxWheelSteerAngle', 0.6, ...
                'rearSteerRatio',     0.0));

            % --- Brakes ---
            %   Ratio mode uses brakeBiasFront + brakeForceCoefficient.
            %   Correlation pressure mode uses front/rear line pressures:
            %     frontForcePerBar / rearForcePerBar [N/bar] are total axle
            %     longitudinal brake force magnitudes before tire slip limits.
            obj.brakePressure = struct( ...
                'frontForcePerBar', NaN, ...
                'rearForcePerBar', NaN);

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
            %     finalDriveRatio [ratio] optional override for the map FDR
            %     efficiency [0-1] motoring drivetrain efficiency
            %       (gearbox + bearings, or correlation scalar)
            %     efficiencyRpm / efficiencyValues optional equal-length
            %       vectors defining a bounded RPM-dependent motoring curve;
            %       empty vectors preserve the scalar efficiency
            %     regenEfficiency [0-1] optional direct-mode regen drivetrain
            %       efficiency; NaN uses efficiency
            %     deliveredTorqueDrivetrainEfficiency [0-1] mechanical-only
            %       loss from measured motor shaft torque to the driven axle;
            %       NaN uses efficiency for backward compatibility
            %     motorRotorInertia [kg*m^2] motor rotor inertia, reflected as
            %       I*ratio^2 onto the driven (rear) wheels (default 0.07).
            %     regenEnabled (false) opt-in regenerative braking at off-throttle
            %     motoringDragTorque [Nm] (0) opt-in motor coastdown drag
            %     motoringDragThrottleThreshold [-] throttle at/below which
            %       motoring drag applies; Inf preserves always-on drag when set
            %     regenTorqueLimitNm [Nm] (30) max regen torque, motor-side
            %     regenEnabledSpeedFloor [m/s] speed below which regen tapers
            %     throttleDeadband [-] pedal command below which drive torque is 0
            %     throttleMapInput / throttleMapOutput shape post-deadband
            %       pedal into motor torque/current request fraction.
            %     differential.type: 'open' | 'locked' (spool) | 'lsd' | 'drexler'
            %       'lsd' may carry preload [N*m], ramp [-], speedGain [-],
            %       biasRatio [-] (torque-bias cap).
            %       'drexler' is a signed ramp-plate LSD with accel/decel
            %       ramp angles [deg], preloadBreakawayTorqueNm [N*m],
            %       rampTorqueScale [-], and fluid metadata. It requires
            %       finite calibrated preload/ramp values before simulation.
            %   Drivetrain is RWD (drive torque only on the rear axle).
            obj.powertrain = struct( ...
                'matFile', '', ...
                'finalDriveRatio', NaN, ...
                'efficiency', 0.92, ...
                'efficiencyRpm', [], ...
                'efficiencyValues', [], ...
                'regenEfficiency', NaN, ...
                'deliveredTorqueDrivetrainEfficiency', NaN, ...
                'motorRotorInertia', 0.07, ...
                'regenEnabled', false, ...
                'motoringDragTorque', 0, ...
                'motoringDragThrottleThreshold', Inf, ...
                'regenTorqueLimitNm', 30, ...
                'regenEnabledSpeedFloor', 1.0, ...
                'throttleDeadband', 0, ...
                'throttleMapInput', [0.00 0.15 0.35 0.60 0.80 1.00], ...
                'throttleMapOutput', [0.00 0.02 0.10 0.28 0.58 1.00], ...
                'differential', struct('type', 'open'));

            % --- Tire ---
            %   Pacejka Magic Formula (MF 6.1) via MFeval. Grip is set entirely
            %   by the tire data (including its load sensitivity). Track
            %   surface-friction scaling is intentionally unsupported.
            %   tirFile lives in +Tire/.
            %     tirFile:  Pacejka .tir filename in +Tire/
            %     wheelInertia [kg*m^2] wheel+tire+brake rotating inertia/corner
            %     relaxationLength [m] lateral contact-patch slip lag
            %       (0 = steady-state)
            %     longitudinalRelaxationLength [m] longitudinal slip-ratio
            %       lag; NaN uses relaxationLength for backward compatibility
            %     normalLoadRelaxationLength [m] contact-patch load-response
            %       lag on the Fz the Magic Formula sees (0 = off); dual
            %       role: at-speed loop damping + low-speed launch smoothing
            %     lateralStiffnessScale [-] multiplier on tire slip angle for
            %       correlation sensitivity (1 preserves raw tire file)
            %     lateralStiffnessScaleByCorner [-] optional [FL FR RL RR]
            %       multipliers applied after lateralStiffnessScale
            %     wheelRadius [m] effective rolling radius
            %     rollingResistanceCoeff [-] Crr; per-wheel resistance torque
            %       T_rr = Crr*Fz*R (0 disables coast-down drag)
            %     bearingDragCoeff [N*m*s/rad] viscous wheel-hub drag
            %       T_b = coeff*omega (0 = off by default)
            obj.tire = struct( ...
                'tirFile', '43105_18x7.5_10_R25B_7.tir', ...
                'wheelInertia', 0.5, ...
                'relaxationLength', 0.30, ...
                'longitudinalRelaxationLength', NaN, ...
                'lateralStiffnessScale', 1.0, ...
                'lateralStiffnessScaleByCorner', [1 1 1 1], ...
                'wheelRadius', 0.241935, ...
                'rollingResistanceCoeff', 0.015, ...
                'bearingDragCoeff', 0);

            % --- Correlation replay ---
            % Scenario-level overrides used only by lts.app.run_correlation.
            % surfaceMu is retained for compatibility and has no physics effect.
            % initialTransientWindowS = 0 uses the first logged sample;
            % positive values fit a local boundary trend over that many seconds.
            % Steering calibration is identity by default. A center gain other
            % than one applies an odd quadratic transfer curve that remains
            % exact at steeringCalibrationEndAngleRad.
            obj.correlation = struct( ...
                'surfaceMu', 1.0, ...
                'useLoggedYawRate', true, ...
                'useLoggedTransientState', true, ...
                'initialTransientWindowS', 0, ...
                'steeringCenterGain', 1.0, ...
                'steeringCenterOffsetRad', 0, ...
                'steeringCalibrationEndAngleRad', deg2rad(22), ...
                'steeringDelayS', 0);
        end
    end

    methods (Static)
        function config = validate(config)
            % VALIDATE Sanity-check the vehicle-level scalars of a config.
            %   config = lts.vehicle.VehicleConfig.validate(config)
            %   Catches obvious typos (totalMass=0, negative wheelbase, a
            %   staticFrontWeight outside [0,1], ...) at the build boundary
            %   instead of letting them surface deep in simulation as a
            %   divide-by-zero or NaN. Sub-system structs (aero/suspension/
            %   tire/...) are validated by their own builders. Returns the
            %   config unchanged on success; errors with typed identifiers.
            checks = struct( ...
                'totalMass',     struct('min', eps,     'max', Inf), ...
                'wheelbase',     struct('min', eps,     'max', Inf), ...
                'trackWidth',    struct('min', eps,     'max', Inf), ...
                'cgHeight',      struct('min', 0,       'max', Inf), ...
                'yawInertia',    struct('min', eps,     'max', Inf), ...
                'airDensity',    struct('min', 0,       'max', Inf), ...
                'unsprungMass',  struct('min', 0,       'max', Inf), ...
                'maxSpeed',      struct('min', 0,       'max', Inf));
            fields = fieldnames(checks);
            for i = 1:numel(fields)
                f = fields{i};
                value = config.(f);
                if ~isscalar(value) || ~isnumeric(value) || ~isfinite(value)
                    error('lts_vehicle_VehicleConfig:InvalidScalar', ...
                        'VehicleConfig.%s must be a finite scalar (got %s).', ...
                        f, dispValue(value));
                end
                if value < checks.(f).min || value > checks.(f).max
                    error('lts_vehicle_VehicleConfig:OutOfRange', ...
                        'VehicleConfig.%s=%g is outside [%g, %g].', ...
                        f, value, checks.(f).min, checks.(f).max);
                end
            end
            % Distribution/bias fractions must be physical probabilities.
            fractions = struct('staticFrontWeight', 'front weight distribution', ...
                'brakeBiasFront', 'brake bias', ...
                'brakeForceCoefficient', 'brake force coefficient');
            fNames = fieldnames(fractions);
            for i = 1:numel(fNames)
                f = fNames{i};
                value = config.(f);
                if ~isscalar(value) || ~isnumeric(value) || ~isfinite(value) || ...
                        value < 0 || value > 1
                    error('lts_vehicle_VehicleConfig:InvalidFraction', ...
                        'VehicleConfig.%s (%s) must be a finite scalar in [0, 1] (got %s).', ...
                        f, fractions.(f), dispValue(value));
                end
            end

            % The chassis attitude model is a sprung-mass model. Do not let
            % the builder silently clamp an impossible mass breakdown to eps.
            sprungMass = config.totalMass - 4 * config.unsprungMass;
            if sprungMass <= 0
                error('lts_vehicle_VehicleConfig:InvalidMassBreakdown', ...
                    ['totalMass (%g kg) must exceed four times unsprungMass ' ...
                    '(%g kg/corner).'], config.totalMass, config.unsprungMass);
            end

            % +Inf is a supported exact rigid-torsion constraint. Other
            % non-finite and all negative values are invalid.
            torsion = config.chassis.torsionalRigidity;
            if ~isnumeric(torsion) || ~isreal(torsion) || ~isscalar(torsion) || ...
                    isnan(torsion) || torsion < 0 || torsion == -Inf
                error('lts_vehicle_VehicleConfig:InvalidTorsionalRigidity', ...
                    ['VehicleConfig.chassis.torsionalRigidity must be a ' ...
                    'nonnegative real scalar or +Inf (got %s).'], ...
                    dispValue(torsion));
            end
        end
    end
end

function s = dispValue(v)
if isnumeric(v)
    s = mat2str(v);
else
    s = char(string(v));
end
end
