classdef VehicleManager
    % VEHICLEMANAGER Configuration container for vehicle components and parameters
    % Holds references to all swappable components (aero, chassis, suspension, powertrain, tire, track)
    % and all non-changing vehicle parameters (mass, wheelbase, track width, etc).
    %
    % Simulation is handled by the lts.simulation.Simulator class, driver decisions by lts.driver.DriverModel.
    
    properties (Constant)
        % Gravitational acceleration [m/s^2]. Still the single source of
        % truth for all physics (weight, load transfer, g-display), but
        % defined once in the shared kernel so component repositories read
        % the same constant without depending on this class. Display
        % scripts still tolerate the legacy 9.81 literal.
        g = lts.util.PhysicalConstants.g
    end

    properties
        % Swappable component objects
        aero        % lts.components.Aero.AeroComponent
        chassis     % lts.components.Chassis.ChassisComponent
        suspension  % lts.components.Suspension.SuspensionManager
        powertrain  % lts.components.PowertrainComponent
        tire        % lts.components.Tire.TireModel
        track       % lts.components.Track

        % Driven-axle differential. Default OpenDifferential (set by
        % lts.app.run_simulation); may be locked, legacy clutch LSD, or Drexler ramp LSD.
        differential  % lts.components.Powertrain.DifferentialComponent
        
        % Vehicle parameters
        totalMass     = 256      % Total mass with driver [kg]
        wheelbase     = 1.558     % Wheelbase [m]
        trackWidth    = 1.21      % Track width [m]
        cgHeight      = 0.3     % CG height [m]
        yawInertia    = 130      % Yaw inertia about CG [kg*m^2]
        airDensity    = 1.225    % Air density [kg/m^3]
        staticFrontWeight = 0.50 % Static front weight distribution [0-1]
        brakeBiasFront = 0.60    % Fraction of brake force commanded to front axle [0-1]
        brakeForceCoefficient = 0.70 % Hydraulic brake force capacity as fraction of normal load
        brakePressureFrontForcePerBar = NaN % Front axle brake force per line pressure [N/bar]
        brakePressureRearForcePerBar = NaN  % Rear axle brake force per line pressure [N/bar]
        
        % Simulation parameters
        maxSpeed      = 80       % Speed limiter [m/s] (~288 km/h)
    end
    
    methods
        function obj = VehicleManager(aero, suspension, powertrain, tire, track, chassis, differential)
            % VEHICLEMANAGER Construct with all component objects
            %   lts.vehicle.VehicleManager(aero, suspension, powertrain, tire, track)
            %   lts.vehicle.VehicleManager(aero, suspension, powertrain, tire, track, chassis)
            %   lts.vehicle.VehicleManager(aero, suspension, powertrain, tire, track, chassis, differential)

            obj.aero = aero;
            obj.suspension = suspension;
            obj.powertrain = powertrain;
            obj.tire = tire;
            obj.track = track;
            if nargin >= 6
                obj.chassis = chassis;
            end
            if nargin >= 7
                obj.differential = differential;
            end
        end
    end

    methods (Static)
        function vehicle = fromConfig(config, track, dt, varargin)
            % FROMCONFIG Build a fully-wired lts.vehicle.VehicleManager from a lts.vehicle.VehicleConfig
            %
            %   vehicle = lts.vehicle.VehicleManager.fromConfig(config, track)
            %   vehicle = lts.vehicle.VehicleManager.fromConfig(config, track, dt)
            %   vehicle = lts.vehicle.VehicleManager.fromConfig(config, track, dt, 'Verbose', false)
            %
            %   config - lts.vehicle.VehicleConfig describing the car (see lts.vehicles.baseline)
            %   track  - lts.components.Track (test/waypoint track to run)
            %   dt     - timestep [s] for suspension/chassis warmup (default 0.001)
            %   Verbose - print the component inventory (default true). Set to
            %             false in sweeps so the build stays quiet without
            %             resorting to evalc() around this call.
            %
            %   Constructs every subsystem from the config, preserving the
            %   order dependencies and warmup steps of the original
            %   lts.app.run_simulation setup, and returns a lts.vehicle.VehicleManager ready to
            %   hand to lts.driver.DriverModel + lts.simulation.Simulator.
            %
            % Build-order physics dependency:
            %   aero/powertrain/tire are independent,
            %   lts.vehicle.VehicleManager supplies global mass/geometry,
            %   suspension geometry needs that mass/geometry,
            %   chassis needs the suspension roll stiffness,
            %   differential is attached last for rear-axle torque splitting.

            if nargin < 3 || isempty(dt)
                dt = 0.001;
            end
            p = inputParser;
            p.addParameter('Verbose', true, ...
                @(x) islogical(x) || (isnumeric(x) && isscalar(x)));
            p.parse(varargin{:});
            verbose = logical(p.Results.Verbose);

            % Catch obvious config typos (totalMass=0, negative wheelbase,
            % staticFrontWeight outside [0,1], ...) at the build boundary
            % rather than surfacing deep in simulation as NaN/divide-by-zero.
            config = lts.vehicle.VehicleConfig.validate(config);

            % Contract item 2 of the repository split: each component
            % repository owns and validates its cfg sub-struct schema
            % (lts.components.<Pkg>.validateConfig). Typed errors name
            % the offending field; see the Contracts documentation page.
            lts.components.Aero.validateConfig(config.aero);
            lts.components.Suspension.validateConfig(config.suspension);
            lts.components.Chassis.validateConfig(config.chassis);
            lts.components.Powertrain.validateConfig(config.powertrain);

            if verbose
                fprintf('=== Building vehicle from config ===\n\n');
            end

            %% ---- Aero ----
            aeroCfg = config.aero;
            aero = lts.components.Aero.WholeCarAero( ...
                aeroCfg.xPosition, aeroCfg.zPosition, aeroCfg.ClA, aeroCfg.CdA, ...
                lts.util.fieldOr(aeroCfg, 'pitchSensitivityClA', 0));
            if verbose
                fprintf('Aero: WholeCarAero | x=%.2f m, z=%.2f m, ClA=%.2f, CdA=%.2f\n', ...
                    aero.xPosition, aero.zPosition, aero.ClA, aero.CdA);
                fprintf('  Front aero balance at zero pitch: %.1f%%\n', ...
                    100 * (config.wheelbase * config.staticFrontWeight + ...
                    aero.xPosition) / config.wheelbase);
                fprintf('\n');
            end

            %% ---- Powertrain ----
            powertrain = lts.components.Powertrain.EMRAX228Powertrain( ...
                config.powertrain.matFile, config.powertrain.efficiency, ...
                lts.util.fieldOr(config.powertrain, 'motorRotorInertia', 0.07));
            powertrain = powertrain.setDrivenWheelRadius(config.tire.wheelRadius);
            powertrain.regenEfficiency = ...
                lts.util.fieldOr(config.powertrain, 'regenEfficiency', NaN);
            powertrain.deliveredTorqueDrivetrainEfficiency = ...
                lts.util.fieldOr(config.powertrain, ...
                'deliveredTorqueDrivetrainEfficiency', NaN);
            powertrain = powertrain.setMotoringEfficiencyCurve( ...
                lts.util.fieldOrDefault(config.powertrain, 'efficiencyRpm', []), ...
                lts.util.fieldOrDefault(config.powertrain, 'efficiencyValues', []));
            finalDriveRatio = lts.util.fieldOr(config.powertrain, 'finalDriveRatio', NaN);
            if isfinite(finalDriveRatio) && finalDriveRatio > 0
                powertrain = powertrain.setFinalDriveRatio(finalDriveRatio);
            end
            % Opt-in coastdown/regen (off by default; baseline unchanged).
            powertrain.regenEnabled = ...
                lts.util.fieldOr(config.powertrain, 'regenEnabled', false);
            powertrain.motoringDragTorque = ...
                lts.util.fieldOr(config.powertrain, 'motoringDragTorque', 0);
            powertrain.motoringDragThrottleThreshold = ...
                lts.util.fieldOr(config.powertrain, 'motoringDragThrottleThreshold', Inf);
            powertrain.throttleDeadband = ...
                lts.util.fieldOr(config.powertrain, 'throttleDeadband', 0);
            powertrain.throttleMapInput = ...
                lts.util.fieldOrDefault(config.powertrain, ...
                'throttleMapInput', powertrain.throttleMapInput);
            powertrain.throttleMapOutput = ...
                lts.util.fieldOrDefault(config.powertrain, ...
                'throttleMapOutput', powertrain.throttleMapOutput);
            if powertrain.regenEnabled
                powertrain.regenTorqueLimitNm = ...
                    lts.util.fieldOr(config.powertrain, 'regenTorqueLimitNm', 30);
                powertrain.regenEnabledSpeedFloor = ...
                    lts.util.fieldOr(config.powertrain, 'regenEnabledSpeedFloor', 1.0);
            end
            if verbose
                fprintf('Powertrain: EMRAX 228 (Tq=%.0f Nm, FDR=%.1f, falloff %.0f->%.0f rpm, factor=%.2f)\n', ...
                    powertrain.maxEngineTorque, powertrain.totalGearRatio, ...
                    powertrain.rpmFalloffStartRPM, powertrain.rpmLimitRPM, ...
                    powertrain.rpmFalloffFactor);
            end

            %% ---- Tire (Pacejka Magic Formula via MFeval) ----
            %  Requires MFeval toolbox:
            %  https://www.mathworks.com/matlabcentral/fileexchange/63618-mfeval
            tire = lts.components.Tire.PacejkaTire(config.tire.tirFile);
            tire.wheelInertia = config.tire.wheelInertia;
            tire.relaxationLength = config.tire.relaxationLength;
            if isfield(config.tire, 'longitudinalRelaxationLength')
                tire.longitudinalRelaxationLength = ...
                    config.tire.longitudinalRelaxationLength;
            end
            if isfield(config.tire, 'normalLoadRelaxationLength')
                tire.normalLoadRelaxationLength = ...
                    config.tire.normalLoadRelaxationLength;
            end
            if isfield(config.tire, 'lateralStiffnessScale')
                tire.lateralStiffnessScale = config.tire.lateralStiffnessScale;
            end
            if isfield(config.tire, 'lateralStiffnessScaleByCorner')
                tire.lateralStiffnessScaleByCorner = ...
                    config.tire.lateralStiffnessScaleByCorner;
            end
            if isfield(config.tire, 'rollingResistanceCoeff')
                tire.rollingResistanceCoeff = config.tire.rollingResistanceCoeff;
            end
            if isfield(config.tire, 'bearingDragCoeff')
                tire.bearingDragCoeff = config.tire.bearingDragCoeff;
            end
            % Effective rolling radius is per-corner state; propagate to all four.
            corners = {tire.FL, tire.FR, tire.RL, tire.RR};
            for k = 1:numel(corners)
                corners{k}.wheelRadius = config.tire.wheelRadius;
            end

            %% ---- lts.vehicle.VehicleManager (constants + core components) ----
            %  Created first so SuspensionManager / SimpleChassis /
            %  SuspensionGeometry can read vehicle-level geometry from it.
            %  Chassis and differential are attached after construction.
            vehicle = lts.vehicle.VehicleManager(aero, [], powertrain, tire, track);
            vehicle.totalMass            = config.totalMass;
            vehicle.wheelbase            = config.wheelbase;
            vehicle.trackWidth           = config.trackWidth;
            vehicle.cgHeight             = config.cgHeight;
            vehicle.yawInertia           = config.yawInertia;
            vehicle.airDensity           = config.airDensity;
            vehicle.staticFrontWeight    = config.staticFrontWeight;
            vehicle.brakeBiasFront       = config.brakeBiasFront;
            vehicle.brakeForceCoefficient = config.brakeForceCoefficient;
            vehicle.brakePressureFrontForcePerBar = ...
                lts.util.fieldOr(config.brakePressure, 'frontForcePerBar', NaN);
            vehicle.brakePressureRearForcePerBar = ...
                lts.util.fieldOr(config.brakePressure, 'rearForcePerBar', NaN);
            vehicle.maxSpeed             = config.maxSpeed;

            %% ---- Suspension geometry + anti-roll bars ----
            suspCfg = config.suspension;
            geometry = lts.components.Suspension.SuspensionGeometry.fromConfig( ...
                suspCfg.geometry, vehicle);
            if verbose
                fprintf('Suspension Geometry: Ackermann %.1f%%\n', ...
                    geometry.ackermann * 100);
            end

            fa = suspCfg.frontArb;
            frontArb = lts.components.Suspension.AntiRollBar( ...
                fa.stiffness, fa.motionRatio, fa.leverArm, fa.enabled);
            ra = suspCfg.rearArb;
            rearArb = lts.components.Suspension.AntiRollBar( ...
                ra.stiffness, ra.motionRatio, ra.leverArm, ra.enabled);
            geometry.frontAntiRollBar = frontArb;
            geometry.rearAntiRollBar = rearArb;
            if verbose
                fprintf('Anti-Roll Bars: front=%.0f N/m, rear=%.0f N/m at the wheel\n', ...
                    frontArb.getWheelRateStiffness(), rearArb.getWheelRateStiffness());
            end

            %% ---- Suspension (needs vehicleManager for geometry) ----
            %  The front/rear elastic load-transfer split is derived from
            %  springs + ARBs unless rollStiffnessOverride is set (below).
            %  dampingKneeSpeed/dampingHighSpeedRatio default to a linear
            %  damper when the config omits them (fieldOr fallback).
            dampingKnee = lts.util.fieldOr(suspCfg, 'dampingKneeSpeed', Inf);
            dampingRatio = lts.util.fieldOr(suspCfg, 'dampingHighSpeedRatio', 1.0);
            dampingReboundKnee = lts.util.fieldOr( ...
                suspCfg, 'dampingReboundKneeSpeed', NaN);
            suspension = lts.components.Suspension.SuspensionManager( ...
                vehicle, ...
                suspCfg.front.springRate, suspCfg.front.dampingCoeff, suspCfg.front.reboundCoeff, ...
                suspCfg.rear.springRate,  suspCfg.rear.dampingCoeff,  suspCfg.rear.reboundCoeff, ...
                suspCfg.motionRatio, ...
                suspCfg.bumpStopLength, ...
                suspCfg.bumpStopRate, ...
                suspCfg.tireSpringRate, ...
                config.unsprungMass, ...
                geometry, ...
                [], [], ...
                dampingKnee, dampingRatio, dampingReboundKnee);
            suspension.rollStiffnessOverride = suspCfg.rollStiffnessOverride;
            suspension.coupleChassisRollToLoadTransfer = suspCfg.coupleChassisRollToLoadTransfer;
            vehicle.suspension = suspension;
            if verbose
                fprintf(['Suspension: SuspensionManager ' ...
                    '(4-corner transient + geometry + ARB F/R %.0f/%.0f N/m at the wheel)\n'], ...
                    frontArb.getWheelRateStiffness(), rearArb.getWheelRateStiffness());
            end

            % Warm up suspension to static equilibrium (avoids a startup transient).
            suspension.warmup(vehicle.totalMass, dt);

            %% ---- Chassis attitude model (heave/pitch/roll DOFs) ----
            % VehicleConfig.validate guarantees this is strictly positive;
            % do not mask an invalid mass breakdown with an epsilon clamp.
            sprungMass = config.totalMass - 4 * config.unsprungMass;
            chassis = lts.components.Chassis.SimpleChassis(vehicle, sprungMass);
            % Apply legacy standalone fallbacks. Once the suspension is
            % linked below, SimpleChassis bypasses these six coefficients and
            % reacts against the actual corner spring/damper/bump-stop/ARB
            % forces. Torsional rigidity/damping remain chassis properties.
            chassis.heaveStiffness     = config.chassis.heaveStiffness;
            chassis.heaveDamping       = config.chassis.heaveDamping;
            chassis.pitchStiffness     = config.chassis.pitchStiffness;
            chassis.pitchDamping       = config.chassis.pitchDamping;
            chassis.rollStiffness      = config.chassis.rollStiffness;
            chassis.rollDamping        = config.chassis.rollDamping;
            chassis.torsionalRigidity  = config.chassis.torsionalRigidity;
            chassis.torsionalDamping   = config.chassis.torsionalDamping;
            chassis.reset();
            % Settle to static equilibrium over a few steps.
            for warm = 1:5
                chassis.updateFromAccelerations(0, 0, ...
                    struct('Fz_front', 0, 'Fz_rear', 0), dt);
            end
            % Link chassis <-> suspension so both roll models share stiffness.
            chassis = chassis.setSuspension(suspension);
            suspension.chassis = chassis;
            vehicle.suspension = suspension;
            vehicle.chassis = chassis;
            if verbose
                fprintf('Chassis: SimpleChassis (heave/pitch/roll DOF, torsional rigidity %.0f N*m/deg)\n', ...
                    chassis.torsionalRigidity * pi / 180);
            end

            %% ---- Differential (driven/rear axle) ----
            diffType = config.powertrain.differential.type;
            switch lower(diffType)
                case 'open'
                    differential = lts.components.Powertrain.OpenDifferential();
                case 'locked'
                    differential = lts.components.Powertrain.LockedDifferential();
                case {'lsd', 'clutchlsd'}
                    d = config.powertrain.differential;
                    differential = lts.components.Powertrain.ClutchLSDDifferential( ...
                        'preload',   lts.util.fieldOr(d, 'preload', 20), ...
                        'ramp',      lts.util.fieldOr(d, 'ramp', 0.5), ...
                        'speedGain', lts.util.fieldOr(d, 'speedGain', 2.0), ...
                        'biasRatio', lts.util.fieldOr(d, 'biasRatio', 2.0), ...
                        'relativeSpeedDamping', ...
                            lts.util.fieldOr(d, 'relativeSpeedDamping', 0.5));
                case {'drexler', 'drexlerrampplate', 'rampplate'}
                    d = config.powertrain.differential;
                    differential = lts.components.Powertrain.DrexlerRampPlateDifferential( ...
                        'accelRampAngleDeg', lts.util.fieldOr(d, 'accelRampAngleDeg', 30), ...
                        'decelRampAngleDeg', lts.util.fieldOr(d, 'decelRampAngleDeg', 45), ...
                        'preloadBreakawayTorqueNm', ...
                            lts.util.fieldOr(d, 'preloadBreakawayTorqueNm', NaN), ...
                        'rampTorqueScale', lts.util.fieldOr(d, 'rampTorqueScale', NaN), ...
                        'fluid', lts.util.fieldOr(d, 'fluid', "Motul Gear Competition 75W-140"), ...
                        'slipSmoothingRadPerSec', ...
                            lts.util.fieldOr(d, 'slipSmoothingRadPerSec', 0), ...
                        'relativeSpeedDamping', ...
                            lts.util.fieldOr(d, 'relativeSpeedDamping', 0.5));
                    differential.validateCalibration();
                otherwise
                    error('lts_vehicle_VehicleManager:UnknownDifferential', ...
                        'Unknown differential type "%s".', diffType);
            end
            vehicle.differential = differential;
            if verbose
                fprintf('Differential: %s\n', differential.getName());

                fprintf('\n');
            end
        end

    end
end
