classdef SimpleChassis < components.Chassis.ChassisComponent
    % SIMPLECHASSIS Lumped sprung-mass heave/pitch/roll chassis model
    %
    % The simulator uses this sprung-mass attitude state to drive suspension
    % corner loads. Aero, longitudinal acceleration, and lateral acceleration
    % move the body; the suspension then converts that heave/pitch/roll motion
    % into tire normal loads through the actual spring and damper rates.

    properties
        state  % components.Chassis.ChassisState

        % Vehicle geometry/mass
        totalMass = 280
        sprungMass = 280
        autoSprungMass = true
        wheelbase = 1.55
        trackWidth = 1.2
        cgHeight = 0.28
        staticFrontWeight = 0.45

        % Lumped inertias [kg*m^2]
        pitchInertia = 60
        rollInertia = 40
        autoPitchInertia = true
        autoRollInertia = true

        % Linear platform stiffness/damping from static equilibrium
        heaveStiffness = 160000   % [N/m]
        heaveDamping = 12000      % [N*s/m]
        pitchStiffness = 90000    % [N*m/rad]
        pitchDamping = 6000       % [N*m*s/rad]
        rollStiffness = 55000     % [N*m/rad]
        rollDamping = 5000        % [N*m*s/rad]
    end

    methods
        function obj = SimpleChassis(vehicleManager, sprungMass, pitchInertia, rollInertia)
            % SIMPLECHASSIS Construct from VehicleManager geometry
            %   SimpleChassis(vehicleManager)
            %   SimpleChassis(vehicleManager, sprungMass, pitchInertia, rollInertia)
            if nargin >= 1 && ~isempty(vehicleManager)
                obj.totalMass = vehicleManager.totalMass;
                obj.sprungMass = obj.totalMass;
                obj.wheelbase = vehicleManager.wheelbase;
                obj.trackWidth = vehicleManager.trackWidth;
                obj.cgHeight = vehicleManager.cgHeight;
                obj.staticFrontWeight = vehicleManager.staticFrontWeight;
            end
            if nargin >= 2 && ~isempty(sprungMass) ...
                    && utils.isPositiveScalar(sprungMass)
                obj.sprungMass = sprungMass;
                obj.autoSprungMass = false;
            else
                obj.autoSprungMass = true;
            end
            if nargin >= 3 && ~isempty(pitchInertia) ...
                    && utils.isPositiveScalar(pitchInertia)
                obj.pitchInertia = pitchInertia;
                obj.autoPitchInertia = false;
            else
                obj.pitchInertia = obj.computeAutoPitchInertia();
                obj.autoPitchInertia = true;
            end
            if nargin >= 4 && ~isempty(rollInertia) ...
                    && utils.isPositiveScalar(rollInertia)
                obj.rollInertia = rollInertia;
                obj.autoRollInertia = false;
            else
                obj.rollInertia = obj.computeAutoRollInertia();
                obj.autoRollInertia = true;
            end

            obj.state = components.Chassis.ChassisState();
            obj.state.updateCornerKinematics( ...
                obj.wheelbase, obj.trackWidth, obj.staticFrontWeight);
        end

        function reset(obj)
            obj.state.reset();
            obj.state.updateCornerKinematics( ...
                obj.wheelbase, obj.trackWidth, obj.staticFrontWeight);
        end

        function syncVehicleGeometry(obj, vehicleManager)
            % SYNCVEHICLEGEOMETRY Refresh copied geometry from VehicleManager.
            %
            % The chassis stores lever arms locally because they are used many
            % times per step. If the VehicleManager is retuned after component
            % construction, stale CG height, wheelbase, or track width would
            % make load-transfer moments disagree with the tire and driver
            % models. Keep those local copies synchronized at the simulator
            % step boundary.
            if nargin < 2 || isempty(vehicleManager)
                return;
            end

            obj.totalMass = vehicleManager.totalMass;
            if obj.autoSprungMass
                obj.sprungMass = obj.totalMass;
            end
            obj.wheelbase = vehicleManager.wheelbase;
            obj.trackWidth = vehicleManager.trackWidth;
            obj.cgHeight = vehicleManager.cgHeight;
            obj.staticFrontWeight = vehicleManager.staticFrontWeight;

            if obj.autoPitchInertia
                obj.pitchInertia = obj.computeAutoPitchInertia();
            end
            if obj.autoRollInertia
                obj.rollInertia = obj.computeAutoRollInertia();
            end

            obj.state.updateCornerKinematics( ...
                obj.wheelbase, obj.trackWidth, obj.staticFrontWeight);
        end

        function syncSuspensionRates(obj, suspension)
            % SYNCSUSPENSIONRATES Derive platform rates from corner hardware.
            %
            % The chassis equations are written in heave/pitch/roll coordinates,
            % while the suspension is configured as four corner spring-damper
            % units. Summing each corner's wheel rate through its lever arm keeps
            % body motion, tire normal load, and plotted suspension travel tied
            % to the same physical parameters.
            if nargin < 2 || isempty(suspension)
                return;
            end

            K_FL = obj.computeCornerWheelRate(suspension.frontLeft);
            K_FR = obj.computeCornerWheelRate(suspension.frontRight);
            K_RL = obj.computeCornerWheelRate(suspension.rearLeft);
            K_RR = obj.computeCornerWheelRate(suspension.rearRight);

            C_FL = obj.computeCornerDamping(suspension.frontLeft);
            C_FR = obj.computeCornerDamping(suspension.frontRight);
            C_RL = obj.computeCornerDamping(suspension.rearLeft);
            C_RR = obj.computeCornerDamping(suspension.rearRight);

            frontArm = obj.wheelbase * (1 - obj.staticFrontWeight);
            rearArm = obj.wheelbase * obj.staticFrontWeight;
            halfTrack = obj.trackWidth / 2;

            obj.heaveStiffness = K_FL + K_FR + K_RL + K_RR;
            obj.heaveDamping = C_FL + C_FR + C_RL + C_RR;

            obj.pitchStiffness = (K_FL + K_FR) * frontArm^2 ...
                + (K_RL + K_RR) * rearArm^2;
            obj.pitchDamping = (C_FL + C_FR) * frontArm^2 ...
                + (C_RL + C_RR) * rearArm^2;

            obj.rollStiffness = (K_FL + K_FR + K_RL + K_RR) * halfTrack^2;
            if ismethod(suspension, 'computeRollStiffnessContributions')
                % SuspensionManager may add anti-roll-bar stiffness to achieve
                % the configured front/rear roll stiffness distribution. The
                % chassis roll equation and corner-load path must use the same
                % elastic roll rate or roll angle and tire normal loads will
                % disagree on the resisting moment.
                [frontRollStiffness, rearRollStiffness] = ...
                    suspension.computeRollStiffnessContributions();
                obj.rollStiffness = frontRollStiffness + rearRollStiffness;
            end
            obj.rollDamping = (C_FL + C_FR + C_RL + C_RR) * halfTrack^2;
        end

        function updateFromAccelerations(obj, ax, ay, aeroForces, dt, ...
                longitudinalGroundForce, lateralGroundForce)
            % UPDATEFROMACCELERATIONS Integrate heave, pitch, and roll
            % ax > 0 creates nose-up pitch. ay > 0 creates right-side-down roll.
            %
            % Pitch is driven by moments about the CG, not by longitudinal
            % acceleration alone. Tire and rolling-resistance forces act at the
            % ground plane, while aero drag acts at its own height above or below
            % the CG. Keeping those moment arms separate avoids assigning drag
            % load transfer to the contact patches.
            %
            % Roll follows the same convention: tire lateral force acts at the
            % ground plane, while aero side drag acts wherever the aero element
            % sits relative to the CG. Total lateral acceleration is still used
            % for VehicleState, but it is not a valid roll moment by itself when
            % aero side force is present.
            if nargin < 4 || isempty(aeroForces)
                aeroForces = struct('Fz_front', 0, 'Fz_rear', 0);
            end
            if nargin < 6 || isempty(longitudinalGroundForce)
                longitudinalGroundForce = obj.sprungMass * ax;
            end
            if nargin < 7 || isempty(lateralGroundForce)
                lateralGroundForce = obj.totalMass * ay ...
                    - utils.getStructField(aeroForces, 'F_side', 0);
            end

            FzFront = utils.getStructField(aeroForces, 'Fz_front', 0);
            FzRear = utils.getStructField(aeroForces, 'Fz_rear', 0);
            F_drag = utils.getStructField(aeroForces, 'F_drag', 0);
            dragHeight = utils.getStructField(aeroForces, 'dragHeight', 0);
            aeroRollMoment = utils.getStructField(aeroForces, 'aeroRollMoment', 0);

            frontArm = obj.wheelbase * (1 - obj.staticFrontWeight);
            rearArm = obj.wheelbase * obj.staticFrontWeight;
            aeroVerticalPitchMoment = FzRear * rearArm - FzFront * frontArm;
            % Positive pitch is nose-up. A backward drag force above the CG
            % produces the same nose-up pitch sign as a forward tire force at
            % the ground plane; a drag resultant below the CG produces nose-down.
            dragPitchMoment = F_drag * dragHeight;
            groundLongitudinalPitchMoment = longitudinalGroundForce * obj.cgHeight;
            groundLateralRollMoment = lateralGroundForce * obj.cgHeight;
            aeroPitchMoment = aeroVerticalPitchMoment + dragPitchMoment;
            sprungMass = utils.positiveScalarOrDefault( ...
                obj.sprungMass, max(obj.totalMass, 1));
            pitchInertia = utils.positiveScalarOrDefault( ...
                obj.pitchInertia, obj.computeAutoPitchInertia());
            rollInertia = utils.positiveScalarOrDefault( ...
                obj.rollInertia, obj.computeAutoRollInertia());

            heaveForce = FzFront + FzRear ...
                - obj.heaveStiffness * obj.state.heave ...
                - obj.heaveDamping * obj.state.heaveRate;

            pitchMoment = groundLongitudinalPitchMoment + aeroPitchMoment ...
                - obj.pitchStiffness * obj.state.pitchAngle ...
                - obj.pitchDamping * obj.state.pitchRate;

            rollMoment = groundLateralRollMoment + aeroRollMoment ...
                - obj.rollStiffness * obj.state.rollAngle ...
                - obj.rollDamping * obj.state.rollRate;

            obj.state.heaveAccel = heaveForce / sprungMass;
            obj.state.pitchAccel = pitchMoment / pitchInertia;
            obj.state.rollAccel = rollMoment / rollInertia;

            obj.state.heaveRate = obj.state.heaveRate + obj.state.heaveAccel * dt;
            obj.state.pitchRate = obj.state.pitchRate + obj.state.pitchAccel * dt;
            obj.state.rollRate = obj.state.rollRate + obj.state.rollAccel * dt;

            obj.state.heave = obj.state.heave + obj.state.heaveRate * dt;
            obj.state.pitchAngle = obj.state.pitchAngle + obj.state.pitchRate * dt;
            obj.state.rollAngle = obj.state.rollAngle + obj.state.rollRate * dt;

            obj.state.longitudinalLoadTransfer = ...
                longitudinalGroundForce * obj.cgHeight / max(obj.wheelbase, eps);
            obj.state.lateralLoadTransfer = ...
                (groundLateralRollMoment + aeroRollMoment) / max(obj.trackWidth, eps);
            obj.state.groundLongitudinalPitchMoment = groundLongitudinalPitchMoment;
            obj.state.groundLateralRollMoment = groundLateralRollMoment;
            obj.state.aeroPitchMoment = aeroPitchMoment;
            obj.state.aeroVerticalPitchMoment = aeroVerticalPitchMoment;
            obj.state.dragPitchMoment = dragPitchMoment;
            obj.state.aeroRollMoment = aeroRollMoment;

            obj.state.updateCornerKinematics( ...
                obj.wheelbase, obj.trackWidth, obj.staticFrontWeight);
        end

        function cornerKinematics = computeCornerKinematics(obj)
            obj.state.updateCornerKinematics( ...
                obj.wheelbase, obj.trackWidth, obj.staticFrontWeight);
            cornerKinematics.displacement = obj.state.cornerDisplacement;
            cornerKinematics.velocity = obj.state.cornerVelocity;
        end

        function heave = getHeave(obj)
            heave = obj.state.heave;
        end

        function pitchAngle = getPitchAngle(obj)
            pitchAngle = obj.state.pitchAngle;
        end

        function rollAngle = getRollAngle(obj)
            rollAngle = obj.state.rollAngle;
        end
    end

    methods (Access = private)
        function inertia = computeAutoPitchInertia(obj)
            sprungMass = utils.positiveScalarOrDefault(obj.sprungMass, 1);
            wheelbase = utils.positiveScalarOrDefault(obj.wheelbase, 1);
            inertia = max(1, sprungMass * wheelbase^2 / 12);
        end

        function inertia = computeAutoRollInertia(obj)
            sprungMass = utils.positiveScalarOrDefault(obj.sprungMass, 1);
            trackWidth = utils.positiveScalarOrDefault(obj.trackWidth, 1);
            inertia = max(1, sprungMass * trackWidth^2 / 12);
        end
    end

    methods (Static, Access = private)




        function wheelRate = computeCornerWheelRate(corner)
            % COMPUTECORNERWHEELRATE Convert installed spring rate to wheel rate.
            wheelRate = max(0, corner.springRate) * corner.motionRatio^2;
        end

        function damping = computeCornerDamping(corner)
            % COMPUTECORNERDAMPING Use a small-signal damper slope.
            %
            % The per-corner suspension has different compression and rebound
            % coefficients. A heave/pitch/roll linear chassis model needs one
            % slope, so it uses their average as the local small-signal damping.
            damping = 0.5 * (max(0, corner.dampingCoeff) ...
                + max(0, corner.reboundCoeff)) * corner.motionRatio^2;
        end

    end
end
