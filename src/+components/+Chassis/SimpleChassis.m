classdef SimpleChassis < components.Chassis.ChassisComponent
    % SIMPLECHASSIS Lumped sprung-mass heave/pitch/roll chassis model
    %
    % Provides body-attitude state and derived corner kinematics consumed by
    % SuspensionManager for chassis-driven tire normal loads.

    properties
        state  % components.Chassis.ChassisState

        % Vehicle geometry/mass
        totalMass = 280
        sprungMass = 280
        wheelbase = 1.55
        trackWidth = 1.2
        cgHeight = 0.28
        staticFrontWeight = 0.45

        % Lumped inertias [kg*m^2]
        pitchInertia = 60
        rollInertia = 40

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
                obj.sprungMass = vehicleManager.totalMass;
                obj.wheelbase = vehicleManager.wheelbase;
                obj.trackWidth = vehicleManager.trackWidth;
                obj.cgHeight = vehicleManager.cgHeight;
                obj.staticFrontWeight = vehicleManager.staticFrontWeight;
            end
            if nargin >= 2 && ~isempty(sprungMass)
                obj.sprungMass = sprungMass;
            end
            if nargin >= 3 && ~isempty(pitchInertia)
                obj.pitchInertia = pitchInertia;
            else
                obj.pitchInertia = max(1, obj.sprungMass * obj.wheelbase^2 / 12);
            end
            if nargin >= 4 && ~isempty(rollInertia)
                obj.rollInertia = rollInertia;
            else
                obj.rollInertia = max(1, obj.sprungMass * obj.trackWidth^2 / 12);
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

        function updateFromAccelerations(obj, ax, ay, aeroForces, dt)
            % UPDATEFROMACCELERATIONS Integrate heave, pitch, and roll
            % ax > 0 creates nose-up pitch. ay > 0 creates right-side-down roll.
            if nargin < 4 || isempty(aeroForces)
                aeroForces = struct('Fz_front', 0, 'Fz_rear', 0, ...
                    'F_drag', 0, 'dragHeight', 0);
            end

            FzFront = obj.getStructField(aeroForces, 'Fz_front', 0);
            FzRear = obj.getStructField(aeroForces, 'Fz_rear', 0);
            Fdrag = obj.getStructField(aeroForces, 'F_drag', 0);
            dragHeight = obj.getStructField(aeroForces, 'dragHeight', 0);

            frontArm = obj.wheelbase * (1 - obj.staticFrontWeight);
            rearArm = obj.wheelbase * obj.staticFrontWeight;
            downforcePitchMoment = FzRear * rearArm - FzFront * frontArm;
            dragPitchMoment = Fdrag * dragHeight;
            aeroPitchMoment = downforcePitchMoment + dragPitchMoment;

            heaveForce = FzFront + FzRear ...
                - obj.heaveStiffness * obj.state.heave ...
                - obj.heaveDamping * obj.state.heaveRate;

            pitchMoment = obj.sprungMass * ax * obj.cgHeight + aeroPitchMoment ...
                - obj.pitchStiffness * obj.state.pitchAngle ...
                - obj.pitchDamping * obj.state.pitchRate;

            rollMoment = obj.sprungMass * ay * obj.cgHeight ...
                - obj.rollStiffness * obj.state.rollAngle ...
                - obj.rollDamping * obj.state.rollRate;

            obj.state.heaveAccel = heaveForce / max(obj.sprungMass, eps);
            obj.state.pitchAccel = pitchMoment / max(obj.pitchInertia, eps);
            obj.state.rollAccel = rollMoment / max(obj.rollInertia, eps);

            obj.state.heaveRate = obj.state.heaveRate + obj.state.heaveAccel * dt;
            obj.state.pitchRate = obj.state.pitchRate + obj.state.pitchAccel * dt;
            obj.state.rollRate = obj.state.rollRate + obj.state.rollAccel * dt;

            obj.state.heave = obj.state.heave + obj.state.heaveRate * dt;
            obj.state.pitchAngle = obj.state.pitchAngle + obj.state.pitchRate * dt;
            obj.state.rollAngle = obj.state.rollAngle + obj.state.rollRate * dt;

            obj.state.longitudinalLoadTransfer = ...
                obj.totalMass * ax * obj.cgHeight / max(obj.wheelbase, eps);
            obj.state.lateralLoadTransfer = ...
                obj.totalMass * ay * obj.cgHeight / max(obj.trackWidth, eps);
            obj.state.downforcePitchMoment = downforcePitchMoment;
            obj.state.dragPitchMoment = dragPitchMoment;
            obj.state.aeroPitchMoment = aeroPitchMoment;

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

    methods (Static, Access = private)
        function value = getStructField(s, fieldName, defaultValue)
            if isstruct(s) && isfield(s, fieldName)
                value = s.(fieldName);
            else
                value = defaultValue;
            end
        end
    end
end
