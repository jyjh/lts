classdef (Abstract) AeroComponent
    % AEROCOMPONENT Abstract interface for aerodynamic models
    % Each aero element has its own position, pitch sensitivity, and
    % computes forces based on full vehicle state (longitudinal speed,
    % lateral speed, pitch, and ride height).
    %
    % Multiple AeroComponents are composed by AeroManager.
    
    properties
        name        = 'AeroComponent'  % Descriptive name
        xPosition   = 0    % Longitudinal position from CG [m], positive = forward
        zPosition   = 0    % Nominal height above ground [m]
        ClA         = 1.0  % Downforce coefficient * area [m^2]
        CdA         = 0.5  % Drag coefficient * area [m^2]
        pitchSensitivityClA = 0  % ClA change per radian of pitch [1/rad]
    end
    
    methods (Abstract)
        %   state.speed, state.vy, state.pitchAngle, state.rideHeight used
        F_downforce = computeDownforce(obj, vehicleState)
        
        % Drag [N] computed from vehicle state
        F_drag = computeDrag(obj, vehicleState)
    end

    methods
        function obj = set.name(obj, value)
            if ischar(value) || (isstring(value) && isscalar(value))
                obj.name = char(value);
            else
                obj.name = 'AeroComponent';
            end
        end

        function obj = set.xPosition(obj, value)
            obj.xPosition = utils.scalarOrDefault(value, 0);
        end

        function obj = set.zPosition(obj, value)
            obj.zPosition = utils.scalarOrDefault(value, 0);
        end

        function obj = set.ClA(obj, value)
            obj.ClA = utils.nonnegativeScalarOrDefault(value, 1.0);
        end

        function obj = set.CdA(obj, value)
            obj.CdA = utils.nonnegativeScalarOrDefault(value, 0.5);
        end

        function obj = set.pitchSensitivityClA(obj, value)
            obj.pitchSensitivityClA = utils.scalarOrDefault(value, 0);
        end

        function obj = AeroComponent(name, xPos, zPos, ClA, CdA, pitchSensitivityClA)
            if nargin == 0
                return;
            end
            obj.name = name;
            obj.xPosition = utils.scalarOrDefault(xPos, obj.xPosition);
            obj.zPosition = utils.scalarOrDefault(zPos, obj.zPosition);
            obj.ClA = utils.nonnegativeScalarOrDefault(ClA, obj.ClA);
            obj.CdA = utils.nonnegativeScalarOrDefault(CdA, obj.CdA);
            obj.pitchSensitivityClA = utils.scalarOrDefault( ...
                pitchSensitivityClA, obj.pitchSensitivityClA);
        end

        function pos = getLongitudinalPosition(obj)
            % Distance from CG [m], positive = forward of CG
            pos = obj.xPosition;
        end
        
        function z = getNominalHeight(obj)
            % Nominal height above reference plane [m]
            z = obj.zPosition;
        end
        
        function n = getName(obj)
            n = obj.name;
        end
        
        function dz = computeHeightChange(obj, vehicleState)
            % COMPUTEHEIGHTCHANGE Local height change due to pitch
            % When the car pitches (nose up), a component ahead of CG
            % moves UP, and a component behind CG moves DOWN.
            %   dz = xPosition * sin(pitchAngle) ≈ xPosition * pitchAngle
            dz = obj.xPosition * vehicleState.pitchAngle;
        end
        
        function effectiveZ = computeEffectiveHeight(obj, vehicleState)
            % COMPUTEEFFECTIVEHEIGHT Actual height of this component
            % Accounts for: nominal z + local pitch displacement + global ride height
            effectiveZ = obj.zPosition ...
                   + obj.computeHeightChange(vehicleState) ...
                   + vehicleState.rideHeight;
        end

        function airSpeed = computeAirSpeed(~, vehicleState)
            % COMPUTEAIRSPEED Magnitude of the car's velocity through still air.
            %
            % The lap model stores body-axis forward speed in state.speed and
            % lateral velocity in state.vy. Aerodynamic pressure depends on
            % the full relative wind magnitude, not only the forward component.
            u = max(objGetField(vehicleState, 'speed', 0), 0);
            v = objGetField(vehicleState, 'vy', 0);
            airSpeed = hypot(u, v);
        end

        function beta = computeAeroSideslipAngle(obj, vehicleState)
            % COMPUTEAEROSIDESLIPANGLE Airflow yaw angle in body axes [rad].
            u = max(objGetField(vehicleState, 'speed', 0), 0);
            v = objGetField(vehicleState, 'vy', 0);
            beta = atan2(v, max(u, eps));
        end

        function q = computeDynamicPressure(obj, vehicleState)
            % COMPUTEDYNAMICPRESSURE Dynamic pressure from relative airspeed.
            rho = vehicleState.vehicleManager.airDensity;
            airSpeed = obj.computeAirSpeed(vehicleState);
            q = 0.5 * rho * airSpeed^2;
        end

        function F_downforce = computeDownforceFromClA(obj, vehicleState, effectiveClA)
            % COMPUTEDOWNFORCEFROMCLA Convert effective ClA to vertical force.
            effectiveClA = max(0, effectiveClA);
            F_downforce = obj.computeDynamicPressure(vehicleState) * effectiveClA;
        end

        function F_drag = computeLongitudinalDragFromCdA(obj, vehicleState, effectiveCdA)
            % COMPUTELONGITUDINALDRAGFROMCDA Body-x component of aero drag.
            %
            % Drag acts opposite the relative wind vector. The simulator's
            % force balance expects a positive body-longitudinal drag term that
            % gets subtracted from tire Fx, so project wind-aligned drag onto
            % the forward body axis instead of assuming vy is zero.
            effectiveCdA = max(0, effectiveCdA);
            u = max(objGetField(vehicleState, 'speed', 0), 0);
            airSpeed = obj.computeAirSpeed(vehicleState);
            if airSpeed <= eps || u <= 0 || effectiveCdA <= 0
                F_drag = 0;
                return;
            end

            windAlignedDrag = obj.computeDynamicPressure(vehicleState) * effectiveCdA;
            F_drag = windAlignedDrag * u / airSpeed;
        end

        function F_side = computeLateralDragFromCdA(obj, vehicleState, effectiveCdA)
            % COMPUTELATERALDRAGFROMCDA Body-y component of aero drag.
            %
            % This is the same wind-aligned drag vector used for F_drag, but
            % projected onto the lateral body axis. Positive vy means motion to
            % the left, so the aerodynamic force is to the right.
            effectiveCdA = max(0, effectiveCdA);
            v = objGetField(vehicleState, 'vy', 0);
            airSpeed = obj.computeAirSpeed(vehicleState);
            if airSpeed <= eps || abs(v) <= eps || effectiveCdA <= 0
                F_side = 0;
                return;
            end

            windAlignedDrag = obj.computeDynamicPressure(vehicleState) * effectiveCdA;
            F_side = -windAlignedDrag * v / airSpeed;
        end

        function F_side = computeSideDrag(obj, vehicleState)
            % COMPUTESIDEDRAG Body-y drag force from aero sideslip [N].
            %
            % Built-in components override this with their effective CdA. The
            % base fallback still gives custom components a physically signed
            % side drag using their nominal CdA.
            F_side = obj.computeLateralDragFromCdA(vehicleState, obj.CdA);
        end

        function forces = computeForces(obj, vehicleState)
            % COMPUTEFORCES Resolve this aero model to simulator force outputs
            %   forces.Fz_front, .Fz_rear, .F_drag, .F_side, .dragHeight
            %
            % A single component is split between front and rear axles by
            % moment balance. AeroManager overrides this for multiple
            % components, but exposes the same simulator-facing contract.

            wb = vehicleState.vehicleManager.wheelbase;
            cgHeight = vehicleState.vehicleManager.cgHeight;
            frontWeightFrac = vehicleState.vehicleManager.staticFrontWeight;

            b = wb * frontWeightFrac;  % CG to rear axle [m]

            F_downforce = obj.computeDownforce(vehicleState);
            F_drag = obj.computeDrag(vehicleState);
            F_side = obj.computeSideDrag(vehicleState);
            xi = obj.getLongitudinalPosition();
            zi = obj.computeEffectiveHeight(vehicleState);

            % Do not clamp this fraction. If an aero element is ahead of the
            % front axle or behind the rear axle, moment balance requires the
            % opposite axle's equivalent load to be negative. The suspension's
            % no-tension tire model handles actual wheel lift; clamping here
            % would delete the aero pitch moment before the chassis sees it.
            frontFrac = (b + xi) / wb;

            forces.Fz_front = F_downforce * frontFrac;
            forces.Fz_rear = F_downforce * (1 - frontFrac);
            forces.F_drag = F_drag;
            forces.F_side = F_side;
            forces.aeroYawMoment = xi * F_side;
            % Body y is positive left and positive roll is right-side-down.
            % A side force above the CG creates roll moment Mx = -z*Fy; this
            % must stay separate from tire lateral force at the ground plane.
            forces.aeroRollMoment = -F_side * (zi - cgHeight);
            if F_drag > 0
                forces.dragHeight = zi - cgHeight;
            else
                forces.dragHeight = 0;
            end
            forces.airSpeed = obj.computeAirSpeed(vehicleState);
            forces.aeroSideslipAngle = obj.computeAeroSideslipAngle(vehicleState);
        end
    end
end

function value = objGetField(obj, fieldName, defaultValue)
    % OBJGETFIELD Read either an object property or struct field.
    if isstruct(obj)
        if isfield(obj, fieldName)
            value = obj.(fieldName);
        else
            value = defaultValue;
        end
        return;
    end

    if isprop(obj, fieldName)
        value = obj.(fieldName);
    else
        value = defaultValue;
    end
end
