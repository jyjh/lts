classdef SuspensionManager < components.Suspension.SuspensionComponent
    % SUSPENSIONMANAGER Manages four corner suspension units
    % Creates and coordinates per-corner SimpleSuspension instances with
    % associated SuspensionState objects.
    %
    % Front corners (FL, FR) share identical suspension parameters.
    % Rear corners (RL, RR) share identical suspension parameters.
    % Each corner has its own independent SuspensionState for transient tracking.
    %
    % Usage:
    %   mgr = SuspensionManager(vehicleManager, ...)
    %   loads = mgr.computeCornerLoads(state, Fz_aero_front, Fz_aero_rear, totalMass, dt)
    %     loads.FL, loads.FR, loads.RL, loads.RR  - per-corner tire normal force [N]
    
    properties
        % Per-corner suspension units
        frontLeft      % SimpleSuspension (front params)
        frontRight     % SimpleSuspension (front params)
        rearLeft       % SimpleSuspension (rear params)
        rearRight      % SimpleSuspension (rear params)
        
        % Static front weight distribution (from VehicleManager)
        staticFrontWeight = 0.48  % [0-1]

        % Suspension and steering kinematic model
        geometry
    end
    
    methods
        function obj = SuspensionManager(vehicleManager, ...
                frontRollStiffDist, ...
                frontSpringRate, frontDampingCoeff, frontReboundCoeff, ...
                rearSpringRate, rearDampingCoeff, rearReboundCoeff, ...
                motionRatio, bumpStopLength, bumpStopRate, ...
                tireSpringRate, unsprungMass, geometry)
            % SUSPENSIONMANAGER Construct with front/rear suspension parameters
            %   SuspensionManager(vehicleManager, ...
            %       frontRollStiffDist, ...
            %       frontSpringRate, frontDampingCoeff, frontReboundCoeff, ...
            %       rearSpringRate, rearDampingCoeff, rearReboundCoeff, ...
            %       motionRatio, bumpStopLength, bumpStopRate, ...
            %       tireSpringRate, unsprungMass)
            %
            %   vehicleManager      - VehicleManager handle (geometry pulled by SimpleSuspension)
            %   frontRollStiffDist  - Front roll stiffness distribution [0-1]
            %   frontSpringRate     - Front heave spring rate [N/m]
            %   frontDampingCoeff   - Front compression damping [N·s/m]
            %   frontReboundCoeff   - Front rebound damping [N·s/m]
            %   rearSpringRate      - Rear heave spring rate [N/m]
            %   rearDampingCoeff    - Rear compression damping [N·s/m]
            %   rearReboundCoeff    - Rear rebound damping [N·s/m]
            %   motionRatio         - Installation motion ratio (shared)
            %   bumpStopLength      - Bump stop travel [m] (shared)
            %   bumpStopRate        - Bump stop stiffness [N/m] (shared)
            %   tireSpringRate      - Tire vertical stiffness [N/m] (shared)
            %   unsprungMass        - Per-corner unsprung mass [kg] (shared)
            
            if nargin < 14 || isempty(geometry)
                geometry = components.Suspension.SuspensionGeometry.fromPreset( ...
                    'neutral', vehicleManager);
                geometry.frontMotionRatioCurve = motionRatio * [1 1 1];
                geometry.rearMotionRatioCurve = motionRatio * [1 1 1];
            end

            % Pull static weight distribution from VehicleManager
            obj.staticFrontWeight = vehicleManager.staticFrontWeight;
            obj.geometry = geometry;

            totalSprungMass = max(vehicleManager.totalMass - 4 * unsprungMass, eps);
            frontSprungMass = max(totalSprungMass * obj.staticFrontWeight / 2, eps);
            rearSprungMass = max(totalSprungMass * (1 - obj.staticFrontWeight) / 2, eps);
            
            % Create front corners (share front parameters, each has own state)
            obj.frontLeft = components.Suspension.SimpleSuspension( ...
                vehicleManager, frontRollStiffDist, ...
                frontSpringRate, frontDampingCoeff, frontReboundCoeff, ...
                motionRatio, bumpStopLength, bumpStopRate, ...
                tireSpringRate, unsprungMass, frontSprungMass);
            
            obj.frontRight = components.Suspension.SimpleSuspension( ...
                vehicleManager, frontRollStiffDist, ...
                frontSpringRate, frontDampingCoeff, frontReboundCoeff, ...
                motionRatio, bumpStopLength, bumpStopRate, ...
                tireSpringRate, unsprungMass, frontSprungMass);
            
            % Create rear corners (share rear parameters, each has own state)
            % Rear roll stiffness distribution = 1 - front
            rearRollStiffDist = 1 - frontRollStiffDist;
            obj.rearLeft = components.Suspension.SimpleSuspension( ...
                vehicleManager, rearRollStiffDist, ...
                rearSpringRate, rearDampingCoeff, rearReboundCoeff, ...
                motionRatio, bumpStopLength, bumpStopRate, ...
                tireSpringRate, unsprungMass, rearSprungMass);
            
            obj.rearRight = components.Suspension.SimpleSuspension( ...
                vehicleManager, rearRollStiffDist, ...
                rearSpringRate, rearDampingCoeff, rearReboundCoeff, ...
                motionRatio, bumpStopLength, bumpStopRate, ...
                tireSpringRate, unsprungMass, rearSprungMass);
        end
        
        %% ---- Warmup: settle suspension to static equilibrium ----
        
        function warmup(obj, totalMass, dt)
            % WARMUP Settle suspension state to static equilibrium
            %   warmup(totalMass, dt)
            %
            %   Initializes deterministic per-corner static load and
            %   deflection state. Dynamic displacement states are measured
            %   from this equilibrium.
            %
            %   totalMass - Total vehicle mass [kg]
            %   dt        - Unused, kept for interface compatibility
            
            if nargin < 3
                dt = 0.001;
            end
            %#ok<NASGU>

            W = totalMass * 9.81;
            
            % Static weight per corner (no aero, no load transfer)
            Fz_static_front = W * obj.staticFrontWeight;
            Fz_static_rear  = W * (1 - obj.staticFrontWeight);
            demanded_FL = Fz_static_front / 2;
            demanded_FR = Fz_static_front / 2;
            demanded_RL = Fz_static_rear  / 2;
            demanded_RR = Fz_static_rear  / 2;

            obj.frontLeft.initializeStaticLoad( obj.frontLeft.state,  demanded_FL);
            obj.frontRight.initializeStaticLoad(obj.frontRight.state, demanded_FR);
            obj.rearLeft.initializeStaticLoad(  obj.rearLeft.state,   demanded_RL);
            obj.rearRight.initializeStaticLoad( obj.rearRight.state,  demanded_RR);
            obj.updateGeometry(0);
        end
        
        %% ---- Per-corner transient computation ----
        
        function loads = computeCornerLoads(obj, state, Fz_aero_front, Fz_aero_rear, totalMass, dt)
            % COMPUTECORNERLOADS Compute demanded loads and update all four corners
            %   loads = computeCornerLoads(state, Fz_aero_front, Fz_aero_rear, totalMass, dt)
            %
            %   state          - VehicleState with ax, ay, speed, etc.
            %   Fz_aero_front  - Aero downforce on front axle [N]
            %   Fz_aero_rear   - Aero downforce on rear axle [N]
            %   totalMass      - Total vehicle mass [kg]
            %   dt             - Timestep [s]
            %
            %   Returns struct with per-corner tire normal forces:
            %     loads.FL, loads.FR, loads.RL, loads.RR  [N]
            
            W = totalMass * 9.81;
            ax = state.ax;
            ay = state.ay;
            
            % Geometry (stored in corners from VehicleManager)
            tw = obj.frontLeft.trackWidth;
            wb = obj.frontLeft.wheelbase;
            cgH = obj.frontLeft.cgHeight;
            frontWeightFrac = obj.staticFrontWeight;
            rollStiffDist = obj.frontLeft.rollStiffDist;
            
            % --- Static weight per corner ---
            Fz_static_front = W * frontWeightFrac;
            Fz_static_rear  = W * (1 - frontWeightFrac);
            Fz_static_FL = Fz_static_front / 2;
            Fz_static_FR = Fz_static_front / 2;
            Fz_static_RL = Fz_static_rear  / 2;
            Fz_static_RR = Fz_static_rear  / 2;
            
            % --- Aero downforce per corner (split evenly per axle) ---
            Fz_aero_FL = Fz_aero_front / 2;
            Fz_aero_FR = Fz_aero_front / 2;
            Fz_aero_RL = Fz_aero_rear  / 2;
            Fz_aero_RR = Fz_aero_rear  / 2;
            
            % --- Lateral load transfer ---
            % positive ay = left turn → load transfers to right side
            totalLatTransfer = totalMass * abs(ay) * cgH / tw;
            frontLatTransfer = totalLatTransfer * rollStiffDist;
            rearLatTransfer  = totalLatTransfer * (1 - rollStiffDist);
            
            sign_ay = sign(ay);
            Fz_lat_FL = -sign_ay * frontLatTransfer / 2;
            Fz_lat_FR =  sign_ay * frontLatTransfer / 2;
            Fz_lat_RL = -sign_ay * rearLatTransfer / 2;
            Fz_lat_RR =  sign_ay * rearLatTransfer / 2;
            
            % --- Longitudinal load transfer ---
            % positive ax (acceleration) → load transfers to rear
            totalLongTransfer = totalMass * ax * cgH / wb;
            Fz_long_FL = -totalLongTransfer / 2;
            Fz_long_FR = -totalLongTransfer / 2;
            Fz_long_RL =  totalLongTransfer / 2;
            Fz_long_RR =  totalLongTransfer / 2;
            
            % --- Total demanded load per corner ---
            demanded_FL = Fz_static_FL + Fz_aero_FL + Fz_lat_FL + Fz_long_FL;
            demanded_FR = Fz_static_FR + Fz_aero_FR + Fz_lat_FR + Fz_long_FR;
            demanded_RL = Fz_static_RL + Fz_aero_RL + Fz_lat_RL + Fz_long_RL;
            demanded_RR = Fz_static_RR + Fz_aero_RR + Fz_lat_RR + Fz_long_RR;
            
            % --- Update each corner's transient state ---
            obj.frontLeft.updateCorner( obj.frontLeft.state,  demanded_FL, dt);
            obj.frontRight.updateCorner(obj.frontRight.state, demanded_FR, dt);
            obj.rearLeft.updateCorner(  obj.rearLeft.state,   demanded_RL, dt);
            obj.rearRight.updateCorner( obj.rearRight.state,  demanded_RR, dt);
            obj.updateGeometry(state.steer);
            
            % --- Return per-corner tire normal forces ---
            loads.FL = obj.frontLeft.state.tireNormalForce;
            loads.FR = obj.frontRight.state.tireNormalForce;
            loads.RL = obj.rearLeft.state.tireNormalForce;
            loads.RR = obj.rearRight.state.tireNormalForce;
        end

        function updateGeometry(obj, steerInput)
            % UPDATEGEOMETRY Refresh per-corner suspension kinematics.
            obj.updateCornerGeometry(obj.frontLeft,  'FL', steerInput);
            obj.updateCornerGeometry(obj.frontRight, 'FR', steerInput);
            obj.updateCornerGeometry(obj.rearLeft,   'RL', steerInput);
            obj.updateCornerGeometry(obj.rearRight,  'RR', steerInput);
        end

        function loads = estimateCornerLoads(obj, state, Fz_aero_front, Fz_aero_rear, totalMass)
            % ESTIMATECORNERLOADS Compute load-transfer demands without
            % advancing suspension state.

            W = totalMass * 9.81;
            ax = state.ax;
            ay = state.ay;

            tw = obj.frontLeft.trackWidth;
            wb = obj.frontLeft.wheelbase;
            cgH = obj.frontLeft.cgHeight;
            frontWeightFrac = obj.staticFrontWeight;
            rollStiffDist = obj.frontLeft.rollStiffDist;

            Fz_static_front = W * frontWeightFrac;
            Fz_static_rear  = W * (1 - frontWeightFrac);
            Fz_static_FL = Fz_static_front / 2;
            Fz_static_FR = Fz_static_front / 2;
            Fz_static_RL = Fz_static_rear  / 2;
            Fz_static_RR = Fz_static_rear  / 2;

            Fz_aero_FL = Fz_aero_front / 2;
            Fz_aero_FR = Fz_aero_front / 2;
            Fz_aero_RL = Fz_aero_rear  / 2;
            Fz_aero_RR = Fz_aero_rear  / 2;

            totalLatTransfer = totalMass * abs(ay) * cgH / tw;
            frontLatTransfer = totalLatTransfer * rollStiffDist;
            rearLatTransfer  = totalLatTransfer * (1 - rollStiffDist);

            sign_ay = sign(ay);
            Fz_lat_FL = -sign_ay * frontLatTransfer / 2;
            Fz_lat_FR =  sign_ay * frontLatTransfer / 2;
            Fz_lat_RL = -sign_ay * rearLatTransfer / 2;
            Fz_lat_RR =  sign_ay * rearLatTransfer / 2;

            totalLongTransfer = totalMass * ax * cgH / wb;
            Fz_long_FL = -totalLongTransfer / 2;
            Fz_long_FR = -totalLongTransfer / 2;
            Fz_long_RL =  totalLongTransfer / 2;
            Fz_long_RR =  totalLongTransfer / 2;

            loads.FL = max(Fz_static_FL + Fz_aero_FL + Fz_lat_FL + Fz_long_FL, 0);
            loads.FR = max(Fz_static_FR + Fz_aero_FR + Fz_lat_FR + Fz_long_FR, 0);
            loads.RL = max(Fz_static_RL + Fz_aero_RL + Fz_lat_RL + Fz_long_RL, 0);
            loads.RR = max(Fz_static_RR + Fz_aero_RR + Fz_lat_RR + Fz_long_RR, 0);
        end

        function updateCornerGeometry(obj, cornerUnit, cornerName, steerInput)
            cornerState = cornerUnit.state;
            wheelTravel = cornerState.damperPosition / max(cornerUnit.motionRatio, eps);
            kin = obj.geometry.computeCornerKinematics(cornerName, wheelTravel, steerInput);

            cornerState.wheelTravel = kin.wheelTravel;
            cornerState.camberAngle = kin.camberAngle;
            cornerState.toeAngle = kin.toeAngle;
            cornerState.steerAngle = kin.steerAngle;
            cornerState.motionRatioEffective = max(kin.motionRatio, eps);
        end

        function cornerKinematics = getCornerKinematics(obj)
            % GETCORNERKINEMATICS Return tire-facing geometry for all corners.
            cornerKinematics.FL = obj.stateToKinematics(obj.frontLeft.state);
            cornerKinematics.FR = obj.stateToKinematics(obj.frontRight.state);
            cornerKinematics.RL = obj.stateToKinematics(obj.rearLeft.state);
            cornerKinematics.RR = obj.stateToKinematics(obj.rearRight.state);
        end
        
        function pitchAngle = computePitchAngle(obj)
            % COMPUTEPITCHANGLE Compute dynamic body pitch from sprung motion
            %   pitchAngle = computePitchAngle()
            %
            %   Uses average front and rear sprung-mass positions measured
            %   from static equilibrium. Static rake or undertray ride-height
            %   offsets are treated as the zero-pitch reference after warmup.
            %
            %   Positive pitch = nose up (e.g. rear compresses more under
            %   acceleration squat).
            %   Negative pitch = nose down (e.g. front compresses more under
            %   braking dive).
            %
            %   Geometry is simplified to:
            %     pitchAngle = atan2(avgRearSprungDown - avgFrontSprungDown, wheelbase)
            
            avgFrontSprungDown = (obj.frontLeft.state.sprungPosition + ...
                                  obj.frontRight.state.sprungPosition) / 2;
            avgRearSprungDown  = (obj.rearLeft.state.sprungPosition + ...
                                  obj.rearRight.state.sprungPosition) / 2;
            
            pitchAngle = atan2(avgRearSprungDown - avgFrontSprungDown, ...
                obj.frontLeft.wheelbase);
        end
    end

    methods (Static, Access = private)
        function kin = stateToKinematics(state)
            kin.wheelTravel = state.wheelTravel;
            kin.camberAngle = state.camberAngle;
            kin.toeAngle = state.toeAngle;
            kin.steerAngle = state.steerAngle;
            kin.motionRatio = state.motionRatioEffective;
        end
    end
end
