classdef ChassisState < handle
    % CHASSISSTATE Mutable chassis attitude and derived corner state
    %
    % State is measured from static equilibrium. Positive heave is downward
    % body motion, positive pitch is nose-up, and positive roll means the
    % right side moves downward relative to the left side.

    properties
        % Body attitude state
        heave = 0          % Sprung-mass vertical displacement [m], positive down
        heaveRate = 0      % Sprung-mass vertical velocity [m/s], positive down
        heaveAccel = 0     % Sprung-mass vertical acceleration [m/s^2]

        pitchAngle = 0     % Body pitch angle [rad], positive nose-up
        pitchRate = 0      % Body pitch rate [rad/s]
        pitchAccel = 0     % Body pitch acceleration [rad/s^2]

        rollAngle = 0      % Body roll angle [rad], positive right-side-down
        rollRate = 0       % Body roll rate [rad/s]
        rollAccel = 0      % Body roll acceleration [rad/s^2]

        % Derived corner chassis displacement/velocity at suspension pickups
        cornerDisplacement = struct('FL', 0, 'FR', 0, 'RL', 0, 'RR', 0)
        cornerVelocity = struct('FL', 0, 'FR', 0, 'RL', 0, 'RR', 0)

        % Derived load-transfer terms for telemetry/debugging [N]
        longitudinalLoadTransfer = 0
        lateralLoadTransfer = 0
        groundLongitudinalPitchMoment = 0
        groundLateralRollMoment = 0
        aeroPitchMoment = 0
        aeroVerticalPitchMoment = 0
        dragPitchMoment = 0
        aeroRollMoment = 0
    end

	    methods
	        function obj = ChassisState()
	            obj.reset();
	        end

	        function set.heave(obj, value)
	            obj.heave = utils.scalarOrDefault(value, 0);
	        end

	        function set.heaveRate(obj, value)
	            obj.heaveRate = utils.scalarOrDefault(value, 0);
	        end

	        function set.heaveAccel(obj, value)
	            obj.heaveAccel = utils.scalarOrDefault(value, 0);
	        end

	        function set.pitchAngle(obj, value)
	            obj.pitchAngle = utils.scalarOrDefault(value, 0);
	        end

	        function set.pitchRate(obj, value)
	            obj.pitchRate = utils.scalarOrDefault(value, 0);
	        end

	        function set.pitchAccel(obj, value)
	            obj.pitchAccel = utils.scalarOrDefault(value, 0);
	        end

	        function set.rollAngle(obj, value)
	            obj.rollAngle = utils.scalarOrDefault(value, 0);
	        end

	        function set.rollRate(obj, value)
	            obj.rollRate = utils.scalarOrDefault(value, 0);
	        end

	        function set.rollAccel(obj, value)
	            obj.rollAccel = utils.scalarOrDefault(value, 0);
	        end

	        function set.cornerDisplacement(obj, value)
	            obj.cornerDisplacement = utils.cornerStructOrDefault(value);
	        end

	        function set.cornerVelocity(obj, value)
	            obj.cornerVelocity = utils.cornerStructOrDefault(value);
	        end

	        function set.longitudinalLoadTransfer(obj, value)
	            obj.longitudinalLoadTransfer = utils.scalarOrDefault(value, 0);
	        end

	        function set.lateralLoadTransfer(obj, value)
	            obj.lateralLoadTransfer = utils.scalarOrDefault(value, 0);
	        end

	        function set.groundLongitudinalPitchMoment(obj, value)
	            obj.groundLongitudinalPitchMoment = utils.scalarOrDefault(value, 0);
	        end

	        function set.groundLateralRollMoment(obj, value)
	            obj.groundLateralRollMoment = utils.scalarOrDefault(value, 0);
	        end

	        function set.aeroPitchMoment(obj, value)
	            obj.aeroPitchMoment = utils.scalarOrDefault(value, 0);
	        end

	        function set.aeroVerticalPitchMoment(obj, value)
	            obj.aeroVerticalPitchMoment = utils.scalarOrDefault(value, 0);
	        end

	        function set.dragPitchMoment(obj, value)
	            obj.dragPitchMoment = utils.scalarOrDefault(value, 0);
	        end

	        function set.aeroRollMoment(obj, value)
	            obj.aeroRollMoment = utils.scalarOrDefault(value, 0);
	        end

        function reset(obj)
            obj.heave = 0;
            obj.heaveRate = 0;
            obj.heaveAccel = 0;
            obj.pitchAngle = 0;
            obj.pitchRate = 0;
            obj.pitchAccel = 0;
            obj.rollAngle = 0;
            obj.rollRate = 0;
            obj.rollAccel = 0;
            obj.cornerDisplacement = struct('FL', 0, 'FR', 0, 'RL', 0, 'RR', 0);
            obj.cornerVelocity = struct('FL', 0, 'FR', 0, 'RL', 0, 'RR', 0);
            obj.longitudinalLoadTransfer = 0;
            obj.lateralLoadTransfer = 0;
            obj.groundLongitudinalPitchMoment = 0;
            obj.groundLateralRollMoment = 0;
            obj.aeroPitchMoment = 0;
            obj.aeroVerticalPitchMoment = 0;
            obj.dragPitchMoment = 0;
            obj.aeroRollMoment = 0;
        end

	        function updateCornerKinematics(obj, wheelbase, trackWidth, staticFrontWeight)
	            % UPDATECORNERKINEMATICS Convert body attitude to corner motion
	            % Outputs are positive for compression-producing body motion.
	            frontArm = wheelbase * (1 - staticFrontWeight);
	            rearArm = wheelbase * staticFrontWeight;
	            halfTrack = trackWidth / 2;

	            displacement.FL = obj.heave - obj.pitchAngle * frontArm ...
	                - obj.rollAngle * halfTrack;
	            displacement.FR = obj.heave - obj.pitchAngle * frontArm ...
	                + obj.rollAngle * halfTrack;
	            displacement.RL = obj.heave + obj.pitchAngle * rearArm ...
	                - obj.rollAngle * halfTrack;
	            displacement.RR = obj.heave + obj.pitchAngle * rearArm ...
	                + obj.rollAngle * halfTrack;

	            velocity.FL = obj.heaveRate - obj.pitchRate * frontArm ...
	                - obj.rollRate * halfTrack;
	            velocity.FR = obj.heaveRate - obj.pitchRate * frontArm ...
	                + obj.rollRate * halfTrack;
	            velocity.RL = obj.heaveRate + obj.pitchRate * rearArm ...
	                - obj.rollRate * halfTrack;
	            velocity.RR = obj.heaveRate + obj.pitchRate * rearArm ...
	                + obj.rollRate * halfTrack;

	            obj.cornerDisplacement = displacement;
	            obj.cornerVelocity = velocity;
	        end
	    end
	end
