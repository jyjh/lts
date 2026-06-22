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
        downforcePitchMoment = 0
        dragPitchMoment = 0
        aeroPitchMoment = 0
    end

    methods
        function obj = ChassisState()
            obj.reset();
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
            obj.downforcePitchMoment = 0;
            obj.dragPitchMoment = 0;
            obj.aeroPitchMoment = 0;
        end

        function updateCornerKinematics(obj, wheelbase, trackWidth, staticFrontWeight)
            % UPDATECORNERKINEMATICS Convert body attitude to corner motion
            % Outputs are positive for compression-producing body motion.
            frontArm = wheelbase * (1 - staticFrontWeight);
            rearArm = wheelbase * staticFrontWeight;
            halfTrack = trackWidth / 2;

            obj.cornerDisplacement.FL = obj.heave - obj.pitchAngle * frontArm - obj.rollAngle * halfTrack;
            obj.cornerDisplacement.FR = obj.heave - obj.pitchAngle * frontArm + obj.rollAngle * halfTrack;
            obj.cornerDisplacement.RL = obj.heave + obj.pitchAngle * rearArm - obj.rollAngle * halfTrack;
            obj.cornerDisplacement.RR = obj.heave + obj.pitchAngle * rearArm + obj.rollAngle * halfTrack;

            obj.cornerVelocity.FL = obj.heaveRate - obj.pitchRate * frontArm - obj.rollRate * halfTrack;
            obj.cornerVelocity.FR = obj.heaveRate - obj.pitchRate * frontArm + obj.rollRate * halfTrack;
            obj.cornerVelocity.RL = obj.heaveRate + obj.pitchRate * rearArm - obj.rollRate * halfTrack;
            obj.cornerVelocity.RR = obj.heaveRate + obj.pitchRate * rearArm + obj.rollRate * halfTrack;
        end
    end
end
