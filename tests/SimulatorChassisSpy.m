classdef SimulatorChassisSpy < lts.components.Chassis.ChassisComponent
    properties
        state = lts.components.Chassis.ChassisState()
        updateCalls = 0
    end

    methods
        function reset(obj)
            obj.state.reset();
        end

        function updateFromAccelerations(obj, ~, ~, aeroForces, ~, ~)
            obj.updateCalls = obj.updateCalls + 1;
            fdrag = 0;
            dragHeight = 0;
            if isstruct(aeroForces) && isfield(aeroForces, 'F_drag')
                fdrag = aeroForces.F_drag;
            end
            if isstruct(aeroForces) && isfield(aeroForces, 'dragHeight')
                dragHeight = aeroForces.dragHeight;
            end
            obj.state.downforcePitchMoment = 0;
            obj.state.dragPitchMoment = fdrag * dragHeight;
            obj.state.aeroPitchMoment = obj.state.downforcePitchMoment + ...
                obj.state.dragPitchMoment;
        end

        function cornerKinematics = computeCornerKinematics(obj)
            cornerKinematics = struct( ...
                'displacement', obj.state.cornerDisplacement, ...
                'velocity', obj.state.cornerVelocity);
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
end
