function tests = TelemetryChannelConformanceTest
% TELEMETRYCHANNELCONFORMANCETEST Consumer-side pin of the telemetry
% contract (item 3 of the repository split; see the Contracts page).
%
% The producer side lives in each component repository's
% tests/ConformanceTest.m, which pins the SuspensionState /
% ChassisState / PowertrainState / aero computeForces field names. This
% test pins the consumer side: the per-corner channel names
% lts.telemetry.StateLogBuilder creates and the stateLog field each one
% is populated from. A rename on either side of the boundary fails CI —
% neither the component nor this repository can drift alone.
tests = functiontests(localfunctions);
end

%% ---- Channel allocation: component-owned channels exist ----------------

function testComponentOwnedChannelsAreAllocated(testCase)
log = lts.telemetry.StateLogBuilder.createStateLog(1, false);
corners = {'FL', 'FR', 'RL', 'RR'};
suspensionChannels = {'Fz', 'suspensionForce', 'antiRollBarForce', ...
    'suspensionDemand', 'tireDeflection', 'damperPos', 'damperVel', ...
    'sprungPosition', 'unsprungPosition', 'sprungVelocity', ...
    'unsprungVelocity', 'wheelTravel', 'camber', 'toe', 'wheelSteer', ...
    'tireSpeed', 'tireUtilization'};
tireChannels = {'slipAngle', 'slipRatio', 'peakMu', 'omega', ...
    'tireFx', 'tireFy'};  % relaxedFz_* is created at log time, not allocated
fields = fieldnames(log);
for c = 1:numel(corners)
    for i = 1:numel(suspensionChannels)
        name = sprintf('%s_%s', suspensionChannels{i}, corners{c});
        verifyTrue(testCase, ismember(name, fields), ...
            sprintf('channel %s must be allocated.', name));
    end
    for i = 1:numel(tireChannels)
        name = sprintf('%s_%s', tireChannels{i}, corners{c});
        verifyTrue(testCase, ismember(name, fields), ...
            sprintf('channel %s must be allocated.', name));
    end
end
attitudeChannels = {'pitchAngle', 'rollAngle', 'rollRate', ...
    'frontRollAngle', 'rearRollAngle', 'frontRollRate', 'rearRollRate', ...
    'twistAngle', 'twistRate', 'rideHeight'};
for i = 1:numel(attitudeChannels)
    verifyTrue(testCase, ismember(attitudeChannels{i}, fields), ...
        sprintf('channel %s must be allocated.', attitudeChannels{i}));
end
powertrainChannels = {'motorRPM', 'motorTorque', 'motorTorqueRequested', ...
    'motorTorquePowerLimitNm', 'motorTorquePowerLimitActive', ...
    'wheelTorque', 'packVoltageV', 'packCurrentA', 'packPowerW', ...
    'drivenWheelRPM', 'rpmLimitActive', 'driveTorqueTotal', ...
    'driveTorque_RL', 'driveTorque_RR'};
for i = 1:numel(powertrainChannels)
    verifyTrue(testCase, ismember(powertrainChannels{i}, fields), ...
        sprintf('channel %s must be allocated.', powertrainChannels{i}));
end
aeroChannels = {'F_downforce', 'F_drag', 'aeroFz_front', 'aeroFz_rear'};
for i = 1:numel(aeroChannels)
    verifyTrue(testCase, ismember(aeroChannels{i}, fields), ...
        sprintf('channel %s must be allocated.', aeroChannels{i}));
end
end

%% ---- Mapping: per-corner channels read the pinned state fields ---------

function testSuspensionCornerMappingPinned(testCase)
% Each per-corner channel must be populated from the SuspensionState
% property of the same corner unit. Distinct sentinel values catch both
% renames and corner mix-ups.
[vm, prevState, newState, input, forces] = localStepFixture();
suspensionCorners = {'frontLeft', 'frontRight', 'rearLeft', 'rearRight'};
corners = {'FL', 'FR', 'RL', 'RR'};
map = {'Fz', 'tireNormalForce'; 'suspensionForce', 'suspensionForce'; ...
    'antiRollBarForce', 'antiRollBarForce'; ...
    'suspensionDemand', 'demandedLoad'; 'tireDeflection', 'tireDeflection'; ...
    'damperPos', 'damperPosition'; 'damperVel', 'damperVelocity'; ...
    'sprungPosition', 'sprungPosition'; ...
    'unsprungPosition', 'unsprungPosition'; ...
    'sprungVelocity', 'sprungVelocity'; ...
    'unsprungVelocity', 'unsprungVelocity'; ...
    'wheelTravel', 'wheelTravel'; 'camber', 'camberAngle'; ...
    'toe', 'toeAngle'; 'wheelSteer', 'steerAngle'};
for j = 1:numel(corners)
    state = vm.suspension.(suspensionCorners{j}).state;
    for i = 1:size(map, 1)
        state.(map{i, 2}) = 100 * j + i;  % corner- and field-unique
    end
    vm.suspension.(suspensionCorners{j}).state = state;
end
builder = lts.telemetry.StateLogBuilder(vm, 'full');
builder.beginRun(1, [], true);
builder.logStep(1, prevState, newState, input, forces);
log = builder.finish();
for j = 1:numel(corners)
    for i = 1:size(map, 1)
        channel = sprintf('%s_%s', map{i, 1}, corners{j});
        verifyEqual(testCase, log.(channel), 100 * j + i, ...
            sprintf('%s must read SuspensionState.%s of %s.', ...
            channel, map{i, 2}, suspensionCorners{j}));
    end
end
end

function testTireCornerMappingPinned(testCase)
% Per-corner tire channels read the TireState fields of the same corner.
[vm, prevState, newState, input, forces] = localStepFixture();
corners = {'FL', 'FR', 'RL', 'RR'};
map = {'slipAngle', 'slipAngle'; 'slipRatio', 'slipRatio'; ...
    'peakMu', 'peakMu'; 'omega', 'angularVelocity'; ...
    'tireFx', 'Fx'; 'tireFy', 'Fy'; 'relaxedFz', 'relaxedNormalLoad'};
for j = 1:numel(corners)
    tire = vm.tire.(corners{j});
    for i = 1:size(map, 1)
        tire.(map{i, 2}) = 200 * j + i;
    end
    vm.tire.(corners{j}) = tire;
end
builder = lts.telemetry.StateLogBuilder(vm, 'full');
builder.beginRun(1, [], true);
builder.logStep(1, prevState, newState, input, forces);
log = builder.finish();
for j = 1:numel(corners)
    for i = 1:size(map, 1)
        channel = sprintf('%s_%s', map{i, 1}, corners{j});
        verifyEqual(testCase, log.(channel), 200 * j + i, ...
            sprintf('%s must read TireState.%s of corner %s.', ...
            channel, map{i, 2}, corners{j}));
    end
end
end

function testAttitudeAndPowertrainChannelsPinned(testCase)
% ChassisState/PowertrainState-sourced channels flow through the
% Simulator's newState/forces; pin their names here (values are covered
% by the Simulator tests) and the aero forces contract fields.
[~, prevState, newState, input, forces] = localStepFixture();
forces.aeroFz_front = 111.5;
forces.aeroFz_rear = 222.5;
forces.motorRPM = 3500;
forces.wheelTorque = 421;
newState.pitchAngle = 0.0123;
newState.frontRollAngle = 0.0456;
builder = lts.telemetry.StateLogBuilder(localStepFixture(), 'full');
builder.beginRun(1, [], true);
builder.logStep(1, prevState, newState, input, forces);
log = builder.finish();
verifyEqual(testCase, log.aeroFz_front, 111.5);
verifyEqual(testCase, log.aeroFz_rear, 222.5);
verifyEqual(testCase, log.motorRPM, 3500);
verifyEqual(testCase, log.wheelTorque, 421);
verifyEqual(testCase, log.pitchAngle, 0.0123);
verifyEqual(testCase, log.frontRollAngle, 0.0456);
end

%% ---- Fixtures -----------------------------------------------------------

function [vm, prevState, newState, input, forces] = localStepFixture()
% Duck-typed vehicleManager: StateLogBuilder only reaches suspension
% corners' .state and the per-corner tire structs, so plain structs pin
% the mapping without any physics.
suspensionFields = {'tireNormalForce', 'suspensionForce', ...
    'antiRollBarForce', 'demandedLoad', 'tireDeflection', ...
    'damperPosition', 'damperVelocity', 'sprungPosition', ...
    'unsprungPosition', 'sprungVelocity', 'unsprungVelocity', ...
    'wheelTravel', 'camberAngle', 'toeAngle', 'steerAngle'};
suspensionCorners = {'frontLeft', 'frontRight', 'rearLeft', 'rearRight'};
suspension = struct();
for j = 1:numel(suspensionCorners)
    state = struct();
    for i = 1:numel(suspensionFields)
        state.(suspensionFields{i}) = 0;
    end
    suspension.(suspensionCorners{j}) = struct('state', state);
end

corners = {'FL', 'FR', 'RL', 'RR'};
tireFields = {'slipAngle', 'slipRatio', 'peakMu', 'angularVelocity', ...
    'Fx', 'Fy', 'relaxedNormalLoad', 'normalForce', 'wheelRadius'};
tire = struct();
for j = 1:numel(corners)
    t = struct();
    for i = 1:numel(tireFields)
        t.(tireFields{i}) = 0;
    end
    t.peakMu = 1;
    t.normalForce = 1;
    t.Fx = 0;
    t.Fy = 0;
    tire.(corners{j}) = t;
end

vm = struct('suspension', suspension, 'tire', tire);

prevState = struct('time', 0, 's', 0, 'speed', 0);
newState = localBlankState();
input = struct('throttle', 0, 'steer', 0);
forces = localBlankForces();
end

function state = localBlankState()
state = struct( ...
    'time', 0.001, 's', 1, 'x', 1, 'y', 0, 'yaw', 0, ...
    'vx', 10, 'vy', 0, 'bodySlipAngle', 0, 'speed', 10, ...
    'ax', 0, 'ay', 0, 'frontAxleAy', 0, 'rearAxleAy', 0, ...
    'yawRate', 0, 'yawAccel', 0, ...
    'refS', 1, 'refHeading', 0, 'refCurvature', 0, ...
    'lateralError', 0, 'onTrack', true, 'heading', 0, ...
    'pitchAngle', 0, 'rollAngle', 0, 'rollRate', 0, ...
    'frontRollAngle', 0, 'rearRollAngle', 0, ...
    'frontRollRate', 0, 'rearRollRate', 0, ...
    'twistAngle', 0, 'twistRate', 0, 'rideHeight', 0);
end

function forces = localBlankForces()
forces = struct( ...
    'brake', 0, 'brakeCommand', 0, 'brakePressureMode', false, ...
    'brakePressureFrontBar', NaN, 'brakePressureRearBar', NaN, ...
    'F_downforce', 0, 'F_drag', 0, 'F_drive', 0, 'F_brake', 0, ...
    'F_tire_long', 0, 'F_tire_lat', 0, 'yawMoment', 0, ...
    'rollResistance', 0, ...
    'F_brake_front', 0, 'F_brake_rear', 0, ...
    'F_brake_FL', 0, 'F_brake_FR', 0, 'F_brake_RL', 0, 'F_brake_RR', 0, ...
    'brakeGrip_FL', 0, 'brakeGrip_FR', 0, ...
    'brakeGrip_RL', 0, 'brakeGrip_RR', 0, ...
    'driveTorqueTotal', 0, 'driveTorque_RL', 0, 'driveTorque_RR', 0, ...
    'brakeTorque_FL', 0, 'brakeTorque_FR', 0, ...
    'brakeTorque_RL', 0, 'brakeTorque_RR', 0, ...
    'motorRPM', 0, 'motorTorque', 0, 'motorTorqueRequested', 0, ...
    'motorTorquePowerLimitNm', NaN, 'motorTorquePowerLimitActive', false, ...
    'wheelTorque', 0, 'packVoltageV', NaN, 'packCurrentA', NaN, ...
    'packPowerW', NaN, 'drivenWheelRPM', 0, 'rpmLimitActive', false, ...
    'aeroFz_front', 0, 'aeroFz_rear', 0);
end
