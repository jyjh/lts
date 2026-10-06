function tests = SimulationCapabilityCacheTest
tests = functiontests(localfunctions);
end

function testTireReplacementAndLiveValues(testCase)
vm = lts.vehicle.VehicleManager([], [], [], SimulatorCapabilityTireSpy(), []);
sim = lts.simulation.Simulator(vm, [], 0.001);
corner = struct('angularVelocity', 44, 'wheelRadius', 0.25);
verifyEqual(testCase, sim.computeLocalSlipRatio(corner, 10), 0.1, 'AbsTol', 1e-12);
verifyEqual(testCase, sim.drivenTireWheelRadius(), 0.25);

vm.tire.RL.wheelRadius = 0.3;
vm.tire.slipOffset = 0.2;
verifyEqual(testCase, sim.drivenTireWheelRadius(), 0.3);
verifyEqual(testCase, sim.computeLocalSlipRatio(corner, 10), 0.3, 'AbsTol', 1e-12);

% A same-class replacement must not retain a bound method or corner handle.
vm.tire = SimulatorCapabilityTireSpy();
vm.tire.slipOffset = 0.4;
sim.vehicleManager = vm;
verifyEqual(testCase, sim.computeLocalSlipRatio(corner, 10), 0.5, 'AbsTol', 1e-12);
verifyEqual(testCase, sim.drivenTireWheelRadius(), 0.25);

vm.tire = [];
sim.vehicleManager = vm;
verifyEqual(testCase, sim.computeLocalSlipRatio(corner, 10), 0.1, 'AbsTol', 1e-12);
verifyEmpty(testCase, sim.drivenTireWheelRadius());

vm.tire = struct('legacy', true);
sim.vehicleManager = vm;
verifyEqual(testCase, sim.computeLocalSlipRatio(corner, 10), 0.1, 'AbsTol', 1e-12);
verifyEmpty(testCase, sim.drivenTireWheelRadius());

vm.tire = SimulatorCapabilityTireSpy();
sim.vehicleManager = vm;
sim.resetForSimulation(true);
verifyEqual(testCase, sim.computeLocalSlipRatio(corner, 10), 0.1, 'AbsTol', 1e-12);
verifyEqual(testCase, sim.drivenTireWheelRadius(), 0.25);
end

function testDynamicCornerPropertyRemainsLive(testCase)
tire = SimulatorDynamicTireSpy();
vm = lts.vehicle.VehicleManager([], [], [], tire, []);
sim = lts.simulation.Simulator(vm, [], 0.001);
verifyEmpty(testCase, sim.drivenTireWheelRadius());
cornerProperty = addprop(tire, 'RL');
tire.RL = struct('wheelRadius', 0.28);
verifyEqual(testCase, sim.drivenTireWheelRadius(), 0.28);
delete(cornerProperty);
verifyEmpty(testCase, sim.drivenTireWheelRadius());
end

function testSuspensionReplacementStillValidatesInterface(testCase)
vm = lts.vehicle.VehicleManager([], SimulatorChassisOnlySuspensionSpy(100), ...
    [], [], [], SimulatorChassisSpy());
sim = lts.simulation.Simulator(vm, [], 0.001);
sim.requireChassis();
vm.suspension = SimulatorCapabilityTireSpy();
sim.vehicleManager = vm;
verifyError(testCase, @() sim.requireChassis(), ...
    'lts_simulation_Simulator:ChassisSuspensionRequired');
vm.suspension = SimulatorChassisOnlySuspensionSpy(200);
sim.vehicleManager = vm;
sim.requireChassis();
vm.chassis = [];
sim.vehicleManager = vm;
verifyError(testCase, @() sim.requireChassis(), 'lts_simulation_Simulator:ChassisRequired');
end

function testAttitudeReadsLiveStateAndReplacementCapabilities(testCase)
vm = lts.vehicle.VehicleManager([], [], [], [], []);
vm.chassis = lts.components.Chassis.SimpleChassis(vm, 200);
state = lts.simulation.VehicleState();
state.vehicleManager = vm;
for value = [0.02 0.04]
    vm.chassis.state.pitchAngle = value;
    vm.chassis.state.frontRollAngle = 2 * value;
    vm.chassis.state.rearRollAngle = value;
    vm.chassis.state.frontRollRate = 3 * value;
    vm.chassis.state.rearRollRate = value;
    state = state.updateAttitude();
    verifyEqual(testCase, state.pitchAngle, value);
    verifyEqual(testCase, state.frontRollAngle, vm.chassis.getFrontRollAngle());
    verifyEqual(testCase, state.twistAngle, vm.chassis.getTwistAngle());
    verifyEqual(testCase, state.frontRollRate, vm.chassis.getFrontRollRate());
    verifyEqual(testCase, state.twistRate, vm.chassis.getTwistRate());
end
vm.chassis = SimulatorChassisSpy();
vm.chassis.state.rollAngle = 0.3;
state.vehicleManager = vm;
state = state.updateAttitude();
verifyEqual(testCase, state.frontRollAngle, 0.3);
verifyEqual(testCase, state.rearRollAngle, 0.3);
verifyEqual(testCase, state.twistAngle, 0);
vm.chassis = lts.components.Chassis.SimpleChassis(vm, 200);
vm.chassis.state.frontRollAngle = 0.05;
state.vehicleManager = vm;
state = state.updateAttitude();
verifyEqual(testCase, state.frontRollAngle, vm.chassis.getFrontRollAngle());
verifyEqual(testCase, state.twistAngle, vm.chassis.getTwistAngle());
end
