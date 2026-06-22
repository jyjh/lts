function tests = ChassisLoadTransferTest
tests = functiontests(localfunctions);
end

function testPositiveLongitudinalAccelCreatesRearwardLoadShift(testCase)
[vehicle, suspension, chassis] = createVehicleWithChassis();
zeroAero = zeroAeroForces();
dt = 0.001;

for idx = 1:500
    chassis.updateFromAccelerations(4, 0, zeroAero, dt);
    loads = suspension.computeCornerLoadsFromChassis(chassis, 0, dt);
end

frontLoad = loads.FL + loads.FR;
rearLoad = loads.RL + loads.RR;
staticFront = vehicle.totalMass * 9.81 * vehicle.staticFrontWeight;
staticRear = vehicle.totalMass * 9.81 * (1 - vehicle.staticFrontWeight);

verifyGreaterThan(testCase, chassis.getPitchAngle(), 0);
verifyLessThan(testCase, frontLoad, staticFront);
verifyGreaterThan(testCase, rearLoad, staticRear);
end

function testPositiveLateralAccelCreatesRightSideLoadShift(testCase)
[~, suspension, chassis] = createVehicleWithChassis();
zeroAero = zeroAeroForces();
dt = 0.001;

for idx = 1:500
    chassis.updateFromAccelerations(0, 6, zeroAero, dt);
    loads = suspension.computeCornerLoadsFromChassis(chassis, 0, dt);
end

leftLoad = loads.FL + loads.RL;
rightLoad = loads.FR + loads.RR;

verifyGreaterThan(testCase, chassis.getRollAngle(), 0);
verifyGreaterThan(testCase, rightLoad, leftLoad);
end

function testDragAboveCgCreatesPositivePitchMoment(testCase)
[~, ~, chassis] = createVehicleWithChassis();
aeroForces = zeroAeroForces();
aeroForces.F_drag = 100;
aeroForces.dragHeight = 0.5;

chassis.updateFromAccelerations(0, 0, aeroForces, 0.001);

verifyEqual(testCase, chassis.state.dragPitchMoment, 50, 'AbsTol', 1e-12);
verifyEqual(testCase, chassis.state.aeroPitchMoment, 50, 'AbsTol', 1e-12);
verifyGreaterThan(testCase, chassis.getPitchAngle(), 0);
end

function testAlgebraicSuspensionFallbackStillComputesLoads(testCase)
vehicle = VehicleManager([], [], [], [], []);
geometry = components.Suspension.SuspensionGeometry.fromPreset('baseline', vehicle);
suspension = createSuspension(vehicle, geometry, 25);
vehicle.suspension = suspension;
suspension.warmup(vehicle.totalMass, 0.001);
state = VehicleState('speed', 20);
state.vehicleManager = vehicle;
state.ax = 3;
state.ay = 6;

loads = suspension.computeCornerLoads(state, 120, 80, vehicle.totalMass, 0.001);
loadValues = [loads.FL; loads.FR; loads.RL; loads.RR];

verifyTrue(testCase, all(isfinite(loadValues)));
verifyTrue(testCase, all(loadValues >= 0));
end

function [vehicle, suspension, chassis] = createVehicleWithChassis()
vehicle = VehicleManager([], [], [], [], []);
unsprungMass = 25;
sprungMass = vehicle.totalMass - 4 * unsprungMass;
chassis = components.Chassis.SimpleChassis(vehicle, sprungMass);
vehicle.chassis = chassis;
geometry = components.Suspension.SuspensionGeometry.fromPreset('baseline', vehicle);
suspension = createSuspension(vehicle, geometry, unsprungMass);
vehicle.suspension = suspension;
suspension.warmup(vehicle.totalMass, 0.001);
end

function suspension = createSuspension(vehicle, geometry, unsprungMass)
suspension = components.Suspension.SuspensionManager( ...
    vehicle, ...
    0.55, ...
    45000, 3000, 4500, ...
    42000, 2800, 4200, ...
    0.95, ...
    0.025, ...
    200000, ...
    200000, ...
    unsprungMass, ...
    geometry);
end

function aeroForces = zeroAeroForces()
aeroForces = struct( ...
    'Fz_front', 0, ...
    'Fz_rear', 0, ...
    'F_drag', 0, ...
    'dragHeight', 0);
end
