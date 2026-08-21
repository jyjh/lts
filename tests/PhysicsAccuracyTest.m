function tests = PhysicsAccuracyTest
tests = functiontests(localfunctions);
end

function testConstantForceWorkMatchesKineticEnergyChange(testCase)
mass = 250;
force = 1250;
dt = 0.001;
duration = 2;
vehicle = lts.vehicle.VehicleManager([], [], [], [], []);
vehicle.totalMass = mass;
vehicle.yawInertia = 100;
vehicle.wheelbase = 1.6;
vehicle.staticFrontWeight = 0.5;
simulator = lts.simulation.Simulator(vehicle, [], dt);
state = lts.simulation.VehicleState( ...
    'speed', 7, 'vx', 7, 'vy', 0, 'yaw', 0, 'yawRate', 0, ...
    'x', 0, 'y', 0);
tireData = struct('sumFxBody', force, 'sumFyBody', 0, 'yawMoment', 0);
initialEnergy = 0.5 * mass * state.speed^2;

for idx = 1:round(duration / dt)
    dynamics = simulator.computePlanarDynamics(state, tireData, 0);
    next = simulator.integratePlanarKinematics(state, dynamics, dt);
    state.vx = next.vx;
    state.vy = next.vy;
    state.speed = hypot(next.vx, next.vy);
    state.yawRate = next.yawRate;
    state.yaw = next.yaw;
    state.x = next.x;
    state.y = next.y;
end

work = force * state.x;
energyChange = 0.5 * mass * state.speed^2 - initialEnergy;
verifyEqual(testCase, energyChange, work, 'RelTol', 1e-11, 'AbsTol', 1e-9);
end

function testQuadraticDragAndRollingResistanceMatchAnalyticCoastdown(testCase)
mass = 250;
rho = 1.2;
cdA = 1.2;
crr = 0.015;
g = lts.vehicle.VehicleManager.g;
v0 = 25;
dt = 0.001;
duration = 3;
vehicle = lts.vehicle.VehicleManager([], [], [], [], []);
vehicle.totalMass = mass;
vehicle.yawInertia = 100;
vehicle.wheelbase = 1.6;
vehicle.staticFrontWeight = 0.5;
vehicle.airDensity = rho;
aero = lts.components.Aero.WholeCarAero(0, 0, 0, cdA, 0);
simulator = lts.simulation.Simulator(vehicle, [], dt);
state = lts.simulation.VehicleState( ...
    'speed', v0, 'vx', v0, 'vy', 0, 'yaw', 0, 'yawRate', 0, ...
    'x', 0, 'y', 0);
state.vehicleManager = vehicle;
rollingForce = crr * mass * g;
tireData = struct( ...
    'sumFxBody', -rollingForce, 'sumFyBody', 0, 'yawMoment', 0);

for idx = 1:round(duration / dt)
    aeroForces = aero.computeForces(state);
    dynamics = simulator.computePlanarDynamics(state, tireData, aeroForces);
    next = simulator.integratePlanarKinematics(state, dynamics, dt);
    state.vx = next.vx;
    state.vy = next.vy;
    state.speed = hypot(next.vx, next.vy);
    state.yawRate = next.yawRate;
    state.yaw = next.yaw;
    state.x = next.x;
    state.y = next.y;
end

quadraticCoeff = 0.5 * rho * cdA / mass;
constantDecel = crr * g;
scale = sqrt(constantDecel / quadraticCoeff);
phase0 = atan(v0 * sqrt(quadraticCoeff / constantDecel));
expectedSpeed = scale * tan(phase0 - ...
    sqrt(quadraticCoeff * constantDecel) * duration);
verifyEqual(testCase, state.speed, expectedSpeed, 'AbsTol', 0.01);
verifyLessThan(testCase, state.speed, v0);
end

function testLowSlipSteadyStateMatchesLinearBicycleModel(testCase)
[vehicle, tire, simulator] = createPlanarFixture(0);
speed = tire.tireConstants.refVelocity;
delta = 0.002;
frontArm = vehicle.wheelbase * (1 - vehicle.staticFrontWeight);
rearArm = vehicle.wheelbase * vehicle.staticFrontWeight;
frontLoad = vehicle.totalMass * vehicle.g * vehicle.staticFrontWeight / 2;
rearLoad = vehicle.totalMass * vehicle.g * ...
    (1 - vehicle.staticFrontWeight) / 2;
alphaProbe = 1e-5;
corneringStiffnessFront = 2 * centralLateralStiffness( ...
    tire, frontLoad, alphaProbe);
corneringStiffnessRear = 2 * centralLateralStiffness( ...
    tire, rearLoad, alphaProbe);

understeerTerm = vehicle.totalMass * speed^2 / vehicle.wheelbase^2 * ...
    (rearArm / corneringStiffnessFront - ...
     frontArm / corneringStiffnessRear);
yawRate = speed / vehicle.wheelbase * delta / (1 + understeerTerm);
expectedFrontForce = vehicle.totalMass * speed * yawRate * ...
    rearArm / vehicle.wheelbase;
expectedRearForce = vehicle.totalMass * speed * yawRate * ...
    frontArm / vehicle.wheelbase;
rearSlip = expectedRearForce / corneringStiffnessRear;
vy = rearArm * yawRate - speed * rearSlip;

[~, axleForces] = evaluatePlanarTires( ...
    vehicle, tire, simulator, speed, vy, yawRate, delta);
verifyEqual(testCase, axleForces.front, expectedFrontForce, 'RelTol', 0.02);
verifyEqual(testCase, axleForces.rear, expectedRearForce, 'RelTol', 0.02);
verifyEqual(testCase, axleForces.front + axleForces.rear, ...
    vehicle.totalMass * speed * yawRate, 'RelTol', 0.02);
end

function testPlanarTireResponseIsMirrorSymmetric(testCase)
[vehiclePositive, tirePositive, simulatorPositive] = createPlanarFixture(0);
[vehicleNegative, tireNegative, simulatorNegative] = createPlanarFixture(0);
speed = 15;
vy = 0.30;
yawRate = 0.20;
steer = 0.04;

[positive, ~] = evaluatePlanarTires(vehiclePositive, tirePositive, ...
    simulatorPositive, speed, vy, yawRate, steer);
[negative, ~] = evaluatePlanarTires(vehicleNegative, tireNegative, ...
    simulatorNegative, speed, -vy, -yawRate, -steer);

forceScale = max([abs(positive.sumFxBody), abs(positive.sumFyBody), 1]);
momentScale = max(abs(positive.yawMoment), 1);
verifyEqual(testCase, negative.sumFxBody, positive.sumFxBody, ...
    'AbsTol', 1e-9 * forceScale);
verifyEqual(testCase, negative.sumFyBody, -positive.sumFyBody, ...
    'AbsTol', 1e-9 * forceScale);
verifyEqual(testCase, negative.yawMoment, -positive.yawMoment, ...
    'AbsTol', 1e-9 * momentScale);
end

function testAerodynamicDragNeverAddsPower(testCase)
mass = 250;
vehicle = lts.vehicle.VehicleManager([], [], [], [], []);
vehicle.totalMass = mass;
vehicle.yawInertia = 100;
vehicle.wheelbase = 1.6;
vehicle.staticFrontWeight = 0.5;
vehicle.airDensity = 1.2;
aero = lts.components.Aero.WholeCarAero(0, 0, 0, 1.4, 0);
simulator = lts.simulation.Simulator(vehicle, [], 0.001);
tireData = struct('sumFxBody', 0, 'sumFyBody', 0, 'yawMoment', 0);
velocities = [20 0; -20 0; 0 12; 8 -15; -7 -11];

for idx = 1:size(velocities, 1)
    vx = velocities(idx, 1);
    vy = velocities(idx, 2);
    state = lts.simulation.VehicleState( ...
        'speed', hypot(vx, vy), 'vx', vx, 'vy', vy);
    state.vehicleManager = vehicle;
    aeroForces = aero.computeForces(state);
    dynamics = simulator.computePlanarDynamics(state, tireData, aeroForces);
    dragPower = dynamics.dragForceX * vx + dynamics.dragForceY * vy;
    verifyLessThanOrEqual(testCase, dragPower, 0);
    verifyEqual(testCase, dragPower, ...
        -aeroForces.F_drag * state.speed, 'RelTol', 1e-12);
end
end

function testZeroDriveBrakingCannotIncreaseMechanicalEnergy(testCase)
[vehicle, tire, simulator] = createPlanarFixture(0.015);
speed = 20;
corners = {tire.FL, tire.FR, tire.RL, tire.RR};
for idx = 1:numel(corners)
    corners{idx}.angularVelocity = speed / corners{idx}.wheelRadius;
end
state = lts.simulation.VehicleState( ...
    's', 0, 'speed', speed, 'vx', speed, 'vy', 0, ...
    'yaw', 0, 'yawRate', 0, 'x', 0, 'y', 0);
state.vehicleManager = vehicle;
input = struct('throttle', 0, 'brake', 0.05, 'steer', 0);
ref = struct('heading', 0, 'x', 0, 'y', 0, 's', 0, ...
    'mu', 1, 'curvature', 0, 'referenceMode', 'free');
energy = zeros(201, 1);
energy(1) = planarMechanicalEnergy(vehicle, tire, state);

for idx = 1:200
    [state, ~] = simulator.step(state, input, ref);
    energy(idx + 1) = planarMechanicalEnergy(vehicle, tire, state);
end

numericalTolerance = 1e-8 * energy(1);
verifyLessThanOrEqual(testCase, max(diff(energy)), numericalTolerance);
verifyLessThan(testCase, energy(end), energy(1));
end

function stiffness = centralLateralStiffness(tire, normalLoad, alpha)
positiveForce = tire.computeLateralForce(normalLoad, alpha, 1);
negativeForce = tire.computeLateralForce(normalLoad, -alpha, 1);
stiffness = (positiveForce - negativeForce) / (2 * alpha);
end

function [tireData, axleForces] = evaluatePlanarTires( ...
        vehicle, tire, simulator, speed, vy, yawRate, steer)
state = lts.simulation.VehicleState( ...
    'speed', hypot(speed, vy), 'vx', speed, 'vy', vy, ...
    'yaw', 0, 'yawRate', yawRate, 'x', 0, 'y', 0, 'steer', steer);
state.vehicleManager = vehicle;
kin = simulator.getCornerKinematics(steer);
kin.FL.steerAngle = steer;
kin.FR.steerAngle = steer;
contact = simulator.computePlanarTireContactData(state, kin);
corners = {tire.FL, tire.FR, tire.RL, tire.RR};
for idx = 1:numel(corners)
    corners{idx}.angularVelocity = ...
        contact.longSpeeds(idx) / corners{idx}.wheelRadius;
end
load = vehicle.totalMass * vehicle.g / 4;
loads = struct('FL', load, 'FR', load, 'RL', load, 'RR', load);
tireData = simulator.updatePlanarTireForces( ...
    state, loads, 0, false, 'steady', contact);

fyBody = zeros(4, 1);
for idx = 1:4
    fyBody(idx) = corners{idx}.Fx * contact.sinWheelHeading(idx) + ...
        corners{idx}.Fy * contact.cosWheelHeading(idx);
end
axleForces = struct( ...
    'front', fyBody(1) + fyBody(2), ...
    'rear', fyBody(3) + fyBody(4));
end

function [vehicle, tire, simulator] = createPlanarFixture(rollingResistance)
mass = 256;
tire = lts.components.Tire.PacejkaTire('43105_18x7.5_10_R25B_7.tir');
tire.relaxationLength = 0;
tire.longitudinalRelaxationLength = 0;
tire.rollingResistanceCoeff = rollingResistance;
tire.bearingDragCoeff = 0;
powertrain = SimulatorZeroPowertrain();
suspension = SimulatorChassisOnlySuspensionSpy( ...
    mass * lts.vehicle.VehicleManager.g / 4);
chassis = SimulatorChassisSpy();
aero = SimulatorZeroAero();
vehicle = lts.vehicle.VehicleManager( ...
    aero, suspension, powertrain, tire, [], chassis, []);
vehicle.totalMass = mass;
vehicle.wheelbase = 1.558;
vehicle.trackWidth = 1.21;
vehicle.cgHeight = 0.3;
vehicle.yawInertia = 130;
vehicle.staticFrontWeight = 0.5;
vehicle.brakeBiasFront = 0.6;
vehicle.brakeForceCoefficient = 0.7;
vehicle.maxSpeed = 80;
simulator = lts.simulation.Simulator(vehicle, [], 0.001);
simulator.wheelSolveIterations = 3;
end

function energy = planarMechanicalEnergy(vehicle, tire, state)
translational = 0.5 * vehicle.totalMass * (state.vx^2 + state.vy^2);
rotational = 0.5 * vehicle.yawInertia * state.yawRate^2;
corners = {tire.FL, tire.FR, tire.RL, tire.RR};
wheelEnergy = 0;
for idx = 1:numel(corners)
    wheelEnergy = wheelEnergy + ...
        0.5 * tire.wheelInertia * corners{idx}.angularVelocity^2;
end
energy = translational + rotational + wheelEnergy;
end
