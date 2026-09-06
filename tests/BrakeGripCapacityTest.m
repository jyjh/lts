function tests = BrakeGripCapacityTest
tests = functiontests(localfunctions);
end

function testCapacityEqualsBiasWeightedLongitudinalGripLimit(testCase)
    assumeTrue(testCase, tireDataAvailable(), 'TTC tire data not present: see src/+lts/+components/+Tire/README.md');
[vehicle, tire] = createVehicle();
frontLoad = 3500;
rearLoad = 2500;
bias = vehicle.brakeBiasFront;
expected = min( ...
    tire.getPeakLongitudinalFriction(frontLoad / 2) * frontLoad / bias, ...
    tire.getPeakLongitudinalFriction(rearLoad / 2) * rearLoad / (1 - bias));

capacity = lts.simulation.BrakeForcePolicy.gripLimitedCapacity( ...
    vehicle, frontLoad, rearLoad);

verifyEqual(testCase, capacity, expected, 'RelTol', 1e-12);
end

function testCapacityIsGripLimitedNotAFixedLoadFraction(testCase)
    assumeTrue(testCase, tireDataAvailable(), 'TTC tire data not present: see src/+lts/+components/+Tire/README.md');
% The former fictional cap was 0.7 x total load. A race tire's
% longitudinal peak sits well above 0.7/bias, so at low speed (no
% downforce, static loads) the capacity must exceed that cap.
[vehicle, ~] = createVehicle();
W = vehicle.totalMass * vehicle.g;

capacity = lts.simulation.BrakeForcePolicy.gripLimitedCapacity( ...
    vehicle, W * vehicle.staticFrontWeight, W * (1 - vehicle.staticFrontWeight));

verifyGreaterThan(testCase, capacity, 0.7 * W);
end

function testCapacityTracksTheCriticalAxleUnderBiasChanges(testCase)
    assumeTrue(testCase, tireDataAvailable(), 'TTC tire data not present: see src/+lts/+components/+Tire/README.md');
[vehicle, ~] = createVehicle();
frontLoad = 3500;
rearLoad = 2500;

vehicle.brakeBiasFront = 0.6;
capacityBalanced = lts.simulation.BrakeForcePolicy.gripLimitedCapacity( ...
    vehicle, frontLoad, rearLoad);
% More front bias leans harder on the (loaded) front axle.
vehicle.brakeBiasFront = 0.75;
capacityFrontHeavy = lts.simulation.BrakeForcePolicy.gripLimitedCapacity( ...
    vehicle, frontLoad, rearLoad);
% Biasing toward the rear axle flips the rear into the critical role.
vehicle.brakeBiasFront = 0.35;
capacityRearHeavy = lts.simulation.BrakeForcePolicy.gripLimitedCapacity( ...
    vehicle, frontLoad, rearLoad);

verifyLessThan(testCase, capacityFrontHeavy, capacityBalanced);
verifyLessThan(testCase, capacityRearHeavy, capacityBalanced);
end

function testFullRatioCommandPutsCriticalAxleAtItsGripLimit(testCase)
    assumeTrue(testCase, tireDataAvailable(), 'TTC tire data not present: see src/+lts/+components/+Tire/README.md');
[vehicle, tire] = createVehicle();
frontLoad = 3500;
rearLoad = 2500;
input = struct('throttle', 0, 'brake', 1.0, 'steer', 0);

brakeForces = lts.simulation.BrakeForcePolicy.compute( ...
    input, frontLoad, rearLoad, vehicle, "ratio");

bias = vehicle.brakeBiasFront;
frontGrip = tire.getPeakLongitudinalFriction(frontLoad / 2) * frontLoad;
rearGrip = tire.getPeakLongitudinalFriction(rearLoad / 2) * rearLoad;
criticalAxleGrip = min(frontGrip / bias, rearGrip / (1 - bias));
verifyEqual(testCase, brakeForces.frontForce + brakeForces.rearForce, ...
    criticalAxleGrip, 'RelTol', 1e-9);
% The bias splits the total; the critical axle sits exactly at its own
% grip limit, the other axle below its own.
verifyEqual(testCase, brakeForces.frontForce, min(frontGrip, ...
    criticalAxleGrip * bias), 'RelTol', 1e-9);
end

function testFullBrakeRunDeceleratesAtTheGripLimit(testCase)
    assumeTrue(testCase, tireDataAvailable(), 'TTC tire data not present: see src/+lts/+components/+Tire/README.md');
[vehicle, tire] = createVehicle();
speed = 20;
corners = {tire.FL, tire.FR, tire.RL, tire.RR};
for idx = 1:numel(corners)
    corners{idx}.angularVelocity = speed / corners{idx}.wheelRadius;
end
state = lts.simulation.VehicleState( ...
    's', 0, 'speed', speed, 'vx', speed, 'vy', 0, ...
    'yaw', 0, 'yawRate', 0, 'x', 0, 'y', 0);
state.vehicleManager = vehicle;
input = struct('throttle', 0, 'brake', 1.0, 'steer', 0);
ref = struct('heading', 0, 'x', 0, 'y', 0, 's', 0, ...
    'mu', 1, 'curvature', 0, 'referenceMode', 'free');
simulator = lts.simulation.Simulator(vehicle, [], 0.001);
simulator.wheelSolveIterations = 3;

% Expectation from the policy capacity at static loads; braking transfer
% moves load forward, so allow tolerance around it.
W = vehicle.totalMass * vehicle.g;
capacity = lts.simulation.BrakeForcePolicy.gripLimitedCapacity( ...
    vehicle, W * vehicle.staticFrontWeight, W * (1 - vehicle.staticFrontWeight));
expectedDecel = capacity / vehicle.totalMass;

nSteps = 500;
recordStart = 100;
speeds = zeros(nSteps, 1);
for idx = 1:nSteps
    [state, ~] = simulator.step(state, input, ref);
    speeds(idx) = state.speed;
end
measuredDecel = (speeds(recordStart) - speeds(end)) ...
    / ((nSteps - recordStart) * 0.001);

verifyGreaterThan(testCase, measuredDecel, 0.85 * expectedDecel);
verifyLessThan(testCase, measuredDecel, 1.30 * expectedDecel);
% And it must clearly beat the former fictional 0.7g cap.
verifyGreaterThan(testCase, measuredDecel, 7.5);
end

function [vehicle, tire] = createVehicle()
tire = lts.components.Tire.PacejkaTire('43105_18x7.5_10_R25B_7.tir');
tire.relaxationLength = 0;
powertrain = SimulatorZeroPowertrain();
suspension = SimulatorChassisOnlySuspensionSpy(256 * 9.80665 / 4);
chassis = SimulatorChassisSpy();
aero = SimulatorZeroAero();

vehicle = lts.vehicle.VehicleManager(aero, suspension, powertrain, tire, [], chassis, []);
vehicle.totalMass = 256;
vehicle.wheelbase = 1.558;
vehicle.trackWidth = 1.21;
vehicle.cgHeight = 0.3;
vehicle.yawInertia = 130;
vehicle.staticFrontWeight = 0.5;
vehicle.brakeBiasFront = 0.6;
vehicle.maxSpeed = 80;
end
