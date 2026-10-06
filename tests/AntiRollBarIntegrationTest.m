function tests = AntiRollBarIntegrationTest
tests = functiontests(localfunctions);
end

function testCoarseVehicleStepsMatchResolvedCornering(testCase)
assumeTrue(testCase, tireDataAvailable(), 'TTC tire data unavailable.');
coarse = corneringResponse(.01);
resolved = corneringResponse(.001);
fine = corneringResponse(.00025);
verifyGreaterThan(testCase, coarse.peakRoll, 1e-5);
verifyEqual(testCase, coarse.peakRoll, resolved.peakRoll, 'AbsTol', 1e-10);
verifyEqual(testCase, coarse.finalState, resolved.finalState, 'AbsTol', 1e-9);
verifyEqual(testCase, resolved.peakRoll, fine.peakRoll, 'RelTol', .03);
verifyEqual(testCase, coarse.dt, .01);
verifyEqual(testCase, coarse.time, .3, 'AbsTol', 1e-12);
end

function result = corneringResponse(dt)
track = lts.components.TestTrack('straight10');
cfg = lts.vehicles.baseline();
vehicle = lts.vehicle.VehicleManager.fromConfig(cfg, track, dt, 'Verbose', false);
sim = lts.simulation.Simulator(vehicle, [], dt);
sim.applySteeringSlew = false;
state = lts.simulation.VehicleState('speed', 12, 'vx', 12, 'vy', 0, ...
    'yaw', 0, 'x', 0, 'y', 0);
state.vehicleManager = vehicle;
for unit = {vehicle.tire.FL, vehicle.tire.FR, vehicle.tire.RL, vehicle.tire.RR}
    unit{1}.angularVelocity = 12 / unit{1}.wheelRadius;
end
ref = sim.freeReferenceForState(0, 0, 0, 0);
input = struct('throttle', 0, 'brake', 0, 'steer', .05);
peak = 0;
for idx = 1:round(.3 / dt)
    [state, ~] = sim.step(state, input, ref);
    ref = sim.cachedNextRef;
    % Compare peaks on the same 10 ms observation grid.
    if abs(state.time / .01 - round(state.time / .01)) < 1e-8
        peak = max(peak, abs(vehicle.chassis.state.frontRollAngle));
    end
end
result = struct('peakRoll', peak, 'dt', sim.dt, 'time', state.time, ...
    'finalState', [state.x, state.y, state.vx, state.vy, state.yawRate, ...
    vehicle.chassis.state.frontRollAngle, vehicle.chassis.state.rearRollAngle]);
end
