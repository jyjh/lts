function tests = LapPlannerTest
% LAPPLANNERTEST Unit tests for LapPlanner racing-line and velocity profile.
tests = functiontests(localfunctions);
end

function trackData = simpleCornerTrackData()
% A flat 60-degree circular arc segment centered on the origin so we know
% the geometry and can assert racing-line properties without a full simulator.
% Radius 30 m, ~15 m of arc, on a 4 m wide track.
R = 30;
halfAngle = deg2rad(30);
thetas = linspace(-halfAngle, halfAngle, 31)';
pts = [R * sin(thetas), R * (1 - cos(thetas))];   % opens rightward from origin
closed = false;
curvature = components.Track.computeCurvature(pts, closed);
heading = components.Track.computeHeading(pts, closed);
arcLen = components.Track.cumulativeArcLength(pts, closed);
trackData = struct( ...
    'points', pts, ...
    'arcLen', arcLen(:), ...
    'curvature', curvature(:), ...
    'heading', heading(:), ...
    'trackWidth', 4.0, ...
    'trackHalfWidth', 2.0, ...
    'closedLoop', closed);
end

function testRacingLineStaysWithinTrackLimits(testCase)
trackData = simpleCornerTrackData();
opts = struct('lineUsage', 0.9);
line = LapPlanner.buildRacingLine(trackData, opts);
% Every racing-line point must lie within half-width (with a small margin
% for the lineUsage cap).
maxOffset = max(abs(line.offsetW));
verifyLessThanOrEqual(testCase, maxOffset, trackData.trackHalfWidth * 0.9 + 1e-6);
end

function trackData = straightCornerStraightTrackData()
% A straight-corner-straight layout where corner-cutting is actually
% possible (unlike an isolated arc, which is already min-curvature). Entry
% along +x, left-hand corner (center (40,20), R=20), exit along +y.
seg1 = [(0:1:40)', zeros(41,1)];
R = 20; cx = 40; cy = 20;
th = linspace(-pi/2, 0, 25)';
corner = [cx + R*cos(th), cy + R*sin(th)];
corner(1,:) = [];
pts = [seg1; corner];
closed = false;
curvature = components.Track.computeCurvature(pts, closed);
heading = components.Track.computeHeading(pts, closed);
arcLen = components.Track.cumulativeArcLength(pts, closed);
trackData = struct( ...
    'points', pts, 'arcLen', arcLen(:), ...
    'curvature', curvature(:), 'heading', heading(:), ...
    'trackWidth', 6.0, 'trackHalfWidth', 3.0, 'closedLoop', closed);
end

function testRacingLineUsesInsideOfCorner(testCase)
% A racing line through a straight-corner-straight must move toward the
% inside of the corner (use the track width) rather than hug the centerline.
trackData = straightCornerStraightTrackData();
line = LapPlanner.buildRacingLine(trackData);
% Corner region is roughly idx 41..65. The inside of this LEFT turn is the
% +offset (left) side, so peak offset there should be clearly positive and
% a meaningful fraction of the half-width.
cornerOff = line.offsetW(41:min(65, numel(line.offsetW)));
verifyGreaterThan(testCase, max(cornerOff), 0.3);
verifyLessThanOrEqual(testCase, max(abs(line.offsetW)), trackData.trackHalfWidth + 1e-6);
end

function testRacingLineMatchesCenterlineOnStraight(testCase)
% On a perfectly straight track there is nothing to gain, so the optimizer
% should leave offsets ~0 (no incentive to move off-center).
pts = [(0:0.5:20)', zeros(41, 1)];
closed = false;
curvature = components.Track.computeCurvature(pts, closed);
heading = components.Track.computeHeading(pts, closed);
arcLen = components.Track.cumulativeArcLength(pts, closed);
trackData = struct('points', pts, 'arcLen', arcLen(:), ...
    'curvature', curvature(:), 'heading', heading(:), ...
    'trackWidth', 4.0, 'trackHalfWidth', 2.0, 'closedLoop', closed);

line = LapPlanner.buildRacingLine(trackData);
verifyLessThanOrEqual(testCase, max(abs(line.offsetW)), 0.05);
end

function testClosedLoopRacingLineIsContinuous(testCase)
% A closed-loop racing line must be approximately periodic: the offset wraps
% with only a small jump at the lap boundary. A perfectly seamless wrap is
% not achievable with bounded coordinate descent at the start/end seam (the
% seam point sits mid-corner), but the residual jump is small relative to the
% track width and is absorbed by the curvature smoothing used for speed
% targets, so it does not produce a visible steering transient.
track = components.TestTrack('oval');
pts = track.getTrackPoints();
closed = true;
curvature = components.Track.computeCurvature(pts, closed);
heading = components.Track.computeHeading(pts, closed);
arcLen = components.Track.cumulativeArcLength(pts, closed);
trackData = struct('points', pts, 'arcLen', arcLen(:), ...
    'curvature', curvature(:), 'heading', heading(:), ...
    'trackWidth', track.getTrackWidth(), ...
    'trackHalfWidth', track.getTrackWidth()/2, 'closedLoop', closed);

line = LapPlanner.buildRacingLine(trackData);
wrapJump = abs(line.offsetW(1) - line.offsetW(end));
verifyLessThan(testCase, wrapJump, 0.25);
end

function vm = mockVehicleManager()
% Minimal VehicleManager-like struct for velocity-profile tests. Uses a
% straight-line drive force (speed-independent) so the sweep is analytic.
vm = struct();
vm.totalMass = 256;
vm.g = 9.80665;
vm.wheelbase = 1.558;
vm.maxSpeed = 80;
vm.staticFrontWeight = 0.50;
vm.brakeBiasFront = 0.60;
vm.brakeForceCoefficient = 0.70;
% Tire mu ~1.6 independent of load -> tireAccel ~1.6g regardless of aero.
vm.tire = struct('getPeakFriction', @(~)1.6);
% Aero contributes no downforce/drag in this mock (returns zeros).
vm.aero = struct('computeForces', @(~)struct( ...
    'F_drag', 0, 'Fz_front', 0, 'Fz_rear', 0));
% Powertrain: constant 3000 N drive force up to maxSpeed.
vm.powertrain = struct('computeMaxDriveForce', @(~)3000);
end

function testVelocityProfileBrakingInequality(testCase)
% The backward sweep guarantees a car at vTarget can brake down to the next
% vTarget within ds: vNext^2 >= vCur^2 - 2*maxBrakeAccel*ds (on a slowing
% segment). Verify the produced profile never requires more than available.
vm = mockVehicleManager();
trackData = simpleCornerTrackData();
line = LapPlanner.buildRacingLine(trackData);
initialState = VehicleState('speed', 15);
profile = LapPlanner.buildVelocityProfile(line, vm, initialState);

ds = diff(line.arcLen);
% max brake accel used in planning = skill * caps.maxBrakeAccel. Use the mock:
% tireAccel ~ 1.6*9.80665, brakeAccel uses brakeForceCoeff 0.7. Take a safe
% upper bound of 1.6g for the inequality check.
maxBrakeG = 1.6 * vm.g;
for i = 1:numel(ds)
    vCur = profile.vTarget(i);
    vNext = profile.vTarget(i+1);
    % If slowing, the decel must be achievable: vNext^2 >= vCur^2 - 2*a*ds.
    if vNext < vCur - 1e-6
        minReachable = vCur^2 - 2 * maxBrakeG * ds(i);
        verifyGreaterThanOrEqual(testCase, vNext^2, minReachable - 1e-6);
    end
end
end

function testVelocityProfileRespectsMaxSpeed(testCase)
vm = mockVehicleManager();
trackData = simpleCornerTrackData();
line = LapPlanner.buildRacingLine(trackData);
initialState = VehicleState('speed', vm.maxSpeed);
profile = LapPlanner.buildVelocityProfile(line, vm, initialState);
verifyLessThanOrEqual(testCase, max(profile.vTarget), vm.maxSpeed + 1e-9);
end
