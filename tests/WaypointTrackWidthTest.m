function tests = WaypointTrackWidthTest
% WAYPOINTTRACKWIDTHTEST Variable per-side track width behavior.
% Covers the cone-derived per-waypoint left/right corridor format written by
% the fsae track image tool, and the symmetric scalar-width fallback.
tests = functiontests(localfunctions);
end

function testScalarWidthLoadsAndReportsSymmetricSides(testCase)
% Legacy scalar-width files still load; getTrackSideWidths returns Width/2 on
% each side for every waypoint.
points = [0 0; 10 0; 10 10; 0 10];
track = lts.components.WaypointTrack(points, 'Width', 5.0, 'Closed', true);

verifyEqual(testCase, track.getTrackWidth(), 5.0, 'AbsTol', 1e-12);
[leftWidth, rightWidth] = track.getTrackSideWidths();
verifyEqual(testCase, leftWidth, repmat(2.5, size(points, 1), 1), 'AbsTol', 1e-12);
verifyEqual(testCase, rightWidth, repmat(2.5, size(points, 1), 1), 'AbsTol', 1e-12);
verifyTrue(testCase, isempty(track.LeftWidth));
verifyTrue(testCase, isempty(track.RightWidth));
end

function testPerWaypointAsymmetricWidthsAreStoredAndReturned(testCase)
points = [0 0; 10 0; 10 10; 0 10];
leftWidth = [1.0; 2.0; 3.0; 2.5];
rightWidth = [4.0; 3.0; 1.5; 2.0];
track = lts.components.WaypointTrack(points, ...
    'LeftWidth', leftWidth, 'RightWidth', rightWidth, 'Closed', true);

[leftGot, rightGot] = track.getTrackSideWidths();
verifyEqual(testCase, leftGot, leftWidth(:), 'AbsTol', 1e-12);
verifyEqual(testCase, rightGot, rightWidth(:), 'AbsTol', 1e-12);
% Representative total width is the mean of (left + right).
verifyEqual(testCase, track.getTrackWidth(), mean(leftWidth + rightWidth), ...
    'AbsTol', 1e-12);
end

function testRejectsWidthVectorWithWrongLength(testCase)
points = [0 0; 10 0; 10 10];
verifyError(testCase, @() lts.components.WaypointTrack(points, ...
    'Width', [3 4], 'Closed', true), 'WaypointTrack:InvalidWidth');
end

function testRejectsOnlyOneSideWidthSupplied(testCase)
points = [0 0; 10 0; 10 10];
verifyError(testCase, @() lts.components.WaypointTrack(points, ...
    'LeftWidth', [1; 2; 3], 'Closed', true), ...
    'WaypointTrack:IncompleteSideWidths');
end

function testRejectsSideWidthsWrongLength(testCase)
points = [0 0; 10 0; 10 10];
verifyError(testCase, @() lts.components.WaypointTrack(points, ...
    'LeftWidth', [1; 2], 'RightWidth', [3; 4], 'Closed', true), ...
    'WaypointTrack:InvalidSideWidths');
end

function testSaveLoadRoundTripsPerSideWidths(testCase)
points = [0 0; 10 0; 10 10; 0 10];
leftWidth = [1.0; 2.0; 3.0; 2.5];
rightWidth = [4.0; 3.0; 1.5; 2.0];
track = lts.components.WaypointTrack(points, ...
    'LeftWidth', leftWidth, 'RightWidth', rightWidth, ...
    'Closed', true, 'Name', 'AsymRoundTrip');

fileName = [tempname '.mat'];
track.saveMat(fileName);
onCleanup(@() delete(fileName));

loaded = lts.components.WaypointTrack.loadMat(fileName);
[leftGot, rightGot] = loaded.getTrackSideWidths();
verifyEqual(testCase, leftGot, leftWidth(:), 'AbsTol', 1e-12);
verifyEqual(testCase, rightGot, rightWidth(:), 'AbsTol', 1e-12);
verifyEqual(testCase, loaded.getTrackPoints(), points, 'AbsTol', 1e-12);
end

function testSaveLoadRoundTripsScalarWidth(testCase)
points = [0 0; 10 0; 10 10; 0 10];
track = lts.components.WaypointTrack(points, 'Width', 4.0, 'Closed', true);

fileName = [tempname '.mat'];
track.saveMat(fileName);
onCleanup(@() delete(fileName));

loaded = lts.components.WaypointTrack.loadMat(fileName);
verifyEqual(testCase, loaded.getTrackWidth(), 4.0, 'AbsTol', 1e-12);
verifyTrue(testCase, isempty(loaded.LeftWidth));
[leftGot, rightGot] = loaded.getTrackSideWidths();
verifyEqual(testCase, leftGot, repmat(2.0, size(points, 1), 1), 'AbsTol', 1e-12);
end

function testLoadMatSwapsLeftRightWhenDirectionReverses(testCase)
% The exporter bakes the requested travel direction into points_m ordering.
% Reversing travel direction (via a Direction override that conflicts with the
% stored winding) must swap left/right half-widths, because the side that was
% on the car's left is now on its right. Build a clockwise track, store a
% 'clockwise' direction, then force anticlockwise at load.
points = [0 0; 10 0; 10 -10; 0 -10];  % clockwise square (y-down area negative)
leftWidth = [1.0; 2.0; 3.0; 2.5];
rightWidth = [4.0; 3.0; 1.5; 2.0];
stored = lts.components.WaypointTrack(points, ...
    'LeftWidth', leftWidth, 'RightWidth', rightWidth, 'Closed', true, ...
    'Metadata', struct('direction', 'clockwise'));
fileName = [tempname '.mat'];
stored.saveMat(fileName);
onCleanup(@() delete(fileName));

% Confirm the stored winding is clockwise before flipping.
verifyEqual(testCase, ...
    lts.components.WaypointTrack.windingFromPoints(points), 'clockwise');

loaded = lts.components.WaypointTrack.loadMat(fileName, ...
    'Direction', 'anticlockwise');
verifyTrue(testCase, loaded.Metadata.resolved_direction_flipped);

% After reversal, the half-width that bounded the left side now bounds the
% right side. reversePreserveStart keeps the start point fixed; the swapped
% arrays must match the reversePreserveStart-reordered opposite side.
revLeft = lts.components.WaypointTrack.reversePreserveStart(leftWidth, true);
revRight = lts.components.WaypointTrack.reversePreserveStart(rightWidth, true);
[leftGot, rightGot] = loaded.getTrackSideWidths();
verifyEqual(testCase, leftGot, revRight, 'AbsTol', 1e-12);
verifyEqual(testCase, rightGot, revLeft, 'AbsTol', 1e-12);
end

function testSideMarginHonorsSignOfLateralError(testCase)
% Positive lateralError (left of line) is bounded by the left half-width;
% negative by the right. A narrow left / wide right corridor must flag
% off-track on the left before the right.
trackData = struct( ...
    'trackHalfWidth', 2.5, ...
    'trackLeftHalfWidth', (1.0)', ...
    'trackRightHalfWidth', (4.0)', ...
    'arcLen', (0:3)', ...
    'nPts', 4);
marginLeft = lts.simulation.TrackReference.sideMargin(1.0, 4.0, 0.5, 2.5);
marginRight = lts.simulation.TrackReference.sideMargin(1.0, 4.0, -0.5, 2.5);
verifyEqual(testCase, marginLeft, 0.5, 'AbsTol', 1e-12);  % 1.0 - 0.5
verifyEqual(testCase, marginRight, 3.5, 'AbsTol', 1e-12);  % 4.0 - 0.5

% At 1.2 m left the car is off the narrow left side but would be on the wide
% right side -- margin must reflect the left (1.0) bound.
verifyLessThan(testCase, lts.simulation.TrackReference.sideMargin( ...
    1.0, 4.0, 1.2, 2.5), -1e-9);
end

function testSideMarginFallsBackToScalarWhenNoPerSide(testCase)
% Synthetic trackData with only the scalar field must still produce a
% well-defined symmetric margin (the test/synthetic-struct code path).
margin = lts.simulation.TrackReference.sideMargin([], [], 0.4, 2.0);
verifyEqual(testCase, margin, 1.6, 'AbsTol', 1e-12);
end
