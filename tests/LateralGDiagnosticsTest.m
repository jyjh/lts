function tests = LateralGDiagnosticsTest
tests = functiontests(localfunctions);
end

function testMatchingRawAndYawSignalsStayQuiet(testCase)
time = (0:0.01:1).';
speed = 10 * ones(size(time));
yawRate = 0.8 * sin(2 * pi * time);
rawLatG = speed .* yawRate ./ LateralGDiagnostics.g;
steer = atan(rawLatG * LateralGDiagnostics.g * 1.6 ./ speed.^2);

report = LateralGDiagnostics.assessSignals( ...
    time, rawLatG, speed, yawRate, steer, 1.6);

verifyEqual(testCase, report.signMismatchFraction, 0, 'AbsTol', 1e-12);
verifyEqual(testCase, report.magnitudeMismatchFraction, 0, 'AbsTol', 1e-12);
verifyEmpty(testCase, report.messages);
end

function testSignFlippedRawAccelWarns(testCase)
time = (0:0.01:1).';
speed = 12 * ones(size(time));
yawRate = 0.7 * ones(size(time));
rawLatG = -speed .* yawRate ./ LateralGDiagnostics.g;
steer = zeros(size(time));

report = LateralGDiagnostics.assessSignals( ...
    time, rawLatG, speed, yawRate, steer, 1.6);

verifyGreaterThan(testCase, report.signMismatchFraction, 0.9);
verifyTrue(testCase, any(contains(report.messages, 'disagrees in sign')));
end

function testMagnitudeMismatchWarns(testCase)
time = (0:0.01:1).';
speed = 10 * ones(size(time));
yawRate = 0.8 * ones(size(time));
yawLatG = speed .* yawRate ./ LateralGDiagnostics.g;
rawLatG = yawLatG + 1.0;
steer = zeros(size(time));

report = LateralGDiagnostics.assessSignals( ...
    time, rawLatG, speed, yawRate, steer, 1.6);

verifyGreaterThan(testCase, report.magnitudeMismatchFraction, 0.9);
verifyTrue(testCase, any(contains(report.messages, 'much larger')));
end

function testTopMismatchEventsKeepSeparateWindows(testCase)
time = (0:0.1:10).';
rawLatG = zeros(size(time));
yawLatG = zeros(size(time));
steerLatG = zeros(size(time));
speed = 10 * ones(size(time));
rawLatG(21) = 2.0;
rawLatG(22) = 1.9;
rawLatG(81) = -1.5;

events = LateralGDiagnostics.topMismatchEvents( ...
    time, rawLatG, yawLatG, steerLatG, speed, 2, 0.5);

verifyEqual(testCase, numel(events), 2);
verifyEqual(testCase, events(1).time, 2.0, 'AbsTol', 1e-12);
verifyEqual(testCase, events(2).time, 8.0, 'AbsTol', 1e-12);
end
