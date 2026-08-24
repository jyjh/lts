function audit_stagger_validation()
% Phase-2 validation: does the attitude predictor remove the ~10-15 Hz Fz
% oscillation, and is normalLoadRelaxationLength still needed with it on?
%
% Runs a straight-line hard-braking event (the historically oscillation-prone
% scenario) with three configurations and reports per-corner Fz oscillation
% energy, dominant frequency, and stopping behavior:
%   A  no predictor, sigmaFz = 0.255   (pre-predictor behavior)
%   B  predictor,    sigmaFz = 0       (predictor alone)
%   C  predictor,    sigmaFz = 0.255   (both)
repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
cd(repoRoot);

configs = {
    'A no-pred sigmaFz=0.255', false, 0.255;
    'B pred     sigmaFz=0    ', true,  0;
    'C pred     sigmaFz=0.255', true,  0.255};

scenarios = { ...
    'brake', 25, struct('throttle', 0, 'brake', 1, 'steer', 0), 0.5; ...
    'turn  ', 25, struct('throttle', 0.4, 'brake', 0, 'steer', 0.12), 0.0};

dt = 0.001;
v0 = 25;           % m/s
brakeStart = 0.5;  % s
duration = 3.0;    % s

for sIdx = 1:size(scenarios, 1)
    scenarioName = scenarios{sIdx, 1};
    v0 = scenarios{sIdx, 2};
    baseInput = scenarios{sIdx, 3};
    inputTime = scenarios{sIdx, 4};
    fprintf('--- scenario: %s (v0=%g, steer=%g, throttle=%g) ---\n', ...
        scenarioName, v0, baseInput.steer, baseInput.throttle);
for c = 1:size(configs, 1)
    cfg = lts.vehicles.R25();
    vehicle = lts.vehicle.VehicleManager.fromConfig(cfg, [], dt, 'Verbose', false);
    vehicle.tire.normalLoadRelaxationLength = configs{c, 3};

    sim = lts.simulation.Simulator(vehicle, [], dt);
    sim.useAttitudePredictor = configs{c, 2};
    sim.verbose = false;

    state = lts.simulation.VehicleState('speed', v0, 'vx', v0, 'vy', 0, ...
        'yaw', 0, 'x', 0, 'y', 0, 'mu', 1);
    state.vehicleManager = vehicle;
    ref = straightRef();
    input = struct('throttle', 0, 'brake', 0, 'steer', 0);

    nSteps = round(duration / dt);
    fz = nan(nSteps, 4);
    speed = nan(nSteps, 1);
    initializeWheelSpeedsR25(vehicle.tire, v0);

    for k = 1:nSteps
        t = (k - 1) * dt;
        if t >= inputTime
            input = baseInput;
        end
        [state, ~] = sim.step(state, input, ref);
        fz(k, :) = [vehicle.tire.FL.normalForce, vehicle.tire.FR.normalForce, ...
                    vehicle.tire.RL.normalForce, vehicle.tire.RR.normalForce];
        speed(k) = state.speed;
        if ~isfinite(state.speed) || state.speed < 0.5
            fz(k+1:end, :) = [];
            speed(k+1:end) = [];
            break;
        end
    end

    % Analyze in two sub-windows after the input onset: early (transient)
    % and late (sustained behavior). Band energy 5-40 Hz isolates the hop
    % mode from low-frequency load-transfer drift.
    tAxis = (0:numel(speed)-1)' * dt;
    brakeMask = tAxis >= inputTime & tAxis <= inputTime + 2.4;
    earlyMask = tAxis >= inputTime + 0.05 & tAxis <= inputTime + 1.0;
    lateMask = tAxis >= inputTime + 1.4 & tAxis <= inputTime + 2.4;
    band = @(r) bandEnergy(r, dt, 5, 40);
    oscEarly = zeros(1, 4);
    oscLate = zeros(1, 4);
    domFreq = 0;
    for corner = 1:4
        oscEarly(corner) = band(detrend(fz(earlyMask, corner), 3));
        oscLate(corner) = band(detrend(fz(lateMask, corner), 3));
        if corner == 3
            r = detrend(fz(brakeMask, corner), 3);
            n = numel(r);
            f = (0:n-1)' / (n * dt);
            sp = abs(fft(r - mean(r)));
            [~, imax] = max(sp(2:floor(n/2)));
            domFreq = f(imax + 1);
        end
    end
    fprintf(['%s | early 5-40Hz rms FL/RL: %6.2f %6.2f N | late rms FL/RL: %6.2f %6.2f N | ' ...
             'dom %.1f Hz | final speed %.2f m/s\n'], ...
        configs{c, 1}, oscEarly(1), oscEarly(3), oscLate(1), oscLate(3), ...
        domFreq, speed(end));
end
end
end

function r = detrend(y, order)
tt = (0:numel(y)-1)';
p = polyfit(tt, y, order);
r = y - polyval(p, tt);
end

function rms = bandEnergy(y, dt, fLow, fHigh)
% RMS of the signal's content between fLow and fHigh Hz.
n = numel(y);
y = y(:) - mean(y);
sp = fft(y);
f = (0:n-1)' / (n * dt);
sel = f >= fLow & f <= fHigh & f <= (0.5 / dt) - fHigh;
power = 2 * sum(abs(sp(sel)).^2) / n^2;
rms = sqrt(power);
end

function ref = straightRef()
trackData = struct( ...
    'points', [0 0; 1000 0], ...
    'arcLen', [0; 1000], ...
    'heading', [0; 0], ...
    'curvature', [0; 0], ...
    'mu', [1.2; 1.2], ...
    'length', 1000, ...
    'trackWidth', 3, ...
    'trackHalfWidth', 1.5, ...
    'nPts', 2);
ref = struct('heading', 0, 'x', 0, 'y', 0, 'idx', 1, 'trackData', trackData);
end

function initializeWheelSpeedsR25(tire, speed)
corners = {tire.FL, tire.FR, tire.RL, tire.RR};
for i = 1:4
    corners{i}.angularVelocity = speed / corners{i}.wheelRadius;
end
end
