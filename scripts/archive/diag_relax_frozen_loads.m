function diag_relax_frozen_loads()
% DIAG_RELAX_FROZEN_LOADS Ablation: run with the chassis->Fz load path
%   frozen at static loads (no pitch/roll load transfer at all) to test
%   whether the one-step attitude/load stagger feeds the oscillation.

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(repoRoot, 'src'));
addpath(fileparts(mfilename('fullpath'))); % archive/ holds FrozenLoadSimulator

track = lts.components.TestTrack('straight75');
config = lts.vehicles.R25();
config.tire.longitudinalRelaxationLength = NaN;  % shared 0.255 m

vehicle = lts.vehicle.VehicleManager.fromConfig(config, track, 0.001);
driver = lts.driver.DriverModel(vehicle);
simulator = FrozenLoadSimulator(vehicle, driver, 0.001);
simulator.verbose = false;
simulator.wheelSolveIterations = 2;
simulator.mode = 'advanceOnly';  % 'static' = full freeze

initialState = lts.simulation.VehicleState('s', 0, 'speed', 0.1);
[stateLog, lapTime] = simulator.simulate(initialState, track);

fprintf('\n=== MODE %s, sigmaLong=NaN : lapTime %.3f s ===\n', simulator.mode, lapTime);
t = stateLog.time(:);
mask = t >= 1.0 & t <= min(max(t) - 0.25, 8.0);
reportSpectrum('wheelSpeed RL [m/s]', t, stateLog.tireSpeed_RL(:), mask);
reportSpectrum('slipRatio RL [-]', t, stateLog.slipRatio_RL(:), mask);
reportSpectrum('pitchAngle [rad]', t, stateLog.pitchAngle(:), mask);
end

function reportSpectrum(name, t, sig, mask)
x = sig(mask); tt = t(mask);
p = polyfit(tt - tt(1), x, 1);
x = x - polyval(p, tt - tt(1));
fs = 1 / mean(diff(tt));
[f, P] = onepsd(x(:) .* hann(numel(x)), fs);
band = f >= 4 & f <= 80;
fb = f(band); Pb = P(band);
[~, mi] = max(Pb);
fprintf('%-20s: peak %6.1f Hz | rms %.5g\n', name, fb(mi), rms(x));
end

function [f, P] = onepsd(x, fs)
X = fft(x(:));
nfft = numel(X);
if mod(nfft, 2) == 0
    half = nfft / 2 + 1;
    P = abs(X(1:half)).^2; P(2:end - 1) = 2 * P(2:end - 1);
else
    half = (nfft + 1) / 2;
    P = abs(X(1:half)).^2; P(2:end) = 2 * P(2:end);
end
f = (0:half - 1)' * fs / nfft;
end
