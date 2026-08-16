function diag_relax_repro(sigmaLong, opts)
% DIAG_RELAX_REPRO Reproduce the longitudinal relaxation coupling oscillation.
%   diag_relax_repro(sigmaLong)        — run R25 on straight75 with the given
%                                       longitudinal relaxation length (NaN =
%                                       share the lateral 0.255 m value)
%   diag_relax_repro(sigmaLong, opts)  — opts: .dt, .wheelSolveIterations,
%                                       .trackType, .outTag
%
%   Reports the dominant oscillation frequency and per-window amplitude
%   envelope of wheel speed, slip ratio, pitch, and rear load during the
%   sustained-acceleration phase, plus saves the full state log to
%   exports/diag_relax/ for offline inspection.

if nargin < 2
    opts = struct();
end
dt = localOpt(opts, 'dt', 0.001);
iters = localOpt(opts, 'wheelSolveIterations', 2);
trackType = localOpt(opts, 'trackType', 'straight75');
outTag = localOpt(opts, 'outTag', 'base');
sigmaFz = localOpt(opts, 'normalLoadRelaxationLength', 0);

repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));
outDir = fullfile(repoRoot, 'exports', 'diag_relax');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

track = lts.components.TestTrack(trackType);
config = lts.vehicles.R25();
config.tire.longitudinalRelaxationLength = sigmaLong;
config.tire.normalLoadRelaxationLength = sigmaFz;

vehicle = lts.vehicle.VehicleManager.fromConfig(config, track, dt);
driver = lts.driver.DriverModel(vehicle);
simulator = lts.simulation.Simulator(vehicle, driver, dt);
simulator.verbose = false;
simulator.wheelSolveIterations = iters;
simulator.telemetryMode = 'full';

initialState = lts.simulation.VehicleState('s', 0, 'speed', 0.1);
[stateLog, lapTime] = simulator.simulate(initialState, track);

fprintf('\n=== sigmaLong=%g sigmaFz=%g dt=%g iters=%d : lapTime %.3f s, %d steps ===\n', ...
    sigmaLong, sigmaFz, dt, iters, lapTime, numel(stateLog.time));

% Analysis window: sustained forward acceleration, skip launch transient.
t = stateLog.time(:);
mask = t >= 1.0 & t <= min(max(t) - 0.25, 8.0);
if nnz(mask) < 500
    mask = t >= 0.25 & t <= max(t) - 0.1;
end
reportSpectrum('wheelSpeed RL [m/s]', t, stateLog.tireSpeed_RL(:), mask);
if isfield(stateLog, 'slipRatio_RL') && any(stateLog.slipRatio_RL)
    reportSpectrum('slipRatio RL [-]', t, stateLog.slipRatio_RL(:), mask);
end
reportSpectrum('pitchAngle [rad]', t, stateLog.pitchAngle(:), mask);
reportSpectrum('ax [m/s^2]', t, stateLog.ax(:), mask);
if isfield(stateLog, 'Fz_RL') && any(stateLog.Fz_RL)
    reportSpectrum('Fz RL [N]', t, stateLog.Fz_RL(:), mask);
end

save(fullfile(outDir, sprintf('repro_%s.mat', outTag)), ...
    'stateLog', 'lapTime', 'sigmaLong', 'dt', 'iters');
fprintf('saved: %s\n', fullfile(outDir, sprintf('repro_%s.mat', outTag)));
end

function reportSpectrum(name, t, sig, mask)
x = sig(mask);
tt = t(mask);
x = x - mean(x);
% Light detrend: remove best-fit line so drift does not dominate the FFT.
p = polyfit(tt - tt(1), x, 1);
x = x - polyval(p, tt - tt(1));

n = numel(x);
fs = 1 / mean(diff(tt));
[f, P] = localOneSidedPsd(x, fs);
band = f >= 4 & f <= 80;
[~, idx] = max(P(band));
fPeak = f(band);
fPeak = fPeak(idx);

% Envelope: RMS over 0.5 s windows tracks growth/decay of the mode.
win = round(0.5 * fs);
envT = [];
envR = [];
for startIdx = 1:win:n - win + 1
    envT(end+1) = tt(startIdx) + win / (2 * fs); %#ok<AGROW>
    envR(end+1) = rms(x(startIdx:startIdx + win - 1)); %#ok<AGROW>
end

fprintf('%-20s: peak %6.1f Hz | rms env [%.1fs..%.1fs]: %s\n', ...
    name, fPeak, envT(1), envT(end), ...
    strjoin(compose('%#.4g', envR(1:min(end, 6))), ' '));
end

function [f, P] = localOneSidedPsd(x, fs)
n = numel(x);
w = hann(n);
X = fft((x - mean(x)) .* w);
nfft = numel(X);
if mod(nfft, 2) == 0
    half = nfft / 2 + 1;
    P = abs(X(1:half)).^2 / (fs * sum(w.^2));
    P(2:end - 1) = 2 * P(2:end - 1);
    f = (0:half - 1)' * fs / nfft;
else
    half = (nfft + 1) / 2;
    P = abs(X(1:half)).^2 / (fs * sum(w.^2));
    P(2:end) = 2 * P(2:end);
    f = (0:half - 1)' * fs / nfft;
end
end

function value = localOpt(opts, name, defaultValue)
if isfield(opts, name) && ~isempty(opts.(name))
    value = opts.(name);
else
    value = defaultValue;
end
end
