function diag_relax_inspect(matFile)
% DIAG_RELAX_INSPECT Offline look at a repro_*.mat state log: bandpassed
%   slip/wheel-speed envelope vs time and speed, plus a crude spectrogram.

repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
if nargin < 1
    matFile = fullfile(repoRoot, 'exports', 'diag_relax', 'repro_base.mat');
end
S = load(matFile);
L = S.stateLog;
t = L.time(:);
fs = 1 / mean(diff(t));

sig = L.slipRatio_RL(:);
% Zero-phase bandpass 4-60 Hz to isolate the mode
[b, a] = butter(4, [4 60] / (fs / 2));
xb = filtfilt(b, a, sig - mean(sig));

% Frequency track: dominant FFT peak in sliding 0.4 s windows
win = round(0.4 * fs);
fprintf('  t_mid   speed   peakHz   amp(slip)  amp(omega RL rad/s)\n');
for startIdx = 1:round(win / 2):numel(t) - win
    idx = startIdx:startIdx + win - 1;
    x = xb(idx);
    n = numel(x);
    [f, P] = onepsd(x(:) .* hann(n), fs);
    band = f >= 4 & f <= 70;
    fb = f(band); Pb = P(band);
    [~, mi] = max(Pb);
    om = L.omega_RL(idx);
    fprintf('  %5.2f   %5.1f   %6.1f    %8.5f   %8.4f\n', ...
        t(idx(1)) + 0.2, mean(L.speed(idx)), fb(mi), ...
        0.5 * (max(x) - min(x)), 0.5 * (max(om) - min(om)));
end
end

function [f, P] = onepsd(x, fs)
n = numel(x);
X = fft(x);
nfft = numel(X);
if mod(nfft, 2) == 0
    half = nfft / 2 + 1;
    P = abs(X(1:half)).^2;
    P(2:end - 1) = 2 * P(2:end - 1);
    f = (0:half - 1)' * fs / nfft;
else
    half = (nfft + 1) / 2;
    P = abs(X(1:half)).^2;
    P(2:end) = 2 * P(2:end);
    f = (0:half - 1)' * fs / nfft;
end
end
