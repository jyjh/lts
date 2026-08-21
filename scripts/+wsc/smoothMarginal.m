function out = smoothMarginal(derivative, nMass)
%SMOOTHMARGINAL Suppress float-noise oscillation in a numerical derivative.
%   Numerical derivative of a near-constant curve is float-noise dominated.
%   Centered moving average + coarse round to suppress spurious oscillation.
out = wsc.movingAverage(derivative, max(5, ceil(nMass / 20)));
out = round(out, 6);
end
