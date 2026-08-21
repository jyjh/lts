function out = curveLinearity(massKg, signal)
%CURVELINEARITY Quadratic coefficient and linear R^2 of (mass, signal).
%   The quadratic sign tells the curvature direction: for a G-vs-mass sweep,
%   c >= 0 means ACCELERATING returns to lightness (lighter pays off
%   progressively more per kg); for a time-vs-mass sweep, c >= 0 means
%   DIMINISHING returns (each kg saved shaves less time off as the car gets
%   lighter).
mass = massKg(:);
sig = signal(:);
out = struct('quadCoeff', NaN, 'linearFitR2', NaN);
finite = isfinite(mass) & isfinite(sig);
mass = mass(finite);
sig = sig(finite);
if numel(mass) < 3
    return;
end
p2 = polyfit(mass, sig, 2);
out.quadCoeff = p2(1);
p1 = polyfit(mass, sig, 1);
residuals = sig - polyval(p1, mass);
ssRes = sum(residuals.^2);
ssTot = sum((sig - mean(sig)).^2);
if ssTot > eps
    out.linearFitR2 = max(0, 1 - ssRes / ssTot);
else
    out.linearFitR2 = 1;
end
end
