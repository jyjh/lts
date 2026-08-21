function [capacity, slipAngleDeg] = tireCapacityForLoads( ...
        loads, loadsN, peakForceN, slipAtPeakDeg)
%TIRECAPACITYFORLOADS Peak tire force (and slip at peak) at corner loads.
%   Linear interpolation of the envelope with extrapolation, floored at
%   zero; non-positive loads get zero capacity and NaN slip.
capacity = zeros(size(loads));
slipAngleDeg = zeros(size(loads));
minLoad = min(loadsN);
maxLoad = max(loadsN);
for i = 1:numel(loads)
    Fz = loads(i);
    if Fz <= 0
        capacity(i) = 0;
        slipAngleDeg(i) = NaN;
        continue;
    end
    capacity(i) = interp1(loadsN, peakForceN, Fz, 'linear', 'extrap');
    capacity(i) = max(capacity(i), 0);
    clippedFz = min(max(Fz, minLoad), maxLoad);
    slipAngleDeg(i) = interp1(loadsN, slipAtPeakDeg, clippedFz, ...
        'nearest', 'extrap');
end
end
