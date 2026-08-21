function [ayG, detail] = solveSkidpadCapacity( ...
        massKg, loadsN, peakForceN, slipAtPeakDeg, loadTransfer)
%SOLVESKIDPADCAPACITY Max steady-state lateral accel of a bicycle car.
%   Bisection root-find on capacityG(m, ay) - ay = 0. The peak-force
%   envelope is monotonically increasing in ay (more load transfer -> more
%   outside capacity) while the demand ay rises linearly, so a unique
%   crossing exists. Inputs are the envelope arrays (loadsN, peakForceN,
%   slipAtPeakDeg) rather than an envelope struct.
maxEqualLoadMu = max(peakForceN ./ max(loadsN, eps));
upper = max(0.5, 1.5 * maxEqualLoadMu);
while residual(upper) > 0 && upper < 5
    upper = upper * 1.5;
end

lower = 0;
for iter = 1:50
    mid = 0.5 * (lower + upper);
    if residual(mid) >= 0
        lower = mid;
    else
        upper = mid;
    end
end

ayG = lower;
detail = capacityAtAy(ayG);

    function r = residual(ayG)
        d = capacityAtAy(ayG);
        r = d.capacityG - ayG;
    end

    function d = capacityAtAy(ayG)
        g = 9.80665;
        cornerLoads = wsc.bicycleCornerLoads(massKg, ayG, loadTransfer);
        [cornerCapacity, cornerSlipDeg] = wsc.tireCapacityForLoads( ...
            cornerLoads, loadsN, peakForceN, slipAtPeakDeg);

        frontCapacity = cornerCapacity(1) + cornerCapacity(2);
        rearCapacity = cornerCapacity(3) + cornerCapacity(4);
        frontFraction = loadTransfer.staticFrontWeight;
        rearFraction = 1 - frontFraction;
        frontLimitedTotal = frontCapacity / max(frontFraction, eps);
        rearLimitedTotal = rearCapacity / max(rearFraction, eps);

        if frontLimitedTotal <= rearLimitedTotal
            totalCapacity = frontLimitedTotal;
            limitedBy = 'front';
        else
            totalCapacity = rearLimitedTotal;
            limitedBy = 'rear';
        end

        if sum(cornerCapacity) > 0
            meanSlipAngleDeg = sum(cornerSlipDeg .* cornerCapacity) / ...
                sum(cornerCapacity);
        else
            meanSlipAngleDeg = NaN;
        end

        d = struct( ...
            'capacityG', totalCapacity / max(massKg * g, eps), ...
            'totalCapacityN', totalCapacity, ...
            'frontCapacityN', frontCapacity, ...
            'rearCapacityN', rearCapacity, ...
            'cornerCapacityN', cornerCapacity, ...
            'cornerLoadsN', cornerLoads, ...
            'meanSlipAngleDeg', meanSlipAngleDeg, ...
            'limitedBy', limitedBy);
    end
end
