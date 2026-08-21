function [idx, limit] = indexAtMass(targetMassKg, massKg, sweep)
%INDEXATMASS Index of the sweep sample closest to the target mass.
idx = NaN;
[~, found] = min(abs(massKg(:) - targetMassKg));
if isempty(found) || ~isfinite(found)
    limit = '';
    return;
end
idx = found;
limit = char(sweep.limitedBy(idx));
end
