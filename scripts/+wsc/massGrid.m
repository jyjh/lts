function massKg = massGrid(massRange, massStepKg)
%MASSGRID Column mass vector from [min max] and a step, endpoint-inclusive.
massKg = (massRange(1):massStepKg:massRange(2)).';
if isempty(massKg)
    massKg = [massRange(1); massRange(2)];
elseif massKg(end) < massRange(2)
    massKg(end + 1, 1) = massRange(2);
end
massKg = unique(massKg, 'stable');
end
