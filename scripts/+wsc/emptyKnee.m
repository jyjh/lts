function knee = emptyKnee(valueFields)
%EMPTYKNEE All-NaN diminishing-returns knee with the caller's field names.
%   valueFields entries are 'kneeField=sweepField' mappings (see
%   wsc.massSweepKnee); benefits are always included.
knee = struct('method', 'empty', 'index', NaN, 'massKg', NaN);
for i = 1:numel(valueFields)
    [kneeField, ~] = wsc.kneeFieldPair(valueFields{i});
    knee.(kneeField) = NaN;
end
knee.benefitSPerKgSaved = NaN;
knee.benefitGPerKgSaved = NaN;
end
