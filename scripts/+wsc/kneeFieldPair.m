function [kneeField, sweepField] = kneeFieldPair(spec)
%KNEEFIELDPAIR Split a 'kneeField=sweepField' mapping; bare names map to
%   themselves.
parts = strsplit(spec, '=');
kneeField = parts{1};
if numel(parts) == 2
    sweepField = parts{2};
else
    sweepField = kneeField;
end
end
