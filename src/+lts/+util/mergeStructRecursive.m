function out = mergeStructRecursive(base, overrides)
% MERGESTRUCTRECURSIVE Recursively overlay one scalar struct onto another.
out = base;
if isempty(overrides)
    return;
end

fields = fieldnames(overrides);
for i = 1:numel(fields)
    field = fields{i};
    if isfield(out, field) && isstruct(out.(field)) && ...
            isstruct(overrides.(field)) && isscalar(out.(field)) && ...
            isscalar(overrides.(field))
        out.(field) = lts.util.mergeStructRecursive(out.(field), overrides.(field));
    else
        out.(field) = overrides.(field);
    end
end
end
