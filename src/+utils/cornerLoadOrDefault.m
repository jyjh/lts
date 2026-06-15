function value = cornerLoadOrDefault(cornerLoads, fieldName)
%CORNERLOADORDEFAULT Return a nonnegative finite corner load from a scalar struct.
    value = 0;
    if isstruct(cornerLoads) && isscalar(cornerLoads) ...
            && isfield(cornerLoads, fieldName)
        value = utils.nonnegativeScalarOrDefault(cornerLoads.(fieldName), 0);
    end
end
