function value = coercedNonnegativeScalarOrDefault(candidate, defaultValue)
%COERCEDNONNEGATIVESCALARORDEFAULT Coerce scalar, then clamp negative values to zero.
    value = utils.coercedScalarOrDefault(candidate, defaultValue);
    value = max(value, 0);
end
