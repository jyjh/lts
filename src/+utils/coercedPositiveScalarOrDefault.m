function value = coercedPositiveScalarOrDefault(candidate, defaultValue)
%COERCEDPOSITIVESCALARORDEFAULT Coerce scalar, then require it to be positive.
    value = utils.coercedScalarOrDefault(candidate, defaultValue);
    if value <= 0
        value = defaultValue;
    end
end
