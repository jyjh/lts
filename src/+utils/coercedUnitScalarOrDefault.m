function value = coercedUnitScalarOrDefault(candidate, defaultValue)
%COERCEDUNITSCALARORDEFAULT Coerce scalar, then clamp it to [0,1].
    value = utils.coercedScalarOrDefault(candidate, defaultValue);
    value = max(0, min(1, value));
end
