function value = positiveScalarOrDefault(candidate, defaultValue)
%POSITIVESCALARORDEFAULT Return candidate if it is a finite, real, positive scalar; else defaultValue.
    if utils.isPositiveScalar(candidate)
        value = candidate;
    else
        value = defaultValue;
    end
end
