function value = nonnegativeScalarOrDefault(candidate, defaultValue)
%NONNEGATIVESCALARORDEFAULT Return candidate if it is a finite, real, >=0 scalar; else defaultValue.
    if isnumeric(candidate) && isreal(candidate) ...
            && isscalar(candidate) && isfinite(candidate) && candidate >= 0
        value = candidate;
    else
        value = defaultValue;
    end
end
