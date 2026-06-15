function value = scalarOrDefault(candidate, defaultValue)
%SCALARORDEFAULT Return candidate if it is a finite real scalar, else defaultValue.
    if isnumeric(candidate) && isreal(candidate) ...
            && isscalar(candidate) && isfinite(candidate)
        value = candidate;
    else
        value = defaultValue;
    end
end
