function value = unitScalarOrDefault(candidate, defaultValue)
%UNITSCALARORDEFAULT Return candidate clamped to [0,1] if a finite real scalar; else defaultValue.
    if isnumeric(candidate) && isreal(candidate) ...
            && isscalar(candidate) && isfinite(candidate)
        value = max(0, min(1, candidate));
    else
        value = defaultValue;
    end
end
