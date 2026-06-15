function value = unitSignedScalarOrDefault(candidate, defaultValue)
%UNITSIGNEDSCALARORDEFAULT Return candidate clamped to [-1,1] if valid; else defaultValue.
    if isnumeric(candidate) && isreal(candidate) ...
            && isscalar(candidate) && isfinite(candidate)
        value = max(-1, min(1, candidate));
    else
        value = defaultValue;
    end
end
