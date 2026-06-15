function valid = isPositiveScalar(candidate)
%ISPOSITIVESCALAR True when candidate is a finite, real, positive scalar.
    valid = isnumeric(candidate) && isreal(candidate) ...
        && isscalar(candidate) && isfinite(candidate) && candidate > 0;
end
