function value = logicalScalarOrDefault(candidate, defaultValue)
%LOGICALSCALARORDEFAULT Return a scalar logical value or coerce finite numeric scalar.
    if islogical(candidate) && isscalar(candidate)
        value = candidate;
    elseif isnumeric(candidate) && isreal(candidate) ...
            && isscalar(candidate) && isfinite(candidate)
        value = candidate ~= 0;
    else
        value = defaultValue;
    end
end
