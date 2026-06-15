function value = coercedScalarOrDefault(value, defaultValue)
%COERCEDSCALARORDEFAULT Return finite numeric/logical scalar as double; else defaultValue.
    if ~(isnumeric(value) || islogical(value)) || ~isreal(value) ...
            || ~isscalar(value) || ~isfinite(value)
        value = defaultValue;
    else
        value = double(value);
    end
end
