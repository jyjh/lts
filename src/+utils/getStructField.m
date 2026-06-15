function value = getStructField(s, fieldName, defaultValue)
%GETSTRUCTFIELD Read an optional field from a scalar struct.
    if isstruct(s) && isfield(s, fieldName)
        value = s.(fieldName);
    else
        value = defaultValue;
    end
end
