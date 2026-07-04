function value = fieldOrDefault(s, fieldName, defaultValue)
% FIELDORDEFAULT Struct field with a fallback only when missing.
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end
