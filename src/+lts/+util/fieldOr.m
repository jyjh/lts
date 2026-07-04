function value = fieldOr(s, fieldName, defaultValue)
% FIELDOR Struct field with a fallback when missing or empty.
if isstruct(s) && isfield(s, fieldName) && ~isempty(s.(fieldName))
    value = s.(fieldName);
else
    value = defaultValue;
end
end
