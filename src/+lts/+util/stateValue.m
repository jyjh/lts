function value = stateValue(state, fieldName, defaultValue)
% STATEVALUE Read an object property or struct field with a fallback.
value = defaultValue;
if isobject(state)
    if isprop(state, fieldName)
        candidate = state.(fieldName);
        if ~isempty(candidate)
            value = candidate;
        end
    end
elseif isstruct(state) && isfield(state, fieldName)
    candidate = state.(fieldName);
    if ~isempty(candidate)
        value = candidate;
    end
end
end
