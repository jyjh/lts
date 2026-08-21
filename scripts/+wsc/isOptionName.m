function tf = isOptionName(value, optionNames)
%ISOPTIONNAME True when value is a char/string scalar naming an option.
tf = false;
if ~(ischar(value) || (isstring(value) && isscalar(value)))
    return;
end
tf = any(strcmpi(char(value), optionNames));
end
