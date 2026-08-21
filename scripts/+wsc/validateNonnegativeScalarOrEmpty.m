function tf = validateNonnegativeScalarOrEmpty(value, paramName)
%VALIDATENONNEGATIVESCALAROREMPTY Nonnegative scalar, or empty.
if isempty(value)
    tf = true;
    return;
end
validateattributes(value, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'nonnegative'}, mfilename, paramName);
tf = true;
end
