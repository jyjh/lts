function tf = validateNonnegativeVector(value, paramName)
%VALIDATENONNEGATIVEVECTOR Real, finite, nonnegative numeric vector.
validateattributes(value, {'numeric'}, ...
    {'vector', 'real', 'finite', 'nonnegative'}, mfilename, paramName);
tf = true;
end
