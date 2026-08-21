function tf = validatePositiveVector(value, paramName)
%VALIDATEPOSITIVEVECTOR Real, finite, positive numeric vector.
validateattributes(value, {'numeric'}, ...
    {'vector', 'real', 'finite', 'positive'}, mfilename, paramName);
tf = true;
end
