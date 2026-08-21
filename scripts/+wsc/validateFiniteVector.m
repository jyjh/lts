function tf = validateFiniteVector(value, paramName)
%VALIDATEFINITEVECTOR Real, finite numeric vector.
validateattributes(value, {'numeric'}, ...
    {'vector', 'real', 'finite'}, mfilename, paramName);
tf = true;
end
