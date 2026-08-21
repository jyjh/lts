function tf = validateUnitScalar(value)
%VALIDATEUNITSCALAR Real, finite scalar in [0, 1].
validateattributes(value, {'numeric'}, ...
    {'scalar', 'real', 'finite', '>=', 0, '<=', 1}, mfilename);
tf = true;
end
