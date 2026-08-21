function tf = validatePositiveScalar(value)
%VALIDATEPOSITIVESCALAR Real, finite, positive numeric scalar.
validateattributes(value, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'positive'}, mfilename);
tf = true;
end
