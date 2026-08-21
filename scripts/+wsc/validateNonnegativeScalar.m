function tf = validateNonnegativeScalar(value)
%VALIDATENONNEGATIVESCALAR Real, finite, nonnegative numeric scalar.
validateattributes(value, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'nonnegative'}, mfilename);
tf = true;
end
