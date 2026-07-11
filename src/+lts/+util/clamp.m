function value = clamp(value, lo, hi)
% CLAMP Constrain a value to the closed interval [lo, hi].
% Uses the max(lo, min(hi, value)) ordering so NaN handling matches the
% element-wise min/max idiom it replaces.
value = max(lo, min(hi, value));
end
