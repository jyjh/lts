function value = maxFinite(values)
% MAXFINITE Maximum finite value, or NaN when no finite value exists.
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = max(values);
end
end
