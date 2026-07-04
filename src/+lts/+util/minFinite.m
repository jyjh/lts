function value = minFinite(values)
% MINFINITE Minimum finite value, or NaN when no finite value exists.
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = min(values);
end
end
