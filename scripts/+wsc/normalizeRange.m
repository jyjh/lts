function values = normalizeRange(values)
%NORMALIZERANGE Min-max normalize to [0, 1]; constant input maps to zeros.
values = values(:);
valueRange = max(values) - min(values);
if valueRange <= eps
    values = zeros(size(values));
else
    values = (values - min(values)) / valueRange;
end
end
