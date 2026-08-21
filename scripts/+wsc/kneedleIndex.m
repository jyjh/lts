function idx = kneedleIndex(xValues, yValues)
%KNEEDLEINDEX Index of the Kneedle elbow of a curve (endpoints excluded).
%   Max perpendicular distance from the chord joining the endpoints. Falls
%   back to the midpoint index when the curve or chord is degenerate.
x = wsc.normalizeRange(xValues);
y = wsc.normalizeRange(yValues);
n = numel(x);
if n < 3
    idx = 1;
    return;
end

p1 = [x(1), y(1)];
p2 = [x(end), y(end)];
lineVec = p2 - p1;
lineNorm = hypot(lineVec(1), lineVec(2));
if lineNorm <= eps
    idx = ceil(n / 2);
    return;
end

distance = zeros(n, 1);
for i = 1:n
    p = [x(i), y(i)];
    distance(i) = abs(lineVec(1) * (p1(2) - p(2)) - ...
        (p1(1) - p(1)) * lineVec(2)) / lineNorm;
end
distance([1, end]) = -Inf;
[~, idx] = max(distance);
if ~isfinite(distance(idx))
    idx = ceil(n / 2);
end
end
