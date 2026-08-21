function out = movingAverage(values, halfWindow)
%MOVINGAVERAGE Centered moving average over a finite-only window.
%   NaNs/Infs are excluded from each local mean but keep their position, so
%   edges and gaps degrade gracefully rather than propagating NaN.
values = values(:);
n = numel(values);
out = zeros(n, 1);
for i = 1:n
    lo = max(1, i - halfWindow);
    hi = min(n, i + halfWindow);
    window = values(lo:hi);
    window = window(isfinite(window));
    if isempty(window)
        out(i) = values(i);
    else
        out(i) = mean(window);
    end
end
end
