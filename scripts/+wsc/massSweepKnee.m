function knee = massSweepKnee(sweep, timeField, valueFields)
%MASSSWEEPKNEE Kneedle elbow of a mass sweep's time-vs-mass curve.
%   The time metric (1/sqrt compression of the underlying g) is where
%   diminishing returns live; raw g has accelerating returns to lightness.
%   timeField names the SWEEP field the elbow is found on. valueFields maps
%   knee field names to sweep field names as 'kneeField=sweepField' entries
%   (a bare name maps to itself); benefits are always included. Sweeps with
%   fewer than three samples fall back to index 1.
mass = sweep.massKg(:);
n = numel(mass);
knee = wsc.emptyKnee(valueFields);
if n == 0
    return;
end

if n < 3
    idx = 1;
    method = 'insufficient-samples';
else
    idx = wsc.kneedleIndex(mass, sweep.(timeField));
    method = 'knee';
end

knee.method = method;
knee.index = idx;
knee.massKg = mass(idx);
for i = 1:numel(valueFields)
    [kneeField, sweepField] = wsc.kneeFieldPair(valueFields{i});
    values = sweep.(sweepField);
    if numel(values) >= n
        knee.(kneeField) = values(idx);
    end
end
if numel(sweep.benefitSPerKgSaved) >= n
    knee.benefitSPerKgSaved = sweep.benefitSPerKgSaved(idx);
end
if numel(sweep.benefitGPerKgSaved) >= n
    knee.benefitGPerKgSaved = sweep.benefitGPerKgSaved(idx);
end
end
