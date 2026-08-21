function printCoupledCgReport(massKg, knee, sweepCoupled, kneeCoupled, ...
        referenceMassKg, cgDropPerKgSavedM, timeSpec, gSpec, ...
        timeLabel, timeFmt)
%PRINTCOUPLEDCGREPORT Coupled-CG block of the weight-savings reports.
%   Prints the coupled diminishing-returns knee, the knee shift vs the
%   fixed-CG sweep, the time-curve quadratic coefficient, and the endpoint
%   samples. timeSpec/gSpec map knee to sweep field names as
%   'kneeField=sweepField' (bare names map to themselves, e.g.
%   'lapTimeS' or 'accelG=sustainedG'); timeLabel/timeFmt shape the wording
%   (e.g. 'lap-time' and '%.3f s lap'). No output when sweepCoupled is empty.
if isempty(sweepCoupled)
    return;
end
[timeField, sweepTimeField] = wsc.kneeFieldPair(timeSpec);
[gField, sweepGField] = wsc.kneeFieldPair(gSpec);
fprintf('\n--- Coupled CG (%.1f mm drop per kg saved, anchored at %.1f kg) ---\n', ...
    1000 * cgDropPerKgSavedM, referenceMassKg);
if isfinite(kneeCoupled.massKg)
    fprintf('>> Coupled diminishing-return mass: %.1f kg', kneeCoupled.massKg);
    if isfinite(kneeCoupled.(timeField))
        fprintf(' (%s, %.3f g', ...
            sprintf(timeFmt, kneeCoupled.(timeField)), kneeCoupled.(gField));
    end
    if isfinite(kneeCoupled.benefitSPerKgSaved)
        fprintf(', %.4f s/kg saved', kneeCoupled.benefitSPerKgSaved);
    end
    fprintf(', %s)\n', kneeCoupled.method);
else
    fprintf('>> Coupled diminishing-return mass: not available (too few samples)\n');
end
if isfinite(knee.massKg) && isfinite(kneeCoupled.massKg)
    deltaKg = kneeCoupled.massKg - knee.massKg;
    if abs(deltaKg) < 0.05
        fprintf('   Coupled-CG knee is unchanged from the fixed-CG knee (%.1f kg).\n', ...
            knee.massKg);
    elseif deltaKg > 0
        fprintf('   Coupled-CG knee shifts HEAVIER by %.1f kg (%.1f -> %.1f kg).\n', ...
            deltaKg, knee.massKg, kneeCoupled.massKg);
    else
        fprintf('   Coupled-CG knee shifts LIGHTER by %.1f kg (%.1f -> %.1f kg).\n', ...
            abs(deltaKg), knee.massKg, kneeCoupled.massKg);
    end
end
tCurvCoupled = wsc.curveLinearity(massKg, sweepCoupled.(sweepTimeField));
fprintf('   Coupled %s: quad coeff %+.3e  (>=0 = diminishing returns as car gets lighter)\n', ...
    timeLabel, tCurvCoupled.quadCoeff);
endpts = [1, numel(massKg)];
for k = 1:numel(endpts)
    i = endpts(k);
    fprintf('   Coupled at %.0f kg (CG %.0f mm): %.3f g, %.3f s', ...
        massKg(i), 1000 * sweepCoupled.cgHeightM(i), ...
        sweepCoupled.(sweepGField)(i), sweepCoupled.(sweepTimeField)(i));
    if isfinite(sweepCoupled.benefitSPerKgSaved(i))
        fprintf(', %.4f s/kg', sweepCoupled.benefitSPerKgSaved(i));
    end
    fprintf(' (%s)\n', sweepCoupled.limitedBy(i));
end
end
