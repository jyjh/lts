% EMPIRICAL CAMBER CONVENTION PROBE (audit follow-up)
% Questions:
%  Q1 Is the MFeval model mirror-antisymmetric in (alpha, gamma)?
%     i.e. Fy(-a,-g) == -Fy(a,g), Fx even, Mz(-a,-g) == -Mz(a,g)
%  Q2 Sign of camber thrust in MFeval's own frame: slope of Fy wrt gamma at a=0.
%  Q3 End-to-end sim convention: FL.Fy / FR.Fy for positive LOCAL camber
%     (TireState: positive = top tilted outward). Physical expectation:
%     thrust toward the lean -> FL.Fy > 0 (outward = +y for left corner),
%     FR.Fy < 0 (outward = -y for right corner).

repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));

tire = lts.components.Tire.PacejkaTire('43105_18x7.5_10_R25B_7.tir');
params = tire.tireConstants.params;
P  = tire.tireConstants.nomPressure;
Fz = 1200;
Vx = 20;

a = 0.05; g = 0.06;

% ---- Q1: mirror antisymmetry of the raw model ----
in_pos = [Fz, 0,  a,  g, 0, Vx, P];
in_neg = [Fz, 0, -a, -g, 0, Vx, P];
o_pos = mfeval(params, in_pos, 111);
o_neg = mfeval(params, in_neg, 111);
fprintf('Q1 mirror antisymmetry (should hold for a symmetric tire fit):\n');
fprintf('   Fy(+a,+g)=%9.1f  Fy(-a,-g)=%9.1f  sum=%9.4f\n', o_pos(2), o_neg(2), o_pos(2)+o_neg(2));
fprintf('   Fx(+a,+g)=%9.1f  Fx(-a,-g)=%9.1f  sum=%9.4f\n', o_pos(1), o_neg(1), o_pos(1)+o_neg(1));
fprintf('   Mz(+a,+g)=%9.3f  Mz(-a,-g)=%9.3f  sum=%9.4f\n', o_pos(6), o_neg(6), o_pos(6)+o_neg(6));

% ---- Q2: pure camber response in MFeval frame ----
o_g0  = mfeval(params, [Fz, 0, 0, -g, 0, Vx, P], 111);
o_gpos = mfeval(params, [Fz, 0, 0, +g, 0, Vx, P], 111);
fprintf('Q2 pure camber, alpha=0: Fy(gamma=-g)=%9.2f  Fy(gamma=+g)=%9.2f  (MFeval frame)\n', o_g0(2), o_gpos(2));

% ---- Q3: end-to-end sim convention ----
corners = {tire.FL, tire.FR};
names = {'FL', 'FR'};
for c = 1:2
    cs = corners{c};
    cs.wheelRadius = 0.26;
    tire.updateCorner(cs, Fz, 0, 0, +g, 0, Vx, true, 'steady');
    fprintf('Q3 %s with LOCAL camber +%.2f rad (top OUTWARD): Fy=%9.2f  Mz=%8.3f\n', ...
        names{c}, g, cs.Fy, cs.Mz);
end
% cross-check: FL with NEGATIVE camber (top inward) -> thrust should flip
tire.updateCorner(tire.FL, Fz, 0, 0, -g, 0, Vx, true, 'steady');
fprintf('Q3 FL with LOCAL camber -%.2f rad (top INWARD):  Fy=%9.2f  Mz=%8.3f\n', g, tire.FL.Fy, tire.FL.Mz);

% reference: positive slip angle -> restoring force direction (sanity)
tire.updateCorner(tire.FL, Fz, +a, 0, 0, 0, Vx, true, 'steady');
fprintf('Q3 sanity FL alpha=+%0.2f (left-turn slip): Fy=%9.2f (expect >0, force to the left)\n', a, tire.FL.Fy);
