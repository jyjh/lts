function Fy = lateralForceGrid(tire, normalLoads, slipAnglesDeg)
%LATERALFORCEGRID Steady-state Pacejka Fy [N] over (load, slip angle).
%   Pure lateral sweep (kappa = gamma = phit = 0) at the tire file's
%   reference speed and pressure. Rows follow normalLoads, columns
%   slipAnglesDeg. Sign follows the mfeval convention (column 2 negated so
%   positive slip gives positive force). Zero loads stay zero.
normalLoads = normalLoads(:);
Fy = zeros(numel(normalLoads), numel(slipAnglesDeg));
active = normalLoads > 0;
if ~any(active)
    return;
end

activeLoads = normalLoads(active);
alphas = deg2rad(slipAnglesDeg(:).');
[loadGrid, alphaGrid] = ndgrid(activeLoads, alphas);
% mfeval input columns: [Fz, kappa, alpha, gamma, phit, Vx, P].
inputsMF = [ ...
    loadGrid(:), ...
    zeros(numel(loadGrid), 1), ...
    alphaGrid(:), ...
    zeros(numel(loadGrid), 1), ...
    zeros(numel(loadGrid), 1), ...
    repmat(tire.tireConstants.refVelocity, numel(loadGrid), 1), ...
    repmat(tire.tireConstants.nomPressure, numel(loadGrid), 1)];
outputs = wsc.batchedMfeval(tire.tireConstants.params, inputsMF);
Fy(active, :) = -reshape(outputs(:, 2), numel(activeLoads), numel(alphas));
end
