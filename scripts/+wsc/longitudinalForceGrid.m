function Fx = longitudinalForceGrid(tire, normalLoads, slipRatios)
%LONGITUDINALFORCEGRID Steady-state Pacejka Fx [N] over (load, slip ratio).
%   Pure longitudinal sweep (alpha = gamma = phit = 0) at the tire file's
%   reference speed and pressure. Rows follow normalLoads, columns
%   slipRatios. Zero loads stay zero.
normalLoads = normalLoads(:);
Fx = zeros(numel(normalLoads), numel(slipRatios));
active = normalLoads > 0;
if ~any(active)
    return;
end

activeLoads = normalLoads(active);
kappas = slipRatios(:).';
[loadGrid, kappaGrid] = ndgrid(activeLoads, kappas);
% mfeval input columns: [Fz, kappa, alpha, gamma, phit, Vx, P].
inputsMF = [ ...
    loadGrid(:), ...
    kappaGrid(:), ...
    zeros(numel(loadGrid), 1), ...
    zeros(numel(loadGrid), 1), ...
    zeros(numel(loadGrid), 1), ...
    repmat(tire.tireConstants.refVelocity, numel(loadGrid), 1), ...
    repmat(tire.tireConstants.nomPressure, numel(loadGrid), 1)];
outputs = wsc.batchedMfeval(tire.tireConstants.params, inputsMF);
Fx(active, :) = reshape(outputs(:, 1), numel(activeLoads), numel(kappas));
end
