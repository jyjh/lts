function loads = bicycleCornerLoads(massKg, ayG, loadTransfer)
%BICYCLECORNERLOADS Steady-state lateral corner loads (bicycle model).
%   Corner order is [frontInside; frontOutside; rearInside; rearOutside].
%   Inside loads are floored at zero so inside-tire-lift does not generate
%   negative capacity through extrapolation.
g = 9.80665;
W = massKg * g;
frontAxleStatic = W * loadTransfer.staticFrontWeight;
rearAxleStatic = W * (1 - loadTransfer.staticFrontWeight);
totalTransfer = W * ayG * loadTransfer.cgHeight / loadTransfer.trackWidth;
frontTransfer = totalTransfer * loadTransfer.frontLoadTransferDistribution;
rearTransfer = totalTransfer * (1 - loadTransfer.frontLoadTransferDistribution);

frontInside = max((frontAxleStatic - frontTransfer) / 2, 0);
frontOutside = frontAxleStatic - frontInside;
rearInside = max((rearAxleStatic - rearTransfer) / 2, 0);
rearOutside = rearAxleStatic - rearInside;
loads = [frontInside; frontOutside; rearInside; rearOutside];
end
