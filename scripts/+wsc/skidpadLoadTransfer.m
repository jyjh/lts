function loadTransfer = skidpadLoadTransfer(opts)
%SKIDPADLOADTRANSFER Lateral load-transfer struct from parsed script options.
%   Requires TrackWidth, CgHeight, StaticFrontWeight, and
%   LateralLoadTransferDistribution (empty = static front weight).
frontLoadTransferDistribution = opts.LateralLoadTransferDistribution;
if isempty(frontLoadTransferDistribution)
    frontLoadTransferDistribution = opts.StaticFrontWeight;
end

loadTransfer = struct( ...
    'trackWidth', opts.TrackWidth, ...
    'cgHeight', opts.CgHeight, ...
    'staticFrontWeight', opts.StaticFrontWeight, ...
    'frontLoadTransferDistribution', frontLoadTransferDistribution);
end
