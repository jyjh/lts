function cgHeight = coupledCgHeight(massKg, referenceCgHeight, ...
        referenceMassKg, cgDropPerKgSavedM)
%COUPLEDCGHEIGHT Per-kg CG schedule for the coupled-CG sweep.
%   For every kg of mass saved below ReferenceMassKg the CG drops by
%   CgDropPerKgSavedM (default 1 mm). Anchored at the reference mass so the
%   coupled sweep reproduces the baseline there. CG is not permitted to go
%   negative.
kgSaved = max(referenceMassKg - massKg(:), 0);
cgHeight = max(referenceCgHeight - kgSaved .* cgDropPerKgSavedM, 0);
end
