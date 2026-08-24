classdef PhysicalConstants
    % PHYSICALCONSTANTS Shared physical constants for every lts package.
    % Lives in +util (the shared kernel) so component repositories can
    % reference it without depending on this repository's vehicle classes.
    % lts.vehicle.VehicleManager.g delegates here so the whole simulation
    % still reads a single named constant.

    properties (Constant)
        % Standard gravitational acceleration [m/s^2] (ISO 80000-3).
        % Display scripts still tolerate the legacy 9.81 literal.
        g = 9.80665
    end
end
