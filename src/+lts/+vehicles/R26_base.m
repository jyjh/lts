function cfg = R26_base()
    % R26_BASE R26 base configuration as an overlay on lts.vehicles.baseline.
    %
    % Only the values that differ from the baseline car are set here; the
    % baseline file remains the documentation of every parameter and the
    % copy/paste template for new car configs.
    %
    % Inherited from baseline and still unreviewed for R26 (TODO): brake bias
    % and force coefficient, damper rates, bump-stop and tire spring rates.
    %
    % Units: SI throughout (m, kg, N, s, rad, Pa).

    cfg = lts.vehicles.baseline();

    cfg.name = "R26_base";

    %% Vehicle-level deltas
    cfg.totalMass         = 264;      % Total mass with driver [kg]
    cfg.staticFrontWeight = 0.5038;   % Static front weight distribution [0-1]
    cfg.unsprungMass      = 9.3;      % Per-corner unsprung mass [kg]

    %% Suspension deltas
    cfg.suspension.front.springRate = 43780;  % Heave spring rate [N/m]
    cfg.suspension.rear.springRate  = 39400;  % Heave spring rate [N/m] (slightly softer)
    cfg.suspension.motionRatio      = 1;      % Installation motion ratio (wheel<->spring)

    %% Chassis deltas
    % ~2836 N*m/deg; couples front vs rear roll (twist).
    cfg.chassis.torsionalRigidity = 162518;   % [N*m/rad]
end
