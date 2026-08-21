function diag_relax_loop_params()
% DIAG_RELAX_LOOP_PARAMS Measure the wheel-slip/relaxation loop parameters.
%   Reports tire longitudinal slip stiffness Cx = dFx/dkappa from the actual
%   MFeval model at representative loads, driveline inertias, and the
%   predicted wheel + relaxation mode (frequency, damping ratio) for both
%   the physical shared relaxation length and the de-tuned 0.05 m value.

repoRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(repoRoot, 'src'));

track = lts.components.TestTrack('straight75');
config = lts.vehicles.R25();
vehicle = lts.vehicle.VehicleManager.fromConfig(config, track, 0.001);

R = config.tire.wheelRadius;
Iwheel = config.tire.wheelInertia;
ratio = vehicle.powertrain.getTotalGearRatio();
Irotor = vehicle.powertrain.getReflectedRotorInertia();
IrearAxleMode = Iwheel + Irotor / 2;  % per-wheel common-mode share of carrier inertia

fprintf('R (wheel radius)        : %.4f m\n', R);
fprintf('I wheel (per corner)    : %.4f kg m^2\n', Iwheel);
fprintf('gear ratio              : %.2f\n', ratio);
fprintf('I rotor reflected       : %.4f kg m^2 (carrier)\n', Irotor);
fprintf('I rear common-mode/wheel: %.4f kg m^2\n', IrearAxleMode);

fprintf('\n  Fz[N]   Cx[N/-]   Cx/Fz   | rear mode (I=%.3f): f, zeta @sigma=0.255 / 0.05\n', IrearAxleMode);
for Fz = [800 1200 1600 2000]
    dk = 1e-4;
    Fp = vehicle.tire.computeLongitudinalForce(Fz, +dk, 1);
    Fm = vehicle.tire.computeLongitudinalForce(Fz, -dk, 1);
    Cx = (Fp - Fm) / (2 * dk);
    for Isel = [Iwheel, IrearAxleMode]
    end
    [f255, z255] = modeParams(Cx, R, IrearAxleMode, 0.255, 14);
    [f005, z005] = modeParams(Cx, R, IrearAxleMode, 0.05, 14);
    fprintf('  %4d  %8.0f  %6.1f   |  %5.1f Hz zeta=%.3f   |  %5.1f Hz zeta=%.3f\n', ...
        Fz, Cx, Cx / Fz, f255, z255, f005, z005);
end

% Front (undriven) wheels at representative load
Fz = 1200;
dk = 1e-4;
Fp = vehicle.tire.computeLongitudinalForce(Fz, +dk, 1);
Fm = vehicle.tire.computeLongitudinalForce(Fz, -dk, 1);
Cx = (Fp - Fm) / (2 * dk);
[fF, zF] = modeParams(Cx, R, Iwheel, 0.255, 14);
fprintf('\nFront wheel (I=%.3f, Fz=%d): %.1f Hz, zeta=%.3f @ sigma=0.255\n', ...
    Iwheel, Fz, fF, zF);
end

function [f, zeta] = modeParams(Cx, R, I, sigma, V)
% Coupled wheel-inertia + slip-relaxation 2nd-order mode (continuous):
%   I*sigma*s^2 + I*V*s + Cx*R^2 = 0
wn = sqrt(Cx * R^2 / (I * sigma));
zeta = (V / 2) * sqrt(I / (sigma * Cx * R^2));
f = wn / (2 * pi);
end
