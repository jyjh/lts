classdef (Abstract) ForceElement
    % FORCEELEMENT Pluggable suspension force element
    %
    % A force element adds wheel-domain force between the sprung and
    % unsprung masses of a corner, alongside the built-in spring, damper,
    % bump stop, and anti-roll bar. Attach instances to
    % SimpleSuspension.forceElements to model heave springs / third
    % elements, progressive spring packs, or other travel-dependent
    % devices without editing the core force law.
    %
    % Contract (all quantities in the wheel domain, positive deflection =
    % compression, positive force pushes the sprung mass up):
    %   F = force(cornerState, suspensionDeflection, suspensionVelocity)
    %     - cornerState: lts.components.Suspension.SuspensionState handle
    %       (read-only; elements must not mutate it)
    %     - returns scalar force [N] at the current corner state
    %
    % Limitation: element stiffness does not yet feed
    % getEffectiveWheelRate / axle roll-stiffness derivation, so elements
    % that add tangent stiffness do not shift the elastic load-transfer
    % split. Keep that in mind when correlating roll balance with
    % stiffness-bearing elements attached.
    %
    % Concrete implementations:
    %   - TravelCurveElement — force vs wheel-travel lookup table

    methods (Abstract)
        F = force(obj, cornerState, suspensionDeflection, suspensionVelocity)
    end
end
