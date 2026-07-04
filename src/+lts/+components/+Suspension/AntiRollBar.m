classdef AntiRollBar
    % ANTIROLLBAR Anti-roll bar (sway bar / stabilizer bar) parameter model
    %
    % A pure-data value class describing one axle's anti-roll bar. It carries
    % no dynamics; it exposes the bar's effective wheel-rate roll stiffness
    % Kw_bar [N/m] — the ARB's contribution to that axle's roll stiffness,
    % already referenced to the wheel (including motion ratio and the
    % drop-link / control-arm lever arm).
    %
    % Used by SuspensionManager to derive the front/rear elastic load-
    % transfer split from actual spring + ARB stiffness, replacing the
    % legacy fixed roll-stiffness-distribution scalar.
    %
    % Wheel-rate conversion (standard ARB-at-the-wheel formula):
    %   Kw_bar = stiffness * motionRatio^2 / leverArm^2
    %
    % Set enabled = false (or stiffness = 0) to disable the bar; Kw_bar then
    % returns 0 and the axle's roll stiffness is just its wheel springs.

    properties
        % Bar stiffness at its end, referenced to the drop-link attachment
        % [N/m]. For a torsional bar this is the linear-equivalent rate at
        % the lever end (T_bar / (leverArm * angle) reduced to a force rate).
        stiffness = 0

        % Installation motion ratio between wheel travel and the ARB end
        % travel [dimensionless]. Squared in the wheel-rate conversion.
        motionRatio = 1.0

        % Lever arm from the bar axis / pivot to the drop-link attachment
        % point [m].
        leverArm = 0.25

        % Master enable. When false the bar contributes no stiffness.
        enabled = false
    end

    methods
        function obj = AntiRollBar(stiffness, motionRatio, leverArm, enabled)
            % ANTIROLLBAR Construct an axle anti-roll bar.
            %   AntiRollBar()                              % disabled
            %   AntiRollBar(stiffness, motionRatio, leverArm)
            %   AntiRollBar(stiffness, motionRatio, leverArm, enabled)
            if nargin >= 1 && ~isempty(stiffness)
                obj.stiffness = max(0, stiffness);
            end
            if nargin >= 2 && ~isempty(motionRatio)
                obj.motionRatio = max(eps, motionRatio);
            end
            if nargin >= 3 && ~isempty(leverArm)
                obj.leverArm = max(eps, leverArm);
            end
            if nargin >= 4 && ~isempty(enabled)
                obj.enabled = logical(enabled);
            end
        end

        function kw = getWheelRateStiffness(obj)
            % GETWHEELRATESTIFFNESS Effective wheel-rate roll stiffness [N/m].
            % The ARB's contribution to this axle's roll stiffness at the
            % wheel. Zero when disabled or when stiffness is zero.
            if ~obj.enabled || obj.stiffness <= 0
                kw = 0;
                return;
            end
            kw = obj.stiffness * obj.motionRatio^2 / obj.leverArm^2;
        end
    end
end
