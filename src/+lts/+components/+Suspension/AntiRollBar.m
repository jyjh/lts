classdef AntiRollBar
    % ANTIROLLBAR Anti-roll bar (sway bar / stabilizer bar) parameter model
    %
    % A pure-data value class describing one axle's anti-roll bar. It carries
    % no dynamics; it exposes the bar's differential wheel-coupling rate
    % B_bar [N/m], already referenced to wheel travel (including motion ratio
    % and the drop-link / control-arm lever arm).
    %
    % Used by SuspensionManager to derive the front/rear elastic load-
    % transfer split from actual spring + ARB stiffness, replacing the
    % legacy fixed roll-stiffness-distribution scalar.
    %
    % Differential wheel-coupling conversion for a torsional bar:
    %   B_bar = stiffness * motionRatio^2 / leverArm^2
    %   F_right = B_bar * (z_right - z_left), F_left = -F_right
    % The equivalent independent-corner rate used for axle roll is 2*B_bar
    % because both equal-and-opposite forces contribute to the roll moment.
    %
    % Set enabled = false (or stiffness = 0) to disable the bar; B_bar then
    % returns 0 and the axle's roll stiffness is just its wheel springs.

    properties
        % Torsional bar stiffness [N*m/rad]. The lever-arm division in
        % getWheelRateStiffness converts torque per bar angle into a linear
        % force rate at the wheel. Radians are dimensionless in the unit
        % conversion.
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
            %   stiffness is torsional rate [N*m/rad].
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
            % GETWHEELRATESTIFFNESS Differential wheel-coupling rate [N/m].
            % This is the force at either wheel per left-right travel
            % difference. Zero when disabled or when stiffness is zero.
            if ~obj.enabled || obj.stiffness <= 0
                kw = 0;
                return;
            end
            kw = obj.stiffness * obj.motionRatio^2 / obj.leverArm^2;
        end
    end
end
