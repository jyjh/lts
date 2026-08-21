classdef TravelCurveElement < lts.components.Suspension.ForceElement
    % TRAVELCURVEELEMENT Force-vs-travel lookup element
    %
    % Piecewise-linear force lookup on wheel-domain suspension deflection
    % (positive = compression). General enough for a heave/third-element
    % spring, a progressive spring pack (nonlinear force curve), or a
    % rebound stop (one-sided curve clamped at zero beyond a travel).
    % Outside the table the force holds the endpoint value.
    %
    % Example — a 60 N/mm engagement after 20 mm of compression:
    %   el = lts.components.Suspension.TravelCurveElement( ...
    %       'travelGrid',  [-0.05 0.020 0.030 0.05], ...
    %       'forceGrid',   [   0      0   600  1800]);
    %   cornerUnit.forceElements = {el};

    properties
        % Monotonic wheel-travel sample points [m] (compression positive)
        travelGrid
        % Force [N] at each sample; positive pushes the sprung mass up
        forceGrid
    end

    properties (SetAccess = private)
        validated = false
    end

    methods
        function obj = TravelCurveElement(varargin)
            % TRAVELCURVEELEMENT Construct from name/value pairs
            %   TravelCurveElement('travelGrid', t, 'forceGrid', f)
            for k = 1:2:numel(varargin)
                obj.(varargin{k}) = varargin{k + 1};
            end
        end

        function F = force(obj, ~, suspensionDeflection, ~)
            if ~obj.validated
                obj.validate();
            end
            F = interp1(obj.travelGrid, obj.forceGrid, ...
                suspensionDeflection, 'linear', 'extrap');
            if ~isfinite(F)
                F = 0;
            end
        end
    end

    methods (Access = private)
        function validate(obj)
            t = obj.travelGrid(:);
            f = obj.forceGrid(:);
            ok = isnumeric(t) && isnumeric(f) && numel(t) == numel(f) && ...
                numel(t) >= 2 && all(isfinite(t)) && all(isfinite(f)) && ...
                all(diff(t) > 0);
            if ~ok
                error('lts_suspension_TravelCurveElement:InvalidCurve', ...
                    ['travelGrid and forceGrid must be equal-length ' ...
                     'finite vectors with strictly increasing travel.']);
            end
            obj.validated = true;
        end
    end
end
