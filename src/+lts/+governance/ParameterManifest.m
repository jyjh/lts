classdef ParameterManifest
    %PARAMETERMANIFEST Validate and query governed vehicle parameters.

    properties (Constant)
        Schema = "lts.governance.parameter-manifest.v1"
        Roles = ["design", "fixed_measured", "global_calibrated", ...
            "run_nuisance", "legacy_effective"]
    end

    methods (Static)
        function manifest = load(filePath)
            if ~exist(filePath, 'file')
                error('lts_governance_ParameterManifest:MissingFile', ...
                    'Parameter manifest does not exist: %s', filePath);
            end
            try
                manifest = jsondecode(fileread(filePath));
            catch err
                error('lts_governance_ParameterManifest:InvalidJson', ...
                    'Could not parse parameter manifest "%s": %s', ...
                    filePath, err.message);
            end
            manifest = lts.governance.ParameterManifest.validate(manifest);
            manifest.sourceFile = char(filePath);
        end

        function manifest = validate(manifest)
            if ~isstruct(manifest) || ~isfield(manifest, 'schema') || ...
                    string(manifest.schema) ~= lts.governance.ParameterManifest.Schema
                error('lts_governance_ParameterManifest:InvalidSchema', ...
                    'Expected schema %s.', lts.governance.ParameterManifest.Schema);
            end
            requiredTop = {'id', 'vehicle', 'parameters'};
            for i = 1:numel(requiredTop)
                if ~isfield(manifest, requiredTop{i}) || isempty(manifest.(requiredTop{i}))
                    error('lts_governance_ParameterManifest:MissingField', ...
                        'Manifest is missing "%s".', requiredTop{i});
                end
            end
            params = lts.governance.ParameterManifest.asStructArray(manifest.parameters);
            names = strings(numel(params), 1);
            paths = strings(numel(params), 1);
            for i = 1:numel(params)
                p = params(i);
                required = {'name', 'path', 'role', 'units', 'source', ...
                    'provenance', 'uncertainty', 'calibrationDomain'};
                for j = 1:numel(required)
                    if ~isfield(p, required{j}) || ...
                            (ischar(p.(required{j})) && isempty(p.(required{j})))
                        error('lts_governance_ParameterManifest:MissingParameterField', ...
                            'Parameter %d is missing "%s".', i, required{j});
                    end
                end
                names(i) = string(p.name);
                paths(i) = string(p.path);
                role = string(p.role);
                if ~any(role == lts.governance.ParameterManifest.Roles)
                    error('lts_governance_ParameterManifest:InvalidRole', ...
                        'Parameter "%s" has invalid role "%s".', names(i), role);
                end
                if strlength(string(p.units)) == 0 || ...
                        strlength(string(p.source)) == 0 || ...
                        strlength(string(p.provenance)) == 0
                    error('lts_governance_ParameterManifest:MissingProvenance', ...
                        'Parameter "%s" requires units, source, and provenance.', names(i));
                end
                lts.governance.ParameterManifest.validateUncertainty(p, names(i));
                lts.governance.ParameterManifest.validateDomain(p, names(i));
            end
            if numel(unique(names)) ~= numel(names) || ...
                    numel(unique(paths)) ~= numel(paths)
                error('lts_governance_ParameterManifest:DuplicateParameter', ...
                    'Parameter names and paths must be unique.');
            end
            manifest.parameters = params;
        end

        function parameter = find(manifest, nameOrPath)
            params = lts.governance.ParameterManifest.asStructArray(manifest.parameters);
            key = string(nameOrPath);
            idx = find(string({params.name}) == key | string({params.path}) == key);
            if numel(idx) ~= 1
                error('lts_governance_ParameterManifest:UnknownParameter', ...
                    'Manifest does not contain exactly one parameter "%s".', key);
            end
            parameter = params(idx);
        end

        function assertCalibratable(manifest, names)
            names = string(names);
            for i = 1:numel(names)
                p = lts.governance.ParameterManifest.find(manifest, names(i));
                if string(p.role) ~= "global_calibrated"
                    error('lts_governance_ParameterManifest:FitForbidden', ...
                        'Parameter "%s" has role "%s" and cannot be fitted.', ...
                        names(i), p.role);
                end
            end
        end

        function assertDesignParameter(manifest, nameOrPath)
            p = lts.governance.ParameterManifest.find(manifest, nameOrPath);
            if string(p.role) ~= "design"
                error('lts_governance_ParameterManifest:DesignChangeForbidden', ...
                    'Parameter "%s" has role "%s", not "design".', p.name, p.role);
            end
        end

        function warnings = domainWarnings(manifest, config)
            warnings = strings(0, 1);
            params = lts.governance.ParameterManifest.asStructArray(manifest.parameters);
            for i = 1:numel(params)
                p = params(i);
                domain = p.calibrationDomain;
                if ~isstruct(domain) || ~isfield(domain, 'lower') || ...
                        ~isfield(domain, 'upper')
                    continue;
                end
                try
                    value = lts.governance.ParameterManifest.getValue(config, p.path);
                catch
                    % The path does not resolve on this config. Surface it as
                    % a warning so a typo'd/malformed manifest path is visible
                    % rather than silently producing no domain warning.
                    warning('lts_governance_ParameterManifest:UnresolvedPath', ...
                        'Parameter "%s" path "%s" does not resolve on the config; skipping domain check.', ...
                        p.name, p.path);
                    continue;
                end
                if isnumeric(value) && isscalar(value) && isfinite(value) && ...
                        (value < domain.lower || value > domain.upper)
                    warnings(end + 1, 1) = sprintf( ... %#ok<AGROW>
                        '%s=%g is outside calibrated domain [%g, %g] %s.', ...
                        p.path, value, domain.lower, domain.upper, p.units);
                end
            end
        end

        function value = getValue(root, path)
            parts = strsplit(char(path), '.');
            value = root;
            for i = 1:numel(parts)
                key = parts{i};
                if isobject(value) && isprop(value, key)
                    value = value.(key);
                elseif isstruct(value) && isfield(value, key)
                    value = value.(key);
                else
                    error('lts_governance_ParameterManifest:InvalidPath', ...
                        'Configuration path "%s" does not exist.', path);
                end
            end
        end

        function root = setValue(root, path, value)
            parts = strsplit(char(path), '.');
            root = lts.governance.ParameterManifest.setRecursive(root, parts, value);
        end
    end

    methods (Static, Access = private)
        function params = asStructArray(raw)
            if iscell(raw)
                params = vertcat(raw{:});
            else
                params = raw(:);
            end
        end

        function validateUncertainty(p, name)
            u = p.uncertainty;
            if ~isstruct(u) || ~isfield(u, 'distribution') || ...
                    ~isfield(u, 'standardDeviation')
                error('lts_governance_ParameterManifest:InvalidUncertainty', ...
                    'Parameter "%s" requires an uncertainty distribution and standardDeviation.', name);
            end
            if ~any(string(u.distribution) == ["fixed", "normal", "uniform"]) || ...
                    ~isscalar(u.standardDeviation) || ...
                    ~isfinite(u.standardDeviation) || u.standardDeviation < 0
                error('lts_governance_ParameterManifest:InvalidUncertainty', ...
                    'Parameter "%s" has invalid uncertainty metadata.', name);
            end
        end

        function validateDomain(p, name)
            d = p.calibrationDomain;
            if ~isstruct(d) || ~isfield(d, 'lower') || ~isfield(d, 'upper') || ...
                    ~isscalar(d.lower) || ~isscalar(d.upper) || ...
                    isnan(d.lower) || isnan(d.upper) || d.lower > d.upper
                error('lts_governance_ParameterManifest:InvalidDomain', ...
                    'Parameter "%s" has an invalid calibration domain.', name);
            end
        end

        function root = setRecursive(root, parts, value)
            key = parts{1};
            if numel(parts) == 1
                if isobject(root) && isprop(root, key)
                    root.(key) = value;
                elseif isstruct(root) && isfield(root, key)
                    root.(key) = value;
                else
                    error('lts_governance_ParameterManifest:InvalidPath', ...
                        'Configuration field "%s" does not exist.', key);
                end
                return;
            end
            if isobject(root) && isprop(root, key)
                child = root.(key);
            elseif isstruct(root) && isfield(root, key)
                child = root.(key);
            else
                error('lts_governance_ParameterManifest:InvalidPath', ...
                    'Configuration field "%s" does not exist.', key);
            end
            child = lts.governance.ParameterManifest.setRecursive(child, parts(2:end), value);
            root.(key) = child;
        end
    end
end
