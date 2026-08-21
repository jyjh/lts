classdef CorrelationParameterRegistry
    %CORRELATIONPARAMETERREGISTRY Bounded physical parameters for tuning.

    methods (Static)
        function registry = load(filePath)
            if ~exist(filePath, 'file')
                error('lts_correlation_CorrelationParameterRegistry:MissingFile', ...
                    'Parameter-space file does not exist: %s', filePath);
            end
            try
                registry = jsondecode(fileread(filePath));
            catch err
                error('lts_correlation_CorrelationParameterRegistry:InvalidJson', ...
                    'Could not parse parameter-space file "%s": %s', ...
                    filePath, err.message);
            end
            if ~isfield(registry, 'schema') || ...
                    string(registry.schema) ~= "lts.correlation.parameter-space.v1"
                error('lts_correlation_CorrelationParameterRegistry:InvalidSchema', ...
                    'Expected schema lts.correlation.parameter-space.v1.');
            end
            if ~isfield(registry, 'parameters') || isempty(registry.parameters)
                error('lts_correlation_CorrelationParameterRegistry:NoParameters', ...
                    'Parameter-space file contains no parameters.');
            end
            if iscell(registry.parameters)
                rawParams = registry.parameters(:);
            else
                rawParams = num2cell(registry.parameters(:));
            end
            for i = 1:numel(rawParams)
                if ~isfield(rawParams{i}, 'indices')
                    rawParams{i}.indices = [];
                end
            end
            params = vertcat(rawParams{:});
            registry.parameters = params;
            names = strings(numel(params), 1);
            for i = 1:numel(params)
                p = params(i);
                required = {'name', 'path', 'baseline', 'lower', 'upper', 'transform', 'units'};
                for j = 1:numel(required)
                    if ~isfield(p, required{j})
                        error('lts_correlation_CorrelationParameterRegistry:MissingField', ...
                            'Parameter %d is missing field "%s".', i, required{j});
                    end
                end
                names(i) = string(p.name);
                if ~isvarname(char(names(i)))
                    error('lts_correlation_CorrelationParameterRegistry:InvalidName', ...
                        'Parameter name "%s" is not a valid table variable name.', names(i));
                end
                if ~isscalar(p.lower) || ~isscalar(p.upper) || ...
                        ~isscalar(p.baseline) || ~isfinite(p.lower) || ...
                        ~isfinite(p.upper) || ~isfinite(p.baseline) || ...
                        p.lower >= p.upper || p.baseline < p.lower || p.baseline > p.upper
                    error('lts_correlation_CorrelationParameterRegistry:InvalidBounds', ...
                        'Parameter "%s" has invalid bounds or baseline.', names(i));
                end
                transform = lower(string(p.transform));
                if transform ~= "linear" && transform ~= "log"
                    error('lts_correlation_CorrelationParameterRegistry:InvalidTransform', ...
                        'Parameter "%s" transform must be linear or log.', names(i));
                end
                if transform == "log" && p.lower <= 0
                    error('lts_correlation_CorrelationParameterRegistry:InvalidLogBounds', ...
                        'Log parameter "%s" must have positive bounds.', names(i));
                end
            end
            if numel(unique(names)) ~= numel(names)
                error('lts_correlation_CorrelationParameterRegistry:DuplicateNames', ...
                    'Parameter names must be unique.');
            end
        end

        function names = names(registry)
            names = string({registry.parameters.name});
            names = names(:).';
        end

        function values = baseline(registry)
            values = double([registry.parameters.baseline]);
        end

        function physical = decode(registry, normalized)
            params = registry.parameters(:);
            normalized = double(normalized);
            if size(normalized, 2) ~= numel(params)
                error('lts_correlation_CorrelationParameterRegistry:WrongWidth', ...
                    'Candidate has %d columns; registry requires %d.', ...
                    size(normalized, 2), numel(params));
            end
            if any(~isfinite(normalized), 'all') || ...
                    any(normalized < 0, 'all') || any(normalized > 1, 'all')
                error('lts_correlation_CorrelationParameterRegistry:OutOfUnitBounds', ...
                    'Normalized candidate values must be finite and within [0, 1].');
            end
            physical = zeros(size(normalized));
            for i = 1:numel(params)
                lo = double(params(i).lower);
                hi = double(params(i).upper);
                if lower(string(params(i).transform)) == "log"
                    physical(:, i) = exp(log(lo) + normalized(:, i) .* ...
                        (log(hi) - log(lo)));
                else
                    physical(:, i) = lo + normalized(:, i) .* (hi - lo);
                end
            end
        end

        function normalized = encode(registry, physical)
            params = registry.parameters(:);
            physical = double(physical);
            if size(physical, 2) ~= numel(params)
                error('lts_correlation_CorrelationParameterRegistry:WrongWidth', ...
                    'Physical candidate width does not match registry.');
            end
            normalized = zeros(size(physical));
            for i = 1:numel(params)
                lo = double(params(i).lower);
                hi = double(params(i).upper);
                values = physical(:, i);
                if any(~isfinite(values)) || any(values < lo) || any(values > hi)
                    error('lts_correlation_CorrelationParameterRegistry:OutOfBounds', ...
                        'Parameter "%s" is outside [%g, %g].', params(i).name, lo, hi);
                end
                if lower(string(params(i).transform)) == "log"
                    normalized(:, i) = (log(values) - log(lo)) ./ (log(hi) - log(lo));
                else
                    normalized(:, i) = (values - lo) ./ (hi - lo);
                end
            end
        end

        function config = apply(registry, config, values)
            values = double(values(:).');
            params = registry.parameters(:);
            if numel(values) ~= numel(params)
                error('lts_correlation_CorrelationParameterRegistry:WrongWidth', ...
                    'Candidate has %d values; registry requires %d.', ...
                    numel(values), numel(params));
            end
            lts.correlation.CorrelationParameterRegistry.encode(registry, values);
            for i = 1:numel(params)
                indices = [];
                if isfield(params(i), 'indices')
                    indices = double(params(i).indices(:).');
                end
                config = lts.correlation.CorrelationParameterRegistry.setPath( ...
                    config, char(params(i).path), values(i), indices);
            end
        end

        function T = candidateTable(registry, candidateIds, physical)
            names = cellstr(lts.correlation.CorrelationParameterRegistry.names(registry));
            T = array2table(physical, 'VariableNames', names);
            T = addvars(T, candidateIds(:), 'Before', 1, ...
                'NewVariableNames', 'candidate_id');
        end
    end

    methods (Static, Access = private)
        function root = setPath(root, path, value, indices)
            parts = strsplit(path, '.');
            root = lts.correlation.CorrelationParameterRegistry.setNested( ...
                root, parts, value, indices);
        end

        function root = setNested(root, parts, value, indices)
            field = parts{1};
            if isobject(root)
                if ~isprop(root, field)
                    error('lts_correlation_CorrelationParameterRegistry:UnknownPath', ...
                        'Unknown configuration property "%s".', field);
                end
            elseif ~isstruct(root) || ...
                    (~isfield(root, field) && numel(parts) > 1)
                error('lts_correlation_CorrelationParameterRegistry:UnknownPath', ...
                    'Unknown configuration field "%s".', field);
            end
            if numel(parts) == 1
                if isempty(indices)
                    root.(field) = value;
                else
                    current = root.(field);
                    if any(indices < 1) || any(indices > numel(current))
                        error('lts_correlation_CorrelationParameterRegistry:InvalidIndices', ...
                            'Indices for "%s" exceed the configured value.', field);
                    end
                    current(indices) = value;
                    root.(field) = current;
                end
            else
                child = root.(field);
                child = lts.correlation.CorrelationParameterRegistry.setNested( ...
                    child, parts(2:end), value, indices);
                root.(field) = child;
            end
        end
    end
end
