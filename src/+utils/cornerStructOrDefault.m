function value = cornerStructOrDefault(candidate)
%CORNERSTRUCTORDEFAULT Return FL/FR/RL/RR scalar fields, defaulting missing values to 0.
    value = struct('FL', 0, 'FR', 0, 'RL', 0, 'RR', 0);
    if ~isstruct(candidate) || ~isscalar(candidate)
        return;
    end
    fields = {'FL', 'FR', 'RL', 'RR'};
    for i = 1:numel(fields)
        fieldName = fields{i};
        if isfield(candidate, fieldName)
            value.(fieldName) = utils.scalarOrDefault(candidate.(fieldName), 0);
        end
    end
end
