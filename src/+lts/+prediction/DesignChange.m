classdef DesignChange
    %DESIGNCHANGE Apply complete, provenance-backed vehicle interventions.

    properties (Constant)
        Schema = "lts.prediction.design-change.v1"
    end

    methods (Static)
        function change = load(filePath)
            if ~exist(filePath, 'file')
                error('lts_prediction_DesignChange:MissingFile', ...
                    'Design change file does not exist: %s', filePath);
            end
            try
                raw = jsondecode(fileread(filePath));
            catch err
                error('lts_prediction_DesignChange:InvalidJson', ...
                    'Could not parse design change "%s": %s', ...
                    filePath, err.message);
            end
            change = lts.prediction.DesignChange.validate(raw);
        end

        function change = validate(change)
            if ~isstruct(change) || ~isfield(change, 'schema') || ...
                    string(change.schema) ~= lts.prediction.DesignChange.Schema || ...
                    ~isfield(change, 'id') || ~isfield(change, 'source') || ...
                    ~isfield(change, 'provenance') || ~isfield(change, 'operations')
                error('lts_prediction_DesignChange:InvalidSchema', ...
                    'Design change requires schema, id, source, provenance, and operations.');
            end
            if strlength(string(change.source)) == 0 || ...
                    strlength(string(change.provenance)) == 0
                error('lts_prediction_DesignChange:MissingProvenance', ...
                    'Design changes require source and provenance.');
            end
            operations = lts.prediction.DesignChange.asStructArray(change.operations);
            for i = 1:numel(operations)
                if ~isfield(operations(i), 'type')
                    error('lts_prediction_DesignChange:MissingType', ...
                        'Operation %d has no type.', i);
                end
                type = string(operations(i).type);
                if type == "mass"
                    lts.prediction.DesignChange.requireFields(operations(i), ...
                        {'deltaMassKg', 'xFromCgM', 'yFromCgM', 'zFromGroundM'});
                elseif type == "aero_map"
                    lts.prediction.DesignChange.requireFields(operations(i), ...
                        {'ClA', 'CdA', 'xPosition', 'zPosition', ...
                        'pitchSensitivityClA'});
                elseif type == "parameter"
                    lts.prediction.DesignChange.requireFields(operations(i), ...
                        {'path', 'value'});
                else
                    error('lts_prediction_DesignChange:UnknownType', ...
                        'Unknown design operation "%s".', type);
                end
            end
            change.operations = operations;
        end

        function variant = apply(governed, change)
            change = lts.prediction.DesignChange.validate(change);
            if ~isstruct(governed) || ~isfield(governed, 'schema') || ...
                    string(governed.schema) ~= "lts.governance.governed-vehicle.v1"
                error('lts_prediction_DesignChange:UngovernedBaseline', ...
                    'Design changes require a governed baseline vehicle.');
            end
            config = governed.config;
            for i = 1:numel(change.operations)
                op = change.operations(i);
                type = string(op.type);
                if type == "mass"
                    lts.governance.ParameterManifest.assertDesignParameter( ...
                        governed.manifest, 'totalMass');
                    config = lts.prediction.DesignChange.applyMass(config, op);
                elseif type == "aero_map"
                    paths = ["aero.ClA", "aero.CdA", "aero.xPosition", ...
                        "aero.zPosition", "aero.pitchSensitivityClA"];
                    for j = 1:numel(paths)
                        lts.governance.ParameterManifest.assertDesignParameter( ...
                            governed.manifest, paths(j));
                    end
                    config.aero = struct('ClA', double(op.ClA), ...
                        'CdA', double(op.CdA), ...
                        'xPosition', double(op.xPosition), ...
                        'zPosition', double(op.zPosition), ...
                        'pitchSensitivityClA', double(op.pitchSensitivityClA));
                else
                    lts.governance.ParameterManifest.assertDesignParameter( ...
                        governed.manifest, op.path);
                    config = lts.governance.ParameterManifest.setValue( ...
                        config, op.path, double(op.value));
                end
            end
            variant = governed;
            variant.config = config;
            variant.change = change;
            variant.domainWarnings = lts.governance.ParameterManifest.domainWarnings( ...
                governed.manifest, config);
        end
    end

    methods (Static, Access = private)
        function config = applyMass(config, op)
            oldMass = double(config.totalMass);
            delta = double(op.deltaMassKg);
            newMass = oldMass + delta;
            x = double(op.xFromCgM);
            y = double(op.yFromCgM);
            z = double(op.zFromGroundM);
            if ~isfinite(newMass) || newMass <= 0 || ...
                    any(~isfinite([delta, x, y, z])) || z < 0
                error('lts_prediction_DesignChange:InvalidMass', ...
                    'Mass intervention produces invalid mass or location.');
            end
            dx = delta * x / newMass;
            dy = delta * y / newMass;
            oldRearArm = config.staticFrontWeight * config.wheelbase;
            config.totalMass = newMass;
            config.staticFrontWeight = lts.util.clamp( ...
                (oldRearArm + dx) / config.wheelbase, 0, 1);
            config.cgHeight = (oldMass * config.cgHeight + delta * z) / newMass;
            config.yawInertia = config.yawInertia + delta * (x^2 + y^2) - ...
                newMass * (dx^2 + dy^2);
            if config.cgHeight <= 0 || config.yawInertia <= 0
                error('lts_prediction_DesignChange:InvalidMassProperties', ...
                    'Mass intervention produces invalid CG height or yaw inertia.');
            end
        end

        function operations = asStructArray(raw)
            if isempty(raw)
                error('lts_prediction_DesignChange:NoOperations', ...
                    'A design change must contain at least one operation.');
            elseif iscell(raw)
                operations = vertcat(raw{:});
            else
                operations = raw(:);
            end
        end

        function requireFields(value, names)
            for i = 1:numel(names)
                if ~isfield(value, names{i})
                    error('lts_prediction_DesignChange:IncompleteOperation', ...
                        'Operation "%s" requires "%s".', value.type, names{i});
                end
            end
        end
    end
end
