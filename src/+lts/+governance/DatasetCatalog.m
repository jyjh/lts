classdef DatasetCatalog
    %DATASETCATALOG Validate whole-run calibration and validation evidence.

    properties (Constant)
        Schema = "lts.governance.dataset-catalog.v1"
        Roles = ["calibration", "baseline-validation", "transport-validation"]
    end

    methods (Static)
        function catalog = load(filePath)
            if ~exist(filePath, 'file')
                error('lts_governance_DatasetCatalog:MissingFile', ...
                    'Dataset catalog does not exist: %s', filePath);
            end
            try
                catalog = jsondecode(fileread(filePath));
            catch err
                error('lts_governance_DatasetCatalog:InvalidJson', ...
                    'Could not parse dataset catalog "%s": %s', ...
                    filePath, err.message);
            end
            catalog = lts.governance.DatasetCatalog.validate(catalog);
            catalog.sourceFile = char(filePath);
        end

        function catalog = validate(catalog)
            if ~isstruct(catalog) || ~isfield(catalog, 'schema') || ...
                    string(catalog.schema) ~= lts.governance.DatasetCatalog.Schema || ...
                    ~isfield(catalog, 'datasets')
                error('lts_governance_DatasetCatalog:InvalidSchema', ...
                    'Expected schema %s with datasets.', lts.governance.DatasetCatalog.Schema);
            end
            datasets = lts.governance.DatasetCatalog.asStructArray(catalog.datasets);
            ids = strings(numel(datasets), 1);
            for i = 1:numel(datasets)
                d = datasets(i);
                required = {'id', 'role', 'vehicleConfiguration', 'testDay', ...
                    'sourceFile', 'sha256', 'conditions', 'intervention'};
                for j = 1:numel(required)
                    if ~isfield(d, required{j})
                        error('lts_governance_DatasetCatalog:MissingField', ...
                            'Dataset %d is missing "%s".', i, required{j});
                    end
                end
                ids(i) = string(d.id);
                if ~any(string(d.role) == lts.governance.DatasetCatalog.Roles)
                    error('lts_governance_DatasetCatalog:InvalidRole', ...
                        'Dataset "%s" has invalid role "%s".', d.id, d.role);
                end
                digest = char(string(d.sha256));
                if isempty(regexp(digest, '^[0-9a-fA-F]{64}$', 'once'))
                    error('lts_governance_DatasetCatalog:InvalidHash', ...
                        'Dataset "%s" requires a 64-character SHA-256.', d.id);
                end
                if string(d.role) == "transport-validation" && isempty(d.intervention)
                    error('lts_governance_DatasetCatalog:MissingIntervention', ...
                        'Transport dataset "%s" requires a recorded intervention.', d.id);
                end
            end
            if numel(unique(ids)) ~= numel(ids)
                error('lts_governance_DatasetCatalog:DuplicateId', ...
                    'Dataset IDs must be unique.');
            end
            catalog.datasets = datasets;
        end

        function state = maximumCertification(catalog)
            roles = string({catalog.datasets.role});
            if any(roles == "transport-validation")
                state = "transport-validated";
            elseif any(roles == "baseline-validation")
                state = "baseline-validated";
            else
                state = "provisional";
            end
        end

        function report = verifySources(catalog, rootDirectory)
            if nargin < 2 || isempty(rootDirectory)
                rootDirectory = pwd;
            end
            canonicalRoot = lts.governance.DatasetCatalog.canonicalize(rootDirectory);
            datasets = catalog.datasets;
            report = repmat(struct('id', "", 'file', "", 'verified', false), ...
                numel(datasets), 1);
            for i = 1:numel(datasets)
                file = lts.governance.DatasetCatalog.resolveContainedPath( ...
                    char(datasets(i).sourceFile), rootDirectory, canonicalRoot, ...
                    datasets(i).id);
                actual = lts.governance.DatasetCatalog.sha256(file);
                if ~strcmpi(actual, char(datasets(i).sha256))
                    error('lts_governance_DatasetCatalog:HashMismatch', ...
                        'Dataset "%s" source hash does not match its catalog entry.', ...
                        datasets(i).id);
                end
                if isfield(datasets(i), 'companionSources') && ...
                        ~isempty(datasets(i).companionSources)
                    companions = datasets(i).companionSources;
                    if iscell(companions)
                        companions = vertcat(companions{:});
                    end
                    for j = 1:numel(companions)
                        companionFile = lts.governance.DatasetCatalog. ...
                            resolveContainedPath( ...
                            char(companions(j).sourceFile), rootDirectory, ...
                            canonicalRoot, datasets(i).id);
                        companionHash = ...
                            lts.governance.DatasetCatalog.sha256(companionFile);
                        if ~strcmpi(companionHash, char(companions(j).sha256))
                            error('lts_governance_DatasetCatalog:HashMismatch', ...
                                'Dataset "%s" companion hash does not match.', ...
                                datasets(i).id);
                        end
                    end
                end
                report(i) = struct('id', string(datasets(i).id), ...
                    'file', string(file), 'verified', true);
            end
        end
    end

    methods (Static, Access = private)
        function datasets = asStructArray(raw)
            if isempty(raw)
                datasets = struct('id', {}, 'role', {}, 'vehicleConfiguration', {}, ...
                    'testDay', {}, 'sourceFile', {}, 'sha256', {}, ...
                    'conditions', {}, 'intervention', {});
            elseif iscell(raw)
                datasets = vertcat(raw{:});
            else
                datasets = raw(:);
            end
        end

        function digest = sha256(file)
            fid = fopen(file, 'rb');
            if fid < 0
                error('lts_governance_DatasetCatalog:ReadFailed', ...
                    'Could not read dataset source: %s', file);
            end
            cleanup = onCleanup(@() fclose(fid));
            bytes = fread(fid, Inf, '*uint8');
            engine = java.security.MessageDigest.getInstance('SHA-256');
            engine.update(bytes);
            raw = typecast(engine.digest(), 'uint8');
            digest = lower(reshape(dec2hex(raw, 2).', 1, []));
        end

        function c = canonicalize(path)
            % CANONICALIZE Resolve a path to its canonical absolute form,
            %   collapsing '.'/'..' and resolving symlinks, via Java so it
            %   works the same on Windows and POSIX.
            c = char(java.io.File(path).getCanonicalPath());
        end

        function file = resolveContainedPath(sourceFile, rootDirectory, ...
                    canonicalRoot, datasetId)
            % RESOLVECONTAINEDPATH Resolve a catalog sourceFile against
            %   rootDirectory and assert the canonical path stays inside it.
            %   A sourceFile like '../../secret.mat' would otherwise let a
            %   catalog read (and with loadMatSafe, potentially load) files
            %   outside the repo. The SHA-256 check authenticates content, not
            %   path legitimacy, and the hash is itself read from the same
            %   untrusted JSON, so this confinement check is the real control.
            file = sourceFile;
            if ~isfile(file)
                file = fullfile(rootDirectory, file);
            end
            if ~isfile(file)
                error('lts_governance_DatasetCatalog:MissingSource', ...
                    'Dataset "%s" source file does not exist: %s', ...
                    datasetId, file);
            end
            canonicalFile = lts.governance.DatasetCatalog.canonicalize(file);
            if ~startsWith(canonicalFile, [canonicalRoot, filesep]) && ...
                    ~strcmp(canonicalFile, canonicalRoot)
                error('lts_governance_DatasetCatalog:PathOutsideRoot', ...
                    ['Dataset "%s" source file "%s" resolves outside the ' ...
                    'catalog root directory ("%s"). Source paths must not ' ...
                    'escape the repository.'], datasetId, sourceFile, ...
                    canonicalRoot);
            end
            file = canonicalFile;
        end
    end
end
