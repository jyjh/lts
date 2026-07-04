function root = repoRoot(startPath)
% REPOROOT Locate the repository root from a file or folder path.
if nargin < 1 || isempty(startPath)
    startPath = mfilename('fullpath');
end

root = char(startPath);
if exist(root, 'file') == 2
    root = fileparts(root);
end

while ~isempty(root)
    if exist(fullfile(root, 'README.md'), 'file') && ...
            exist(fullfile(root, 'src', '+lts'), 'dir')
        return;
    end

    parent = fileparts(root);
    if strcmp(parent, root)
        break;
    end
    root = parent;
end

error('lts_util_repoRoot:NotFound', ...
    'Could not locate the LTS repository root from "%s".', char(startPath));
end
