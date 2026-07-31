function data = loadMatSafe(fileName, caller)
% LOADMATSAFE Load a .mat file after rejecting object/class variables.
%   data = lts.util.loadMatSafe(fileName) inspects the variables stored in
%   fileName via whos('-file') and errors if any variable is an object or
%   other non-data class, because MATLAB's load() reconstructs saved objects
%   by invoking their class constructor / loadobj, which executes arbitrary
%   code. Only plain data classes (numeric, logical, char, string, struct,
%   cell, table, datetime, duration) are permitted.
%
%   data = lts.util.loadMatSafe(fileName, caller) includes caller in the
%   error identifier prefix for diagnosis, e.g. 'EMRAX228Powertrain'.
%
%   This is a defense-in-depth control for .mat files whose path or origin
%   is not fully trusted (config-supplied paths, shared track/powertrain
%   maps). Field-level validation by the caller still runs after loading.
if nargin < 2 || isempty(caller)
    caller = 'loadMatSafe';
end
if ~ischar(fileName) && ~isstring(fileName)
    error('lts_util_loadMatSafe:InvalidPath', ...
        'File path must be a character vector or string.');
end
fileName = char(fileName);
if ~exist(fileName, 'file')
    error('lts_util_loadMatSafe:MissingFile', ...
        'MAT file does not exist: %s', fileName);
end

info = whos('-file', fileName);
if isempty(info)
    error('lts_util_loadMatSafe:EmptyFile', ...
        'MAT file contains no variables: %s', fileName);
end
% Allow-list of plain data classes that load() reconstructs without
% executing any user code. Anything else (an @class, a value object with
% loadobj, a Java/COM handle, etc.) is rejected before load() runs.
allowedClassPrefix = { ...
    'double', 'single', 'int', 'uint', ...
    'logical', 'char', 'string', 'cell', ...
    'struct', 'table', 'timetable', ...
    'datetime', 'duration', 'calendarDuration'};
for i = 1:numel(info)
    % whos('-file') returns the class under 'class' (and 'className' on
    % newer releases); accept either so the check is version-agnostic.
    if isfield(info, 'className')
        className = info(i).className;
    else
        className = info(i).class;
    end
    ok = false;
    for j = 1:numel(allowedClassPrefix)
        if startsWith(className, allowedClassPrefix{j})
            ok = true;
            break;
        end
    end
    if ~ok
        error(['lts_' caller '_loadMatSafe:ObjectVariable'], ...
            ['MAT file "%s" contains variable "%s" of class "%s", which is ' ...
            'not a plain data type. Object deserialization is rejected to ' ...
            'prevent arbitrary code execution during load().'], ...
            fileName, info(i).name, className);
    end
end

data = load(fileName, '-mat');
end
