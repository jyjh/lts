function command = validatePythonCommand(command, caller)
% VALIDATEPYTHONCOMMAND Restrict a caller-supplied Python interpreter token.
%   command = lts.util.validatePythonCommand(command) returns the token only
%   if it is free of shell metacharacters, so placing it unquoted at the head
%   of a system() string cannot inject shell commands (e.g. 'python & calc').
%
%   The token may be a bare executable name ('python', 'python3') or an
%   absolute/relative path, but must not contain characters that cmd.exe /
%   sh interpret: & | < > ; ` $ ( ) % ! ^ " ' and newlines. A path with
%   spaces must be passed already quoted by the caller, or the caller should
%   switch to an unquoted filesystem path without spaces.
%
%   command = lts.util.validatePythonCommand(command, caller) tags the
%   error identifier with the caller name for diagnosis.
if nargin < 2 || isempty(caller)
    caller = 'lts_util_validatePythonCommand';
end
if ~(ischar(command) || isstring(command))
    error([caller ':InvalidPythonCommand'], ...
        'PythonCommand must be a character vector or string.');
end
token = char(command);
if isempty(strtrim(token))
    error([caller ':InvalidPythonCommand'], ...
        'PythonCommand must not be empty.');
end
% Reject shell metacharacters outright. This is stricter than trying to
% escape them portably across cmd.exe and POSIX shells, and PythonCommand is
% expected to be a simple interpreter name or path.
forbidden = ['&|<>;`$()%!^"' newline char(10) char(13)];
for i = 1:length(forbidden)
    if contains(token, forbidden(i))
        error([caller ':InvalidPythonCommand'], ...
            ['PythonCommand "%s" contains the shell metacharacter ''%s'', ' ...
            'which is rejected to prevent command injection. Pass a plain ' ...
            'interpreter name or path.'], token, forbidden(i));
    end
end
command = token;
end
