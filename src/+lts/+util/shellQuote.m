function value = shellQuote(value)
% SHELLQUOTE Quote one argument for MATLAB system() command strings.
%   Wraps the value in quotes and escapes characters that cmd.exe / POSIX
%   shells interpret, so a free-text field (driver name, comment, event)
%   cannot break out of its argument and inject commands. This is a
%   defense-in-depth control; callers should additionally validate any
%   executable token separately (see lts.util.validatePythonCommand).
value = char(string(value));
if isempty(value)
    value = '""';
    return;
end
if ispc
    % MATLAB system() on Windows routes through cmd.exe. Inside double
    % quotes, cmd.exe still interprets several characters; escape each
    % with '^' (cmd's escape char) and escape embedded double quotes. A
    % trailing backslash would escape the closing quote, so double it.
    if endsWith(value, '\')
        value = [value '\'];
    end
    value = regexprep(value, '(["&|<>%^();`$])', '^$1');
    value = ['"' value '"'];
else
    value = ['''' strrep(value, '''', '''"''"''') ''''];
end
end
