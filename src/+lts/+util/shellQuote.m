function value = shellQuote(value)
% SHELLQUOTE Quote one argument for MATLAB system() command strings.
value = char(string(value));
if isempty(value)
    value = '""';
    return;
end
if ispc
    value = ['"' strrep(value, '"', '\"') '"'];
else
    value = ['''' strrep(value, '''', '''"''"''') ''''];
end
end
