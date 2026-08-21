function [tirFile, extraArgs] = resolveTireArg(tirFile, ...
        defaultTirFile, optionNames, varargin)
%RESOLVETIREARG Default the optional leading tire-file argument.
%   Analysis scripts take an optional tirFile first argument; when the caller
%   passed an option name instead, shift it into extraArgs and use the
%   default. Pass extraArgs back through as varargin{:}.
if isempty(tirFile) || wsc.isOptionName(tirFile, optionNames)
    if wsc.isOptionName(tirFile, optionNames)
        extraArgs = [{tirFile}, varargin];
    else
        extraArgs = varargin;
    end
    tirFile = defaultTirFile;
else
    extraArgs = varargin;
end
end
