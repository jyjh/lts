function command = shellJoin(args)
% SHELLJOIN Join command arguments after shell quoting each argument.
quoted = cell(size(args));
for i = 1:numel(args)
    quoted{i} = lts.util.shellQuote(args{i});
end
command = strjoin(quoted, ' ');
end
