function coupledFile = coupledOutputFile(baseFile)
%COUPLEDOUTPUTFILE Mirror the base figure path with a _coupled_cg suffix.
[folder, name, ext] = fileparts(baseFile);
coupledFile = fullfile(folder, [name '_coupled_cg' ext]);
end
