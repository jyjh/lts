classdef TireConstants
    % TIRECONSTANTS Immutable Pacejka tire coefficients shared by all corners
    %
    % Reads a .tir file once via MFeval_readTIR and stores the resulting
    % coefficient structure. Since all four corners use the same tire,
    % a single TireConstants instance is shared across all TireState objects.
    %
    % Dependencies:
    %   MFeval toolbox — https://www.mathworks.com/matlabcentral/fileexchange/63618-mfeval
    %
    % Usage:
    %   tc = lts.components.Tire.TireConstants('43105_18x7.5_10_R25B_7.tir')
    %   tc.params       % MFeval_readTIR output structure
    %   tc.nomPressure  % Nominal inflation pressure [Pa]
    %   tc.nomLoad      % Nominal normal force [N]
    
    properties (SetAccess = immutable)
        % Path to the .tir file (for reference/debugging)
        tirFilePath
        
        % Full coefficient structure from MFeval_readTIR
        params
        
        % Nominal tire inflation pressure [Pa]
        nomPressure
        
        % Nominal normal force [N]
        nomLoad
        
        % Reference forward velocity [m/s]
        refVelocity
    end
    
    methods
        function obj = TireConstants(tirFilePath, varargin)
            % TIRECONSTANTS Construct from a .tir file
            %   TireConstants(tirFilePath)
            %   TireConstants(tirFilePath, 'Verbose', true)
            %
            %   tirFilePath — path to the .tir file. If relative, resolved
            %                 relative to this class's folder (+Tire/).
            %   'Verbose'   — print the loaded nominal operating point
            %                 (default false; the VehicleManager build
            %                 report already summarizes the tire).

            verboseParser = inputParser;
            verboseParser.addParameter('Verbose', false, ...
                @(x) islogical(x) || (isnumeric(x) && isscalar(x)));
            verboseParser.parse(varargin{:});
            verbose = logical(verboseParser.Results.Verbose);
            
            % Resolve relative paths: search +Tire/ → src/ → project root
            if ~startsWith(tirFilePath, '/') && ~startsWith(tirFilePath, '\') ...
                    && ~contains(tirFilePath, ':')
                % Try +Tire/ folder first
                tireFolder = fullfile(fileparts(mfilename('fullpath')), tirFilePath);
                if exist(tireFolder, 'file')
                    tirFilePath = tireFolder;
                else
                    % Try src/ folder
                    srcFolder = fullfile(fileparts(fileparts(mfilename('fullpath'))), tirFilePath);
                    if exist(srcFolder, 'file')
                        tirFilePath = srcFolder;
                    else
                        % Try project root
                        rootFolder = fullfile(lts.util.repoRoot(mfilename('fullpath')), tirFilePath);
                        if exist(rootFolder, 'file')
                            tirFilePath = rootFolder;
                        else
                            error('TireConstants:TirFileNotFound', ...
                                ['Tire data file "%s" was not found in +Tire/, src/, or the ' ...
                                 'repo root. The .tir files are untracked because they are fitted ' ...
                                 'from FSAE TTC member data; see src/+lts/+components/+Tire/README.md ' ...
                                 'for the required files.'], tirFilePath);
                        end
                    end
                end
            end
            obj.tirFilePath = tirFilePath;
            
            % Read TIR file using MFeval
            obj.params = mfeval.readTIR(obj.tirFilePath);

            % Extract nominal operating conditions from TIR data
            if isfield(obj.params, 'NOMPRES')
                obj.nomPressure = obj.params.NOMPRES;
            else
                obj.nomPressure = 84000;  % Default for FSAE: ~84 kPa gauge
            end
            
            if isfield(obj.params, 'FNOMIN')
                obj.nomLoad = obj.params.FNOMIN;
            else
                obj.nomLoad = 1080;  % Default nominal load [N]
            end
            
            if isfield(obj.params, 'LONGVL')
                obj.refVelocity = obj.params.LONGVL;
            else
                obj.refVelocity = 10;  % Default reference velocity [m/s]
            end
            
            if verbose
                fprintf('TireConstants: Loaded from %s\n', tirFilePath);
                fprintf('  Nominal pressure: %.0f Pa, Nominal load: %.0f N\n', ...
                    obj.nomPressure, obj.nomLoad);
            end
        end
    end
end
