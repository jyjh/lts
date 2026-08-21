function writeFigureToFile(fig, outputFile)
%WRITEFIGURETOFILE Export a figure, creating the output folder if needed.
[folder, ~, ~] = fileparts(outputFile);
if ~isempty(folder) && ~exist(folder, 'dir')
    mkdir(folder);
end

if exist('exportgraphics', 'file') == 2
    exportgraphics(fig, outputFile, 'Resolution', 160, ...
        'BackgroundColor', 'white');
else
    saveas(fig, outputFile);
end
end
