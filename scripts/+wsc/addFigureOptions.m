function addFigureOptions(parser, defaultOutputFile)
%ADDFIGUREOPTIONS Register the OutputFile/SavePlot/ShowFigure/CloseFigure
%   options shared by every analysis-script inputParser.
parser.addParameter('OutputFile', defaultOutputFile, ...
    @(x) ischar(x) || isstring(x));
parser.addParameter('SavePlot', true, @wsc.validateScalarLogical);
parser.addParameter('ShowFigure', true, @wsc.validateScalarLogical);
parser.addParameter('CloseFigure', false, @wsc.validateScalarLogical);
end
