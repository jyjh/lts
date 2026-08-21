function outputs = batchedMfeval(tireParams, inputsMF)
%BATCHEDMFEVAL Evaluate mfeval over arbitrarily many rows in bounded chunks.
%   Suppresses the expected 'Solver:Limits:Exceeded' warnings mfeval emits
%   for very small vertical loads (its internal lower limit sits near ~100 N;
%   those samples lie below any realistic inside-tire load).
nRows = size(inputsMF, 1);
maxRowsPerCall = 100000;
nChunks = ceil(nRows / maxRowsPerCall);
chunks = cell(max(nChunks, 0), 1);

warnState = warning('off', 'Solver:Limits:Exceeded');
cleanups = onCleanup(@() warning(warnState));
chunkIdx = 0;
for startIdx = 1:maxRowsPerCall:nRows
    endIdx = min(startIdx + maxRowsPerCall - 1, nRows);
    chunkIdx = chunkIdx + 1;
    chunks{chunkIdx} = mfeval(tireParams, inputsMF(startIdx:endIdx, :), 111);
end
clear cleanups;

if isempty(chunks)
    outputs = zeros(0, 4);
else
    outputs = vertcat(chunks{:});
end
end
