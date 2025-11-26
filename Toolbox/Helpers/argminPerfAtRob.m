function [rowIdx, colIdx, linearIdx, minPerfVal, robVal] = argminPerfAtRob(robMat, perfMat, rob)
%ARGMINPERFATROB  Index of the minimum performance value where robustness
%matches 'rob'. 
%[rowIdx, colIdx, linearIdx, minPerfVal] = ARGMINPERFATSTAB(robMat, perfMat, rob) 
%finds all positions where robMat >= rob, ignores non-finite entries in
%both matrices, then returns the index of the minimum perfMat among those positions.
%
%   Inputs:
%     robMat - Robustness matrix (Mst values)
%     perfMat - Performance matrix (IAE values)
%     rob - Robustness value to be matched
%
%   Outputs:
%     rowIdx - row index of the selected position (empty [] if none found)
%     colIdx - column index of the selected position (empty [] if none found)
%     linearIdx  - linear index (empty [] if none found)
%     minPerfVal - minimum performance value at the selected robustness value (empty [] if none)
%     robVal - fouind robustness value at determined position (empty [] if none)
%
%   Notes:
%     - Non-finite values (Inf, -Inf, NaN) in either matrix are ignored.
%     - If multiple minima tie, the first in linear indexing order is returned.
%     - If 'rob' itself is non-finite, the function returns empty outputs.
%     - Created with support from Microsoft Copilot (GPT-5)


arguments
    robMat {mustBeNonempty}
    perfMat {mustBeNonempty}
    rob (1,1) double {mustBeNonempty, mustBeNumeric, mustBePositive}
end

% Default empty outputs
rowIdx = []; colIdx = []; linearIdx = []; minPerfVal = [];

% Basic checks
if ~isequal(size(robMat), size(perfMat))
    warning('robMat and perfMat must have the same size. Returning empty outputs.');
    return
end

% Finite-only mask in both matrices
finiteMask = isfinite(robMat) & isfinite(perfMat);

% Match positions in robMat
matchMask = (rob >= robMat);

% Candidates: finite in BOTH matrices AND match 'rob'
candidateMask = finiteMask & matchMask;

if ~any(candidateMask, 'all')
    % Nothing to do; return empties
    return
end

% Among candidates, get perf values and find the minimum
linIdx = find(candidateMask);            % linear indices of candidates
perfVals = perfMat(linIdx);              % guaranteed finite because of finiteMask

[minPerfVal, k] = min(perfVals);         % min over finite candidates
linearIdx = linIdx(k);
robVal = robMat(linearIdx);
[rowIdx, colIdx] = ind2sub(size(robMat), linearIdx);

end