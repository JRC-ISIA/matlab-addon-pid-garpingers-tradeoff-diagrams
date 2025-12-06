function segments = extractPerformanceBoundaries(I, x, y, nLines)
%EXTRACTROBUSTNESSBOUNDARIES Extract outer boundary of a binary mask as (x,y).
%  [xb, yb] = EXTRACTROBUSTNESSBOUNDARIES(I, x, y, nLines) returns the 
%  coordinates of the robustness boundaries of I, using an isocontour at 
%  given levels. No toolboxes required. 
%
%  Assumes size(I) == [numel(y), numel(x)].
%
% Notes:
%   - Created with support from Microsoft Copilot (GPT-5)

arguments
    I (:,:) {mustBeNonempty, mustBeNumeric}
    x (:,1) {mustBeNonempty, mustBeVector}
    y (:,1) {mustBeNonempty, mustBeVector}
    nLines (1,1) double {mustBeNonempty, mustBePositive}
end

[nr, nc] = size(I);
if nr ~= numel(y) || nc ~= numel(x)
    error("Size mismatch: size(I) = [%d %d], but numel(y)=%d, numel(x)=%d.", ...
          nr, nc, numel(y), numel(x));
end


% Calculate levels
finiteMask = isfinite(I);          % Logical mask for finite values
if ~any(finiteMask, 'all')
    warning('Performance matrix contains no finite values. Performance boundaries are not plotted.');
    segments = {};
    return
end

% Replace Inf with NaN so they are ignored by min
I(~finiteMask) = NaN;

% Find the minimum finite value and its linear index
[~, linearIdx] = min(I(:), [], 'omitnan');

% Convert linear index to row and column
[rowIdx, colIdx] = ind2sub(size(I), linearIdx);

% Generate n indices strictly between 0 and IdxI
rowIdxStart = 1;
while ~isfinite(I(rowIdxStart, colIdx)) && (rowIdxStart < rowIdx)
    rowIdxStart = rowIdxStart + 1;
end
levelsRowIdx = floor(linspace(rowIdxStart, rowIdx, nLines + 2));  % creates n+2 points including endpoints
levelsRowIdx = levelsRowIdx(2:end-1);                  % remove first and last (minVal, maxVal
levelsIdx = sub2ind(size(I), levelsRowIdx, colIdx);
levels = I(levelsIdx);

% Compute contour at given levels
C = contourc(x, y, I, levels(~isnan(levels)));

% Parse contour matrix C into segments
segments = parseContourMatrix(C);
if isempty(segments)
    warning("No performance boundary found");
    return;
end

end

function segments = parseContourMatrix(C)
% Parse MATLAB contourc matrix into a cell array of [2 x N] coordinate arrays.

segments = {};
k = 1;
while k < size(C,2)
    level = C(1,k); % not used since single level
    npts  = C(2,k);
    idx   = k + (1:npts);
    coords = C(:, idx);
    coords = [coords; level * ones(1, size(coords,2))]; %#ok<AGROW>
    segments{end+1} = coords; %#ok<AGROW>
    k = idx(end) + 1;
end

end