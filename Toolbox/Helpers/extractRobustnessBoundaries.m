function segments = extractRobustnessBoundaries(M, x, y, levels)
%EXTRACTROBUSTNESSBOUNDARIES Extract outer boundary of a binary mask as (x,y).
%  [xb, yb] = EXTRACTROBUSTNESSBOUNDARIES(, x, y) returns the coordinates
%  of the robustness boundaries of M, using an %  isocontour at given
%  levels. No toolboxes required. 
%
%  Assumes size(M) == [numel(y), numel(x)].
%
% Notes:
%   - Created with support from Microsoft Copilot (GPT-5)


arguments
    M (:,:) {mustBeNonempty, mustBeNumeric}
    x (:,1) {mustBeNonempty, mustBeVector}
    y (:,1) {mustBeNonempty, mustBeVector}
    levels (:,1) {mustBeNonempty, mustBeVector}
end

% Sanity check
[nr, nc] = size(M);
if nr ~= numel(y) || nc ~= numel(x)
    error("Size mismatch: size(M) = [%d %d], but numel(y)=%d, numel(x)=%d.", ...
          nr, nc, numel(y), numel(x));
end

% Calculate levels
finiteMask = isfinite(M);          % Logical mask for finite values
if ~any(finiteMask, 'all')
    error('Matrix contains no finite values.');
end

% Restrict to domain M >= 1.05
Mmasked = M;
Mmasked((Mmasked < 1.05) & finiteMask) = NaN;

% Keep only levels >= 1.05 (others can't exist in the masked domain anyway)
levels = levels(levels >= 1.05);
if isempty(levels)
    warning("No levels >= 1.05. Nothing to contour.");
    segments = {};
    return;
end

% Compute contour at given levels
C = contourc(x, y, Mmasked, levels);

% Parse contour matrix C into segments
segments = parseContourMatrix(C);
if isempty(segments)
    warning("No robustness boundary found");
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