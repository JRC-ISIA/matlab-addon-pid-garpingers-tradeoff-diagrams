function segments = extractStabilityBoundaries(B, x, y)
%EXTRACTSTABILITYBOUNDARIES Extract outer boundary of a binary mask as (x,y).
%  [xb, yb] = EXTRACTSTABILITYBOUNDARIES(B, x, y) returns the coordinates
%  of the closed stability boundaries of B (logical matrix), using an 
%  isocontour at level 0.5. No toolboxes required.
%
%  Assumes size(B) == [numel(y), numel(x)].
%
% Notes:
%   - Created with support from Microsoft Copilot (GPT-5)

arguments
    B (:,:) logical {mustBeNonempty, mustBeNumericOrLogical}
    x (:,1) {mustBeNonempty, mustBeVector}
    y (:,1) {mustBeNonempty, mustBeVector}
end

[nr, nc] = size(B);
if nr ~= numel(y) || nc ~= numel(x)
    error("Size mismatch: size(B) = [%d %d], but numel(y)=%d, numel(x)=%d.", ...
          nr, nc, numel(y), numel(x));
end

% Compute contour at level 0.5 (boundary between false/true)

% Add columns on left and right
B = [zeros(size(B,1),1), double(B), zeros(size(B,1),1)];
% Add rows on top and bottom
B = [zeros(1,size(B,2)); double(B); zeros(1,size(B,2))];
x = [x(1)-(x(2)-x(1)); x; x(end)+(x(end)-(x(end-1)))];
y = [y(1)-(y(2)-y(1)); y; y(end)+(y(end)-(y(end-1)))];
C = contourc(x, y, B, [0.5 0.5]);

% Parse contour matrix C into segments
segments = parseContourMatrix(C);
if isempty(segments)
    warning("No stability boundary found");
    return;
end

% Ensure closure (optional: usually already closed)
for i = 1:numel(segments)
    seg = segments{i};  % Extract current segment (2 x N)
    if any(seg(:,1) ~= seg(:,end))
        seg = [seg, seg(:,1)];  %#ok<AGROW>
    end
    segments{i} = seg;
end

end

function segments = parseContourMatrix(C)
% Parse MATLAB contourc matrix into a cell array of [2 x N] coordinate arrays.

segments = {};
k = 1;
while k < size(C,2)
    level = C(1,k); %#ok<NASGU>  % not used since single level
    npts  = C(2,k);
    idx   = k + (1:npts);
    coords = C(:, idx);
    segments{end+1} = coords; %#ok<AGROW>
    k = idx(end) + 1;
end

end