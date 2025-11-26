function labelPerformanceLines(ax, x, y, label, color)
%LABELPERFORMANCELINES Place a text label with an arrow pointing to the curve.
%  ax       - target axes handle
%  x, y     - curve coordinates (1xN or Nx1)
%  labelStr - text to display
%  color    - (optional) RGB or color char; defaults to black
%
% Notes:
%   - Created with support from Microsoft Copilot (GPT-5)

arguments
    ax (1,1) matlab.graphics.axis.Axes {mustBeNonempty}
    x double {mustBeVector}
    y double {mustBeVector}
    label (1,1) string
    color (1,:) {mustBeVector} = [0 0 0]
end

x = x(:).'; y = y(:).';  % row vectors

% --- Choose target point on the curve
[~, idx] = min(x);

x0 = x(idx); 
y0 = y(idx);

% Arrow and text
text(ax, x0, y0, ...
    compose('%s%s', label, '\rightarrow'),...
    'Color', color, 'HorizontalAlignment', 'right', 'VerticalAlignment', 'middle');

end