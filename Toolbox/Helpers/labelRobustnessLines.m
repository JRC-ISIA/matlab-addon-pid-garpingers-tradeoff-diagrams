function labelRobustnessLines(ax, x, y, xPosSpec, label, color)
%LABELROBUSTNESSLINES Place a text label with an arrow pointing to the curve.
%  ax       - target axes handle
%  x, y     - curve coordinates (1xN or Nx1)
%  xPosSpec  - position specifier: target x-value (nearest point used)
%  labelStr - text to display
%  color    - (optional) RGB or color char; defaults to black
%
% Notes:
%   - Created with support from Microsoft Copilot (GPT-5)


arguments
    ax (1,1) matlab.graphics.axis.Axes {mustBeNonempty}
    x double {mustBeVector}
    y double {mustBeVector}
    xPosSpec double {mustBeNonempty}
    label (1,1) string
    color (1,:) {mustBeVector} = [0 0 0]
end

x = x(:).'; y = y(:).';  % row vectors

% --- Choose target point on the curve
[~, idx] = min(abs(x - xPosSpec));

x0 = x(idx); y0 = y(idx);

% Arrow and text
text(ax, x0, y0, ...
    compose('%s%s', '\leftarrow', label),...
    'Color', color, 'HorizontalAlignment', 'left', 'VerticalAlignment', 'middle');

end