function [figDistRej, figSetTrack] = plotTradeoff(data, opts)
%PLOTTRADEOFF Visualizes trade-off diagrams for robustness and control
%performance according to Garpinger
%
%   Description:
%       This function generates two plots illustrating the trade-off
%       between robustness (M_st) and control performance (IAE) for a given
%       set of PID controller gains. One plot is for disturbance rejection,
%       and the other for setpoint tracking. 
%
%   Inputs:
%       data.kp - Proportional gains of PID controller
%       data.ki - Integral gains of PID controller
%       data.kd - Derivative gains of PID controller
%       data.stabMat - Stability matrix
%       data.robMat - Robustness matrix (M_st)
%       data.perfMatDistRej - Performance matrix (IAE) for disturbance rejection
%       data.perfMatSetTrack - Performance matrix (IAE) for setpoint tracking 
%
%   Name-Value Pairs:
%       'RobustnessLevels' - Vector of robustness values (Mst) for
%         which contour lines are plotted. Values exceeding the optimal
%         robustness will be ignored.
%       'NumberOfPerformanceLines' - Number of contour lines to be drawn
%         for performance. These lines represent levels of integrated
%         absolute error (IAE) in the plot. 
%       'ExternalFigureWindow' - Display trade off plots additionally in
%         external  window.
%       'UpsampleScale' - Either a numeric vector [nUpY nUpX] specifying
%         the bilinear upsampling scale of input matrices, or a string:
%         'off', 'low', 'medium', 'high'. 
%       'SmoothParetoFrontMethod' - Smoothing method for pareto front,
%         specified as one of these values: 
%         "off" - Pareto Front is not smoothed. 
%         "movmean" — Average over each window of A. This method is useful
%         for reducing periodic trends in data. 
%         "movmedian" — Median over each window of A. This method is useful
%         for reducing periodic trends in data when outliers are present. 
%         "gaussian" — Gaussian-weighted average over each window of A. 
%         "lowess" — Linear regression over each window of A. This method
%         can be computationally expensive, but results in fewer
%         discontinuities.  
%         "loess" — Quadratic regression over each window of A. This method
%         is slightly more computationally expensive than "lowess". 
%         "rlowess" — Robust linear regression over each window of A. This
%         method is a more computationally expensive version of the method
%         "lowess", but it is more robust to outliers.  
%         "rloess" — Robust quadratic regression over each window of A.
%         This method is a more computationally expensive version of the
%         method "loess", but it is more robust to outliers.  
%         "sgolay" — Savitzky-Golay filter, which smooths according to a
%         quadratic polynomial that is fitted over each window of A. This
%         method can be more effective than other methods when the data
%         varies rapidl.
%       'SmoothParetoFrontWindow' - Window size, specified as a positive
%         integer or duration scalar or two-element vector of nonnegative
%         integer or duration values. smoothdata defines the window
%         relative to the sample points.   
%         When window is a positive integer scalar, then the window has
%         length window and is centered about the current element. 
%         When window is a two-element vector of nonnegative integers [b
%         f], the window contains the current element, b preceding
%         elements, and f succeeding elements.  
%
%   Outputs:
%       figDistRej - Figure handle for disturbance rejection trade-off plot
%         matlab.ui.figure
%       figSetTrack - Figure handle for setpoint tracking trade-off plot
%         matlab.ui.figure
%
% Notes:
%   - Created with support from Microsoft Copilot (GPT-5)


%% Name-Value definition & validation

arguments
    data struct
    % data.kp (1,:) double {mustBeFinite, mustBeNonempty}
    % data.ki (1,:) double {mustBeFinite, mustBeNonempty}
    % data.kd (1,:) double {mustBeFinite, mustBeNonempty}
    % data.stabMat logical {mustBeFinite, mustBeNonempty}
    % data.robMat {mustBeNonempty}
    % data.perfMatDistRej {mustBeNonempty}
    % data.perfMatSetTrack {mustBeNonempty}
    
    % Name-Value pairs:
    opts.RobustnessLevels double = [1.1 1.2 1.4 1.6 2.0 2.5]
    opts.NumberOfPerformanceLines double = 4
    opts.ExternalFigureWindow logical = false
    opts.UpsampleScale = "low"
    opts.SmoothParetoFrontMethod = "movmean"
    opts.SmoothParetoFrontWindow (1,1) double = 5
end

if ~any(isfinite(data.kp), "all") || isempty(data.kp) || ~isnumeric(data.kp)
    error("Field 'kp' must be a finite, nonempty, numeric vector")
end

if ~any(isfinite(data.ki), "all") || isempty(data.ki) || ~isnumeric(data.ki)
    error("Field 'ki' must be a finite, nonempty, numeric vector")
end

if ~any(isfinite(data.kd), "all") || isempty(data.kd) || ~isnumeric(data.kd)
    error("Field 'kd' must be a finite, nonempty, numeric vector")
end

if isempty(data.stabMat)
    error("Field 'stabMat' must be a nonempty matrix")
end

if isempty(data.robMat)
    error("Field 'robMat' must be a nonempty matrix")
end

enablePlotDistRej = true;
if isempty(data.perfMatDistRej)
    warning("Plotting trade-off diagram for disturbance rejection is skipped because no performance lines has been identified.")
    enablePlotDistRej = false;
end

enablePlotSetTrack = true;
if isempty(data.perfMatSetTrack)
    warning("Plotting trade-off diagram for setpoint tracking is skipped because no performance lines has been identified.")
    enablePlotSetTrack = false;
end


%% Read out resolution specification
if isnumeric(opts.UpsampleScale)
    if numel(opts.UpsampleScale) ~= 2
        error("plotTradeoff:InvalidNumUpsDim", ...
            "Bilinear upsample scale must be a vector of size [nY nX].");
    end
    nUpscaleY = opts.UpsampleScale(1);
    nUpscaleX = opts.UpsampleScale(2);
elseif ischar(opts.UpsampleScale) || isstring(opts.UpsampleScale)
    switch lower(char(opts.UpsampleScale))
        case 'off'
            nUpscaleX = 1; nUpscaleY = 1;
        case 'low'
            nUpscaleX = 2; nUpscaleY = 2;
        case 'medium'
            nUpscaleX = 5; nUpscaleY = 5;
        case 'high'
            nUpscaleX = 10; nUpscaleY = 10;
        otherwise
            error("plotTradeoff:InvalidUpsStr", ...
                "Unknown bilinear upsample scale string: " + opts.UpsampleScale);
    end
else        
    error("plotTradeoff:InvalidUpsType", ...
        "Invalid input type for 'UpsampleScale'.");
end


%% Identify which controller gains need to be applied via which axes

nKp = numel(data.kp);
nKi = numel(data.ki);
nKd = numel(data.kd);

sz = size(data.stabMat);
if numel(sz) < 2
    error('garpinger:InvalidStabilityMatrixSize', ...
          'stabMat must be at least 2D (got %dD).', ndims(data.stabMat));
end
sz = sz(1:2);

if (nKp > 1) && (nKi > 1) && (nKd == 1) && isequal(sz, [nKi, nKp])
    plotType = "kp-ki";
elseif (nKp > 1) && (nKi == 1) && (nKd > 1) && isequal(sz, [nKp, nKd])
    plotType = "kd-kp";
else
    error('garpinger:AutoDetectFailed', ...
        ['Failed to auto-detect plot type.\n' ...
         'Expected one of:\n' ...
         '  A) kp & ki are vectors, kd is scalar, size(stabMat) = [numel(ki)  numel(kp)]  -> "kp-ki"\n' ...
         '  B) ki is scalar, kd is vector, size(stabMat) = [numel(kp)  numel(kd)]        -> "kd-kp"\n' ...
         'Got: size(stabMat) = [%d  %d], numel(kp)=%d, numel(ki)=%d, numel(kd)=%d.'], ...
         sz(1), sz(2), nKp, nKi, nKd);
end


%% Validate other matrices once plotType is known

switch plotType
    case "kp-ki"
        expectedSz = [numel(data.ki), numel(data.kp)];
        matrices = {data.stabMat, data.robMat, data.perfMatDistRej, data.perfMatSetTrack};
        names    = {'stabMat','robMat','perfMatDistRej','perfMatSetTrack'};
        for i = 1:numel(matrices)
            if ~isequal(size(matrices{i}), expectedSz) && ~isempty(matrices{i})
                error('garpinger:SizeMismatch', ...
                    '%s must be sized [%d x %d] for plot type "kp-ki" (got [%d x %d]).', ...
                    names{i}, expectedSz(1), expectedSz(2), size(matrices{i},1), size(matrices{i},2));
            end
        end

    case "kd-kp"
        expectedSz = [numel(data.kp), numel(data.kd)];
        matrices = {data.stabMat, data.robMat, data.perfMatDistRej, data.perfMatSetTrack};
        names    = {'stabMat','robMat','perfMatDistRej','perfMatSetTrack'};
        for i = 1:numel(matrices)
            if ~isequal(size(matrices{i}), expectedSz) && ~isempty(matrices{i})
                error('garpinger:SizeMismatch', ...
                    '%s must be sized [%d x %d] for plot type "kd-kp" (got [%d x %d]).', ...
                    names{i}, expectedSz(1), expectedSz(2), size(matrices{i},1), size(matrices{i},2));
            end
        end
end


%% Prepare figure


% Prepare figures
if opts.ExternalFigureWindow
    if enablePlotDistRej
        figDistRej  = figure('Color', 'w', 'WindowStyle', 'normal', 'Visible', 'on');
        axDistRej = axes('Color', [0.7 0.7 0.7]);  % slightly lighter gray
        figDistRej.Position(3:4) = [400 400];
    else
        figDistRej = matlab.graphics.GraphicsPlaceholder;
        axDistRej = matlab.graphics.GraphicsPlaceholder;
    end
    if enablePlotSetTrack
        figSetTrack = figure('Color', 'w', 'WindowStyle', 'normal', 'Visible', 'on');
        axSetTrack = axes('Color', [0.7 0.7 0.7]);  % slightly lighter gray
        figSetTrack.Position(3:4) = [400 400];
    else
        figSetTrack = matlab.graphics.GraphicsPlaceholder;
        axSetTrack = matlab.graphics.GraphicsPlaceholder;
    end
else
    if enablePlotDistRej
        figDistRej  = figure('Color', 'w', 'WindowStyle', 'normal');
        axDistRej = axes('Color', [0.7 0.7 0.7]);  % slightly lighter gray
        figDistRej.Position(3:4) = [400 400];
    else
        figDistRej = matlab.graphics.GraphicsPlaceholder;
        axDistRej = matlab.graphics.GraphicsPlaceholder;
    end
    if enablePlotSetTrack
        figSetTrack = figure('Color', 'w', 'WindowStyle', 'normal');
        axSetTrack = axes('Color', [0.7 0.7 0.7]);  % slightly lighter gray
        figSetTrack.Position(3:4) = [400 400];
    else
        figSetTrack = matlab.graphics.GraphicsPlaceholder;
        axSetTrack = matlab.graphics.GraphicsPlaceholder;
    end
end



fig = [figDistRej, figSetTrack];
ax = [axDistRej, axSetTrack];

pStab = [matlab.graphics.GraphicsPlaceholder, matlab.graphics.GraphicsPlaceholder];
pRob = [matlab.graphics.GraphicsPlaceholder, matlab.graphics.GraphicsPlaceholder];
pPerf = [matlab.graphics.GraphicsPlaceholder, matlab.graphics.GraphicsPlaceholder];
pPfDr = [matlab.graphics.GraphicsPlaceholder, matlab.graphics.GraphicsPlaceholder];
pPfSt = [matlab.graphics.GraphicsPlaceholder, matlab.graphics.GraphicsPlaceholder];

switch plotType
    case "kp-ki"
        assert(isscalar(data.kd), 'kd must be a scalar if ki is to be plotted via kp.');
        kx = data.kp;
        ky = data.ki;
        if enablePlotDistRej
            figure(figDistRej)
            title("Disturbance Rejection ($K_\mathrm{D}=" + string(data.kd) + "$)", 'Interpreter', 'latex');
        end
        if enablePlotSetTrack
            figure(figSetTrack)
            title("Setpoint Tracking ($K_\mathrm{D}=" + string(data.kd) + "$)", 'Interpreter', 'latex');
        end
        for iFig = 1:2
            if isequal(fig(iFig), matlab.graphics.GraphicsPlaceholder)
                continue
            end
            figure(fig(iFig))
            xlim([min(data.kp), max(data.kp)])
            ylim([min(data.ki), max(data.ki)])
            xlabel('$K_\mathrm{P}$', 'Interpreter', 'latex')
            ylabel('$K_\mathrm{I}$', 'Interpreter', 'latex')
            hold on;
        end

    case "kd-kp"
        assert(isscalar(data.ki), 'ki must be a scalar if kp is to be plotted via kd.');
        kx = data.kd;
        ky = data.kp;
        if enablePlotDistRej
            figure(figDistRej)
            title("Disturbance Rejection ($K_\mathrm{I}=" + string(data.ki) + "$)", 'Interpreter', 'latex');
        end
        if enablePlotSetTrack
            figure(figSetTrack)
            title("Setpoint Tracking ($K_\mathrm{I}=" + string(data.ki) + "$)", 'Interpreter', 'latex');
        end
        for iFig = 1:2
            if isequal(fig(iFig), matlab.graphics.GraphicsPlaceholder)
                continue
            end
            figure(fig(iFig))
            xlim([min(data.kd), max(data.kd)])
            ylim([min(data.kp), max(data.kp)])
            xlabel('$K_\mathrm{D}$', 'Interpreter', 'latex')
            ylabel('$K_\mathrm{P}$', 'Interpreter', 'latex')  
            hold on;
        end
    otherwise
        assert(false, 'Plant type is not supported.');
end


%% Plot stability region

stabSegs = extractStabilityBoundaries(data.stabMat, kx, ky);

for iFig = 1:2
    if isequal(fig(iFig), matlab.graphics.GraphicsPlaceholder)
        continue
    end
    figure(fig(iFig))
    pStabCell = cellfun(@(seg) patch(seg(1,:), seg(2,:), 'white', 'FaceAlpha', 1.0, 'DisplayName', ''), stabSegs); 
    pStab(iFig) = pStabCell(1); 
end


%% Upscale data

nX = numel(kx);
nXFine = nUpscaleX * nX;
kxFine = linspace(min(kx), max(kx), nXFine);
nY = numel(ky);
nYFine = nUpscaleY * nY;
kyFine = linspace(min(ky), max(ky), nYFine);
[kxMesh, kyMesh] = meshgrid(kx, ky);
[kxFineMesh, kyFineMesh] = meshgrid(kxFine, kyFine);
robMatFine = interp2(kxMesh, kyMesh, data.robMat, kxFineMesh, kyFineMesh);
if ~isempty(data.perfMatDistRej)
    perfMatDistRejFine = interp2(kxMesh, kyMesh, data.perfMatDistRej, kxFineMesh, kyFineMesh);
    signedPerfMatDistRejFine = interp2(kxMesh, kyMesh, data.signedPerfMatDistRej, kxFineMesh, kyFineMesh);
end
if ~isempty(data.perfMatSetTrack)
    perfMatSetTrackFine = interp2(kxMesh, kyMesh, data.perfMatSetTrack, kxFineMesh, kyFineMesh);
    signedPerfMatSetTrackFine = interp2(kxMesh, kyMesh, data.signedPerfMatSetTrack, kxFineMesh, kyFineMesh);
end


%% Plot robustness lines

% Extract robustnes lines
robSegs = extractRobustnessBoundaries(robMatFine, kxFine, kyFine, opts.RobustnessLevels);

% Draw robustness lines
for iFig = 1:2
    if isequal(fig(iFig), matlab.graphics.GraphicsPlaceholder)
        continue
    end
    figure(fig(iFig))
    level = 0;
    for i = 1:numel(robSegs)
                
        % Get x,y from your cell
        x = robSegs{i}(1,:);
        y = robSegs{i}(2,:);
    
        % Plot the line and store color for consistent labeling
        pRob(iFig) = plot(ax(iFig), x, y, 'r', 'DisplayName', ''); 
        col = pRob(iFig).Color;
        
        % Check whether the robustness value has already been labeled
        if isequal(level, robSegs{i}(3,1))
            %continue
        end
        level = robSegs{i}(3,1);

        % Choose label position per line:
        pos = x(1) + 0.1 * (x(end) - x(1));
            
        % Label text with a fixed value (example)
        lbl = num2str(level, '%.2f');
    
        labelRobustnessLines(ax(iFig), x, y, pos, lbl, col);
    end
end


%% Plot performance lines

% Extract robustnes lines
if enablePlotDistRej
    perfSegs{1} = extractPerformanceBoundaries(perfMatDistRejFine, kxFine, kyFine, opts.NumberOfPerformanceLines);
end
if enablePlotSetTrack
    perfSegs{2} = extractPerformanceBoundaries(perfMatSetTrackFine, kxFine, kyFine, opts.NumberOfPerformanceLines);
end

% Draw robustness lines
for iFig = 1:2
    if isequal(fig(iFig), matlab.graphics.GraphicsPlaceholder)
        continue
    end
    figure(fig(iFig))
    level = 0;
    for i = 1:numel(perfSegs{iFig})
                
        % Get x,y from your cell
        x = perfSegs{iFig}{i}(1,:);
        y = perfSegs{iFig}{i}(2,:);
    
        % Plot the line and store color for consistent labeling
        pPerf(iFig) = plot(ax(iFig), x, y, 'b', 'DisplayName', ''); 
        col = pPerf(iFig).Color;
        
        % Check whether the robustness value has already been labeled
        if isequal(level, perfSegs{iFig}{i}(3,1))
            continue
        end
        level = perfSegs{iFig}{i}(3,1);
   
        % Label text with a fixed value (example)
        lbl = num2str(level, '%.3f');
    
        labelPerformanceLines(ax(iFig), x, y, lbl, col);
    end
end


%% Plot Pareto fronts

% Compute pareto front
paretoDistRej = [];
foundParetoDistRejOptimum = false;
if enablePlotDistRej
    [paretoDistRej, foundParetoDistRejOptimum] = ...
        extractParetoFront(robMatFine, perfMatDistRejFine, kxFine, kyFine, ...
        "SmoothMethod", opts.SmoothParetoFrontMethod, ...
        "SmoothWindow", opts.SmoothParetoFrontWindow);
end

paretoSetTrack = [];
foundParetoSetTrackOptimum = false;
if enablePlotSetTrack
    [paretoSetTrack, foundParetoSetTrackOptimum] = ...
        extractParetoFront(robMatFine, perfMatSetTrackFine, kxFine, kyFine, ...
        "SmoothMethod", opts.SmoothParetoFrontMethod, ...
        "SmoothWindow", opts.SmoothParetoFrontWindow);
end

% Draw pareto fronts
for iFig = 1:2
    if isequal(fig(iFig), matlab.graphics.GraphicsPlaceholder)
        continue
    end    
    figure(fig(iFig))
    if ~isempty(paretoDistRej)
        pPfDr(iFig) = plot(paretoDistRej(1,:), paretoDistRej(2,:), 'g', 'DisplayName', ''); 
        if foundParetoDistRejOptimum
            plot(paretoDistRej(1,end), paretoDistRej(2,end), 'go', 'DisplayName', '');
        end
    end
    if ~isempty(paretoSetTrack)
        pPfSt(iFig) = plot(paretoSetTrack(1,:), paretoSetTrack(2,:), 'm', 'DisplayName', ''); 
        if foundParetoSetTrackOptimum
            plot(paretoSetTrack(1,end), paretoSetTrack(2,end), 'mo', 'DisplayName', '');
        end
    end
end

% Legend
for iFig = 1:2
    if isequal(fig(iFig), matlab.graphics.GraphicsPlaceholder)
        continue
    end    
    pTradeoff = [];
    strTradeoff = {};
    
    if ~isempty(pStab(iFig)) & ~isequal(pStab(iFig), matlab.graphics.GraphicsPlaceholder)
        pTradeoff = [pTradeoff pStab(iFig)]; %#ok<AGROW>
        strTradeoff{end + 1} = 'Stable system'; %#ok<AGROW>
    end

    if ~isempty(pRob(iFig)) & ~isequal(pRob(iFig), matlab.graphics.GraphicsPlaceholder)
        pTradeoff = [pTradeoff pRob(iFig)]; %#ok<AGROW>
        strTradeoff{end + 1} = 'Robustness contour lines ($M_\mathrm{st}$)'; %#ok<AGROW>
    end

    if ~isempty(pPerf(iFig)) & ~isequal(pPerf(iFig), matlab.graphics.GraphicsPlaceholder)
        pTradeoff = [pTradeoff pPerf(iFig)]; %#ok<AGROW>
        strTradeoff{end + 1} = 'Performance contour lines ($IAE$)'; %#ok<AGROW>
    end

    if ~isempty(pPfDr(iFig)) & ~isequal(pPfDr(iFig), matlab.graphics.GraphicsPlaceholder)
        pTradeoff = [pTradeoff pPfDr(iFig)]; %#ok<AGROW>
        strTradeoff{end + 1} = 'Pareto front (disturbance rejection)'; %#ok<AGROW>
    end

    if ~isempty(pPfSt(iFig)) & ~isequal(pPfSt(iFig), matlab.graphics.GraphicsPlaceholder)
        pTradeoff = [pTradeoff pPfSt(iFig)]; %#ok<AGROW>
        strTradeoff{end + 1} = 'Pareto front (setpoint tracking)'; %#ok<AGROW>
    end

    legend(pTradeoff, strTradeoff, ...
        'NumColumns', 1, ...
        'Interpreter', 'latex', ...
        'Location', 'northoutside')
    ax(iFig).Layer = 'top';
    ax(iFig).Box = 'on';
end

end