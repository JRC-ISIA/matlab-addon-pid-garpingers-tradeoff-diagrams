function [paretoFront, foundOptimum] = extractParetoFront(robMat, perfMat, kx, ky, opts)
%EXTRACTPARETOFRONT 
%
%  Assumes size(robMat) == [numel(ky), numel(kx)].
%  Assumes size(perfMat) == [numel(ky), numel(kx)].
%
%   Inputs:
%     robMat - Robustness matrix (Mst values)
%     perfMat - Performance matrix (IAE values)
%     kx - Controller gains on the x-axis
%     ky - Controller gains on the y-axis
%
%   Name-Value Pairs:
%       'SmoothMethod' - Smoothing method for pareto front,
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
%       'SmoothWindow' - Window size, specified as a positive
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
%     paretoFront - Extracted pareto front
%     foundOptimum - Indicates if the optimum has been reached on the
%       Pareto front 
%
% Notes:
%   - Created with support from Microsoft Copilot (GPT-5)


arguments
    robMat (:,:) {mustBeNonempty, mustBeNumeric}
    perfMat (:,:) {mustBeNonempty, mustBeNumeric}
    kx (:,1) {mustBeNonempty, mustBeVector}
    ky (:,1) {mustBeNonempty, mustBeVector}

    opts.SmoothMethod = "movmean"
    opts.SmoothWindow double = 5    
end

% Sanity check
[nr, nc] = size(robMat);
if nr ~= numel(ky) || nc ~= numel(kx)
    error("Size mismatch: size(robMat) = [%d %d], but numel(ky)=%d, numel(kx)=%d.", ...
          nr, nc, numel(ky), numel(kx));
end

[nr, nc] = size(perfMat);
if nr ~= numel(ky) || nc ~= numel(kx)
    error("Size mismatch: size(perfMat) = [%d %d], but numel(ky)=%d, numel(kx)=%d.", ...
          nr, nc, numel(ky), numel(kx));
end

% Find maximum robustness level
szRobMat = size(robMat);
[~, minPerfIndex] = min(perfMat, [], 'all', 'omitnan');
maxRob = robMat(minPerfIndex);
if ~isfinite(maxRob)
    maxRob = max(robMat,[],"all", "omitnan");
end

robLevels = 1:0.01:maxRob;

% Calculate pareto front
paretoFront = [];
foundOptimum = true;
for i = 1:numel(robLevels)
    [rowIdx, colIdx] = ...
        argminPerfAtRob(robMat, perfMat, robLevels(i));
    if isequal(rowIdx, szRobMat(1)) || isequal(colIdx, szRobMat(2))
        foundOptimum = false;
        break
    end
    paretoFront = [paretoFront, [kx(colIdx); ky(rowIdx)]]; %#ok<AGROW>
end

if foundOptimum
    [minPerfRowIdx, minPerfColIdx] = ind2sub(szRobMat, minPerfIndex);
    paretoFront = [paretoFront, [kx(minPerfColIdx); ky(minPerfRowIdx)]];
end

% Smooth pareto front
paretoFront = unique(paretoFront.', 'rows', 'stable');
paretoFront = paretoFront';

switch lower(char(opts.SmoothMethod))
    case 'off'
    case 'auto'
        paretoFront = smoothdata(paretoFront, 2);
        if foundOptimum
            [minPerfRowIdx, minPerfColIdx] = ind2sub(szRobMat, minPerfIndex);
            paretoFront = [paretoFront, [kx(minPerfColIdx); ky(minPerfRowIdx)]];
        end
    otherwise
        paretoFront = smoothdata(paretoFront, 2, opts.SmoothMethod, opts.SmoothWindow);
        if foundOptimum
            [minPerfRowIdx, minPerfColIdx] = ind2sub(szRobMat, minPerfIndex);
            paretoFront = [paretoFront, [kx(minPerfColIdx); ky(minPerfRowIdx)]];
        end
end

end