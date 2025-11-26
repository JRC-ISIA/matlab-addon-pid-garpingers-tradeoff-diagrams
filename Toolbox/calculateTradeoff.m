function data = calculateTradeoff(plant, kp, ki, kd, opts)
%CALCULATETRADEOFF Calculates design criteria for Garpinger's trade-off
%diagram 
%
%   Description:
%   This function computes the necessary design criteria to generate a 
%   Garpinger trade-off diagram for various PID controllers. Based on the 
%   characteristics of the plant, the function determines whether the 
%   system is proportional or integral, with or without time delay.
%
%   For proportional plants, the function interpolates over the integral 
%   gain (ki) via proportional gain (kp) starting from zero up to the 
%   specified maximum, using the resolution defined by ResolutionSpec. 
%   The derivative gain (kd) is kept constant.
%
%   For integral plants, the function interpolates over the proportional 
%   gain (kp) via the derivative gain (kd) starting from zero up to the 
%   specified maximum, while keeping the integral gain (ki) constant.
%
%   Inputs:
%       plant - Plant
%       kp - Maximum proportional gain
%       ki - Maximum integral gain
%       kd - Maximum derivative gain
%
%   Name-Value Pairs:
%       ResolutionSpec - Either a numeric vector [nY nX] specifying the
%       resolution, or a string: 'low', 'medium', 'high'.
%
%   Outputs:
%       data - Calculated design criteria (returned as struct)
%       data.kp - Proportional controller gains 
%       data.ki - Integral controller gains
%       data.kd - Derivative controller gains
%       data.stabMat - Stability matrix
%       data.robMat - Robustness matrix (Mst values)
%       data.perfMatDistRej - Performance matrix for disturbance rejection 
%       (Integrated absolute error for a disturbance unit step)
%       data.perfMatSetTrack - Performance matrix for setpoint tracking 
%       (Integrated absolute error for a setpoint unit step)
%
% Example:
%   plant = tf(1, [1 2 1], 'IoDelay', 0.5);  % User-defined plant model
%   dataComplete = calculateTradeoff(plant, 5, 2.5, 0, 'ResolutionSpec', 'low');
%   plotTradeoff(dataComplete)
%   dataRelevant = calculateTradeoff(plant, 1.5, 0.6, 0, 'ResolutionSpec', 'medium');
%   plotTradeoff(dataRelevant)
%
% Notes:
%   - The resolution of the matrices affects computation time and detail.
%   - Created with support from Microsoft Copilot (GPT-5)

%% Name-Value definition & validation

arguments
    plant (1,1) ss {mustBeNonempty}
    kp (1,1) double {mustBeFinite, mustBeNonempty}
    ki (1,1) double {mustBeFinite, mustBeNonempty}
    kd (1,1) double {mustBeFinite, mustBeNonempty}
    % Name-Value pairs:
    opts.ResolutionSpec = 'medium'
end

warning('off')

%% Read out resolution specification
if isnumeric(opts.ResolutionSpec)
    if numel(opts.ResolutionSpec) ~= 2
        error("calculateTradeoff:InvalidNumResDim", ...
            "Numeric resolution specification must be a vector of size [nY nX].");
    end
    nY = opts.ResolutionSpec(1);
    nX = opts.ResolutionSpec(2);
elseif ischar(opts.ResolutionSpec) || isstring(opts.ResolutionSpec)
    switch lower(char(opts.ResolutionSpec))
        case 'low'
            nX = 25; nY = 25;
        case 'medium'
            nX = 50; nY = 50;
        case 'high'
            nX = 100; nY = 100;
        otherwise
            error("calculateTradeoff:InvalidResStr", ...
                "Unknown resolution specification: " + opts.ResolutionSpec);
    end
else        
    error("calculateTradeoff:InvalidResType", ...
        "Invalid input type for resolutionSpec.");
end

%% Check model data

sys = minreal(plant);

if ~isproper(sys)
    error("calculateTradeoff:NoProperPlant", ...
        "Plant has to be be a proper system. Check plant model and try again!");
end

if ~isscalar(sys)
    error("calculateTradeoff:NoSisoSystem", ...
        "Plant has to be a SISO system. Check plant model and try again")
end


%% Detect plant type (proportional, integral)

sysDeg = length(find(abs(pole(sys)) < 10^-4));

switch sysDeg
    case 0
        disp("Detected proportional plant. Garpingers trade-off " + ...
            "plots are calculated for ki via kp (" + num2str(nY) + " x " + ...
                num2str(nX) + ").")
        data.kp = linspace(0, kp, nX);
        data.ki = linspace(0, ki, nY);
        data.kd = kd;
    case 1
        disp("Detected integral plant. Garpingers trade-off plots ", ...
            "are calculated for kp via kd (" + num2str(nY) + " x " + ...
            num2str(nX) + ").")
        data.kp = linspace(0, kp, nY);
        data.ki = ki;
        data.kd = linspace(0, kd, nX);
    otherwise
        error("calculateTradeoff:UnsupportedPantType", ... 
            "Unsupported plant type detected. Grapingers trade-off " + ...
            "plots can only be calculated for proportional and integral " + ...
            "plants without and with IO delay.")
end

P = sys;


%% Initialize stabMat, robMat & perfMatDistRej matrices

data.stabMat = false(nY, nX);
data.robMat = NaN(nY, nX);
data.perfMatDistRej = NaN(nY, nX);
data.perfMatSetTrack = NaN(nY, nX);


%% Calculate stability, robMat & IAE matrices

tic
disp("Start calculation of the design criteria for Garpingers trade-off plots.")

s = tf('s');
tStart = cputime;

for ilX = 1:nX
    
    for ilY = 1:nY

        switch sysDeg
            case 0
                C = data.kp(ilX) + data.ki(ilY) / s + data.kd * s;
            case 1
                C = data.kp(ilY) + data.ki / s + data.kd(ilX) * s;
            otherwise
                error("calculateTradeoff:UnsupportedPantType", ... 
                    "Unsupported plant type detected. Grapingers trade-off " + ...
                    "plots can only be calculated for proportional and integral " + ...
                    "plants without and with IO delay.")
        end
        
        % Compute transfer functions 
        S = (1 / (1 + C * P));
        T = (P * C / (1 + C * P));      
        Ger = pade(S);
        Ged = pade(-P / (1 + C * P));

        % Determine if closed loop models are stable
        data.stabMat(ilY, ilX) = isstable(Ger);

        % Calculate Mst & IAE values only for stable controllers
        if data.stabMat(ilY, ilX)
            
            % Robustness (Ms, Mt, robMat)      
            [~, Ms] = evalc('computeM(S)');
            [~, Mt] = evalc('computeM(T)');         
            data.robMat(ilY, ilX) = max(Ms, Mt, "omitnan");

            % IAE for setpoint tracking and disturbance rejection
            data.perfMatDistRej(ilY, ilX) = computeIae(Ged);
            data.perfMatSetTrack(ilY, ilX) = computeIae(Ger);

        end
    end
    
    sec = seconds((cputime - tStart) * (nX / ilX - 1));
    sec.Format = 'hh:mm:ss';
    disp(string(round(ilX / nX * 100, 2)) + " % (Estimated remaining time: " + string(sec) + "; Date: " + string(datetime) + ")")

end

toc
disp("Finished calculation of the design criteria for Garpingers trade-off plots.")
warning('on')

end