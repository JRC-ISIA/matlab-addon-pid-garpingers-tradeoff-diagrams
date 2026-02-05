function data = calculateCascadedTradeoff(Pi, Po, Ci, kp, ki, kd, opts)
%CALCULATECASCADEDTRADEOFF Calculates design criteria for Garpinger's
%trade-off plot a cascaded control loop
%
%   Description:
%   This function computes the necessary design criteria to generate a 
%   Garpinger trade-off diagram for various PID controllers for a cascaded
%   closed loop control.  Based on the characteristics of the inner loop
%   plant (Pi), the inner loop controller (Ci) and the outer loop plant
%   (Po) the function determines whether the system is proportional or
%   integral, with or without time delay.
%
%   The calculation of the plant for the design criteria is based on
%   P = Pi*Ci/(1+Pi*Ci)*Po.
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
%       Pi - Inner loop plant
%       Po - Outer loop plant
%       Ci - Inner loop controller
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
%       data.robMat - Maximum robustness matrix (data.robMat values)
%       data.perfMatDistRej - Performance matrix for disturbance rejection 
%       (Integrated absolute error for a disturbance unit step)
%       data.perfMatSetTrack - Performance matrix for setpoint tracking 
%       (Integrated absolute error for a setpoint unit step)
%       data.signedPerfMatDistRej - Signed performance matrix for disturbance rejection 
%       (Integrated error for a disturbance unit step)
%       data.signedPerfMatSetTrack - Signed performance matrix for setpoint tracking 
%       (Integrated error for a setpoint unit step)
%
% Example:
%   Pi = tf(1, [1 2 1], 'IoDelay', 0.5);  % User-defined inner loop plant model
%   s = tf("s");
%   Ci = 0.8 + 0.5/s; % User-defined inner loop controller
%   Po = 1/s; % User-defined outer loop plant model
%   data = calculateCascadedTradeoff(Pi, Po, Ci, 2, 0, 2);
%   plotTradeoff(data)
%
% Notes:
%   - The resolution of the matrices affects computation time and detail.
%   - Created with support from Microsoft Copilot (GPT-5)


%% Name-Value definition & validation

arguments
    Pi (1,1) ss {mustBeNonempty}
    Po (1,1) ss {mustBeNonempty}
    Ci (1,1) ss {mustBeNonempty}
    kp (1,1) double {mustBeFinite, mustBeNonempty}
    ki (1,1) double {mustBeFinite, mustBeNonempty}
    kd (1,1) double {mustBeFinite, mustBeNonempty}
    % Name-Value pairs:
    opts.ResolutionSpec = 'medium'
    opts.Tf {mustBeGreaterThanOrEqual(opts.Tf, 0.0)} = 0.0
end

warning('off')

%% Read out resolution specification
if isnumeric(opts.ResolutionSpec)
    if numel(opts.ResolutionSpec) ~= 2
        error("garpingerCalc:InvalidNumResDim", ...
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
            error("calculateCascadedTradeoff:InvalidResStr", ...
                "Unknown resolution specification: " + opts.ResolutionSpec);
    end
else        
    error("calculateCascadedTradeoff:InvalidResType", ...
        "Invalid input type for 'ResolutionSpec'.");
end


%% Check model data

sysPi = minreal(tf(Pi));
if ~isproper(sysPi)
    error("calculateCascadedTradeoff:NoProperPlantPi", ...
        "Inner loop plant has to be a proper system. Check inner loop plant and try again!");
end

if ~isscalar(sysPi)
    error("calculateCascadedTradeoff:NoSisoSystemPi", ...
        "Inner loop plant has to be a SISO system. Check outer loop plant and try again")
end

sysPo = minreal(tf(Po));
if ~isproper(sysPo)
    error("calculateCascadedTradeoff:NoProperPlantPo", ...
        "Outer loop plant has to be a proper system. Check outer loop plant and try again!");
end

if ~isscalar(sysPo)
    error("calculateCascadedTradeoff:NoSisoSystemPo", ...
        "Outer loop plant has to be a SISO system. Check outer loop plant and try again")
end

Pi = sysPi;
Po = sysPo;
Pil = Pi*Ci/(1+Pi*Ci);
P = Pil*Po; %#ok<NASGU>


%% Detect plant type (proportional, integral)

[~, sys] = evalc('minreal(ss(P))');
sysDeg = length(find(abs(pole(sys)) < 10^-4));

switch sysDeg
    case 0
        disp("Detected proportional plant Pi*Ci/(1+Pi*Ci)*Po. " + ...
            "Garpingers trade-off plots are calculated for ki via kp (" + ...
            num2str(nY) + " x " + num2str(nX) + ").")
        data.kp = linspace(0, kp, nX);
        data.ki = linspace(0, ki, nY);
        data.kd = kd;
    case 1
        disp("Detected integral plant Pi*Ci/(1+Pi*Ci)*Po. Garpingers " + ...
            "trade-off plots are calculated for kp via kd (" + ...
            num2str(nY) + " x " + num2str(nX) + ").")
        data.kp = linspace(0, kp, nY);
        data.ki = ki;
        data.kd = linspace(0, kd, nX);
    otherwise
        error("calculateCascadedTradeoff:UnsupportedPantType", ...
            "Unsupported plant type detected. Grapingers " + ... 
            "trade-off plots can only be calculated for proportional " + ...
            "and integral plants with IO delay.")
end

P = sys;


%% Initialize data.stabMat, data.robMat & data.perfMatDistRej matrices

data.stabMat = zeros(nY, nX);
data.robMat = NaN(nY, nX);
data.perfMatDistRej = Inf(nY, nX);
data.perfMatSetTrack = Inf(nY, nX);
data.signedPerfMatDistRej = NaN(nY, nX);
data.signedPerfMatSetTrack = NaN(nY, nX);


%% Calculate stability, data.robMat & data.perfMatDistRej matrices

tic
disp("Start calculation of the design criteria for Garpingers trade-off plots.")

s = tf('s');
tStart = cputime;

for ilX = 1:nX

    for ilY = 1:nY
        
        switch sysDeg
            case 0
                Co = data.kp(ilX) + data.ki(ilY) / s + data.kd * s / (1 * opts.Tf + 1);
            case 1
                Co = data.kp(ilY) + data.ki / s + data.kd(ilX) * s / (1 * opts.Tf + 1);
            otherwise
                error("calculateCascadedTradeoff:UnsupportedPantType", ... 
                    "Unsupported plant type detected. Grapingers trade-off " + ...
                    "plots can only be calculated for proportional and integral " + ...
                    "plants without and with IO delay.")
        end
        
        % Compute transfer functions 
        S = (1 / (1 + Co * P));
        T = (P * Co / (1 + Co * P)); %#ok<NASGU>

        Ger = pade(S);
        Pid = Pi / (1 + Pi * Ci);
        Ged = pade(-(Pid * Po / (1 + Co * Ci * Pid * Po)));

        % Determine if closed loop models are stable
        data.stabMat(ilY, ilX) = isstable(Ger);

        % Calculate data.robMat & data.perfMatDistRej values only for stable controllers
        if data.stabMat(ilY, ilX) 
            
            % Robustness (Ms, Mt, robMat)      
            [~, Ms] = evalc('computeM(S)');
            [~, Mt] = evalc('computeM(T)');         
            data.robMat(ilY, ilX) = max(Ms, Mt);

            if data.robMat(ilY, ilX) <= 1
                data.robMat(ilY, ilX) = NaN;
            end

            % IAE for setpoint tracking and disturbance rejection
            [~, data.perfMatDistRej(ilY, ilX), data.signedPerfMatDistRej(ilY, ilX)] = ...
                evalc('computeIae(Ged)');
            [~, data.perfMatSetTrack(ilY, ilX), data.signedPerfMatSetTrack(ilY, ilX)] = evalc('computeIae(Ger)');
        end
        
    end
        
    sec = seconds((cputime - tStart) * (nX / ilX - 1));
    sec.Format = 'hh:mm:ss';
    disp(string(round(ilX / nX * 100, 2)) + ...
        " % (Estimated remaining time: " + string(sec) + "; Date: " + ...
        string(datetime) + ")")

end

toc
disp("Finished calculation of the design criteria for Garpingers trade-off plots.")
warning('on')

end