function [iae, ie] = computeIae(G)
% COMPUTEIAE Computes the Integrated Absolute Error (IAE) and Integrated error (IE) of a dynamic system.
%
% This function simulates the response of a dynamic system G to a unit step
% input and calculates the IAE, defined as the integral of the absolute error
% over time.
%
% INPUT:
%   G - A dynamic system object (e.g., created using tf or ss).
%
% OUTPUT:
%   iae - The Integrated Absolute Error (IAE) value.
%   ie - The Integrated Error (IE) value.

iae = NaN;
ie = NaN;

if ~isproper(G)
    return
end

Gminreal = minreal(G);

if ~isstable(Gminreal)
    return
end

tolerance = 0.001;
S = stepinfo(G, 'SettlingTimeThreshold', tolerance);

if S.SettlingTime <= 0
    return
end

if isfinite(S.SettlingTime)
    [y, t] = step(G, linspace(0, S.SettlingTime * 11, 10000));
else
    [~, t] = step(G);
    [y, t] = step(G, linspace(0, t(end) * 10, 10000));
end

iae = trapz(t, abs(y));
ie = trapz(t, y);

end
