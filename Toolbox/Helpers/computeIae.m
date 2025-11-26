function iae = computeIae(G)
% COMPUTEIAE Computes the Integrated Absolute Error (IAE) of a dynamic system.
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

iae = NaN;

if ~isproper(G)
    return
end

[y, t] = impulse(G);
tEnd = t(end);


if ~all(y == 0)
    while abs(y(end)) > 10^-12
        tEnd = tEnd * 10;
        y = impulse(G, tEnd);
    end
    [e, t] = step(G, tEnd);
    if abs(e(end)) <= 10^-10
        iae = trapz(t, abs(e));
    else
        iae = Inf;
    end
end

end
