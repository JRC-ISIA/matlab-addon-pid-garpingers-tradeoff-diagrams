function M = computeM(G)
% COMPUTEIAE Computes the robustness value M of a dynamic system.
%
% INPUT:
%   G - A dynamic system object (e.g., created using tf or ss).
%
% OUTPUT:
%   M - Robustness value M.

Gpade = pade(G);
if isproper(Gpade)
    Gapprox = minreal(Gpade);
else
    Gapprox = Gpade;
end
H = freqresp(Gapprox);
M = max(abs(H));
if (M <= 1) || ~isfinite(M)
    M = NaN;
end

end
