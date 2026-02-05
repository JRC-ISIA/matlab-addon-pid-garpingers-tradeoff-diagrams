function M = computeM(G)
% COMPUTEIAE Computes the robustness value M of a dynamic system.
%
% INPUT:
%   G - A dynamic system object (e.g., created using tf or ss).
%
% OUTPUT:
%   M - Robustness value M.

M = NaN;

Gpade = pade(G);

if ~isproper(Gpade)
    return
end

Gminreal = minreal(Gpade);

if ~isstable(Gminreal)
    return
end

H = freqresp(Gminreal);
M = max(abs(H));
if (M <= 1) || ~isfinite(M)
    M = NaN;
end

end
