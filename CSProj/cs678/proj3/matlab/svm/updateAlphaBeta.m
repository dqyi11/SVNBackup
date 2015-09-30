function [alpha beta] = updateAlphaBeta(alpha, beta, w, b, nw, nb, eta)

B = size(nw,2);

for i=1:1:B
    alpha = alpha + (eta/2) * (w - nw(:,i));
    beta = beta + (eta/2) * (b - nb(:,i));
end

end