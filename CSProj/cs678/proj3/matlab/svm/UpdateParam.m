function [paramA paramB paramC] = UpdateParam(X, Y, concensusV, w, b, alpha, beta, lamda, nw, nb, eta, ker)

n = size(X,1);
dim = size(X,2);
L = size(concensusV,1);

B = size(nw,2);

f = 2 * alpha;
for i=1:1:B
    f = f - eta * (nw(:,i)+w);
end

h = 2 * beta;
for i=1:1:B
    h = h - eta * (nb(:,i)+b);
end

U = ones(L,L) + 2 * eta * B * svkernel2(ker, concensusV, concensusV);

tX = X;
tY = zeros(n,n);
for i=1:1:n
    tY(i,i) = Y(i,1);
end

paramA = tY * lamda;
paramC = - inv(U) * f - (2 * eta * B) * inv(U) * svkernel2(ker, concensusV, tX) * tY * lamda;
paramB = (ones(1,n) * tY * lamda - h) / (2 * eta * B);

end