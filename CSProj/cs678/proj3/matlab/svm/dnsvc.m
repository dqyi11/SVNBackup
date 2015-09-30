function [lamda,w, b] = dnsvc(X,Y,concensusV, C,lamda,w, b,alpha, beta, nw, nb, eta, ker)
%fprintf('Distributed Nonlinear Support Vector Classification\n');
%fprintf('________________________________________________\n');

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

K1 = svkernel2(ker, tX, tX);
Kcap = 2 * eta * B * svkernel2(ker, tX, concensusV) * inv(U) * svkernel2(ker, concensusV, tX);

H = tY * (K1 - Kcap + ones(n,n)/(2* eta * B) ) * tY;

K2 = svkernel2(ker, concensusV, tX);
Kcap2 = 2 * eta * B * svkernel2(ker, concensusV, concensusV) * inv(U) * svkernel2(ker, concensusV, tX);
c = - ones(1,n)+ (f' * inv(U) * K2 + h * ones(1,n)/(2 * eta * B))* tY ;
c = c';

vlb = zeros(n,1);
vub = C * ones(n,1);

x0 = zeros(n,1);

%fprintf('Optimising ...\n');

A = [];
b = [];
neqcstr = 0;

[lamda min how] = qp(H, c, A, b, vlb, vub, x0, neqcstr);

%fprintf(how);
%fprintf('\n');

K3 = svkernel2(ker, concensusV, concensusV);
%Kcap3 = 2 * eta * B * K3 * inv(U) * K3;
deltaK2 = inv(U) * K2;%K2 - Kcap2;
w1 = deltaK2 * tY * lamda;
deltaK3 = K3 * inv(U);
w2 = deltaK3 * f;
w = w1 - w2;
b = (ones(1,n) * tY  * lamda - h)/(2 * eta * B);

end
