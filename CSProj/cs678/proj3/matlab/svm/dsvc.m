function [lamda,v] = dsvc(X,Y,C,lamda,v,alpha, nv, eta)

%fprintf('Distributed Linear Support Vector Classification\n');
%fprintf('________________________________________________\n');

n = size(X,1);
dim = size(X,2);

B = size(nv,2);

f = 2 * alpha;
for i=1:1:B
    f = f - eta * (nv(:,i)+v);
end

PI = zeros(dim+1, dim+1);
PI(dim+1,dim+1) = 1;
U = (1 + 2 * eta * B)* eye(dim+1) - PI;

tX = [X , ones(n, 1)];
tY = eye(n);
for i=1:1:n
    tY(i,i) = Y(i,1);
end

H = tY * tX * inv(U) * tX' * tY;

c = -(ones(n,1)+tY*tX*inv(U)*f);

vlb = zeros(n,1);
vub = B * C * ones(n,1);

x0 = zeros(n,1);

%fprintf('Optimising ...\n');

A = [];
b = [];
neqcstr = 0;

[lamda min how] = qp(H, c, A, b, vlb, vub, x0, neqcstr);

%fprintf(how);
%fprintf('\n');

v = inv(U) * (tX' * tY * lamda - f);

end












