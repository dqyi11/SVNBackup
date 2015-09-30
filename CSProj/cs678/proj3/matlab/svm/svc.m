function [nsv, alpha, b0] = svc(X,Y,ker,C)

fprintf('Support Vector Classification\n');
fprintf('_____________________________\n');

n = size(X,1);

H = zeros(n,n);
for i=1:1:n
    for j=1:1:n
        H(i,j) = Y(i)*Y(j)*svkernel(ker, X(i,:),X(j,:));
    end
end

c = -ones(n,1);

vlb = zeros(n,1);
vub = C * ones(n,1);

x0 = zeros(n,1);

if strcmp(lower(ker), 'linear')
    A = Y';
    b = 0;
    neqcstr = 1;
else
    A = [];
    b = [];
    neqcstr = 0;
end

fprintf('Optimising ...\n');

[alpha lambda how] = qp(H, c, A, b, vlb, vub, x0, neqcstr);

fprintf(how);
fprintf('\n');
%w2 = alpha' * H * alpha;

nsv = length(Y);

b0 = 0;

if strcmp(lower(ker), 'linear')
    svi = find( alpha > 0);
    svii = find(alpha > 0 & alpha < C);
    b0 =  (1/length(svii))*sum(Y(svii) - H(svii,svi)*alpha(svi).*Y(svii));
end







