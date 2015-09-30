load 'house.mat';

X = House(:,1:13);
Y = House(:,14);

lamda = 0.2;
ker = 'rbf';

noiseVar = 0.8;

%add noise
X1 = X + noiseVar * randn(n,13);
Y1 = Y + noiseVar * randn(n,1);


C = KernelReg(X1, Y1, lamda, ker);

predY = KernelEst(C, ker, X1, X);

error = (Y - predY);

n = size(Y,1);
rt = zeros(n,1);
for i=1:1:n
    rt(i,1) = error(i,1)/Y(i,1);
end

rate = sum(rt) / n;

    
