load 'heartScale.mat'

C = 5
eta = 0.8

dataSize = size(X,1);
dim = size(X,2);

trSize = 60;

trX = zeros(trSize, dim);
trY = zeros(trSize, 1);

for i=1:1:trSize
    index = uint32(1+(dataSize-1)*rand());
    trX(i,:) = X(index,:);
    trY(i,:) = Y(index, 1);
end

tsSize = 200;

tsX = zeros(tsSize, dim);
tsY = zeros(tsSize, 1);

for i=1:1:tsSize
    index = uint32(1+(dataSize-1)*rand());
    tsX(i,:) = X(index,:);
    tsY(i,:) = Y(index, 1);
end

v = rand(dim+1,1);
alpha = rand(dim+1,1);
lamda = rand(trSize,1);
%v = zeros(dim+1,1);
%alpha = zeros(dim+1,1);
%lamda = zeros(n,1);

nv = rand(dim+1, 3);

[lamda,v] = dsvc(trX,trY,C,lamda,v,alpha, nv, eta);

alpha = updateAlpha(alpha, v, nv, eta);

predictedY = dsvcoutput(tsX, v);

cnt = 0;
for i=1:1:tsSize
    if predictedY(1,i) == tsY(i,1)
        cnt = cnt+1;
    end
end

rate = cnt/ tsSize;



