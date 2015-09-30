load 'heartScale.mat'

C = 5

ker = 'rbf'

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

[nsv, alpha, b0] = svc(trX,trY,ker,C);

predictedY = svcoutput(trX,trY,tsX, ker,alpha,b0);

%successCnt = find(predictedY == Y);

cnt = 0;
for i=1:1:tsSize
    if predictedY(i,1) == tsY(i,1)
        cnt = cnt + 1;
    end
end


rate = cnt / tsSize;
%rate = size(successCnt)/size(predictedY);

