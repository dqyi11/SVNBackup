load 'heartScale.mat'

C = 5
eta = 0.8

dataSize = size(X,1);
dim = size(X,2);

trSize = 20;
tr1 = 1;

trX1 = X(tr1:tr1+trSize-1,:);
trY1 = Y(tr1:tr1+trSize-1,:);

tr2 = 60;

trX2 = X(tr2:tr2+trSize-1,:);
trY2 = Y(tr2:tr2+trSize-1,:);

tsSize = dataSize;
ts = 1;

tsX = X(ts:ts+tsSize-1,:);
tsY = Y(ts:ts+tsSize-1,:);

v = rand(dim+1,2);
alpha = rand(dim+1,2);
lamda = rand(trSize,2);
%v = zeros(dim+1,1);
%alpha = zeros(dim+1,1);
%lamda = zeros(n,1);

%nv = rand(dim+1, 3);

rate1 = []
rate2 = []
for l = 1:1:10000

    [lamda(:,1),v(:,1)] = dsvc(trX1,trY1,C,lamda(:,1),v(:,1),alpha(:,1), v(:,2), eta);
    [lamda(:,2),v(:,2)] = dsvc(trX2,trY2,C,lamda(:,2),v(:,2),alpha(:,2), v(:,1), eta);

    alpha(:,1) = updateAlpha(alpha(:,1), v(:,1), v(:,2), eta);
    alpha(:,2) = updateAlpha(alpha(:,2), v(:,2), v(:,1), eta);

    predictedY1 = dsvcoutput(tsX, v(:,1));
    predicateY2 = dsvcoutput(tsX, v(:,2));

    cnt1 = 0;
    cnt2 = 0;
    for i=1:1:tsSize
        if predictedY1(1,i) == tsY(i,1)
            cnt1 = cnt1 + 1;
        end
        
        if predicateY2(1,i) == tsY(i,1)
            cnt2 = cnt2 + 1;
        end
    end
    
    rate1 = [rate1, cnt1/ tsSize];
    rate2 = [rate2, cnt2/ tsSize];

end