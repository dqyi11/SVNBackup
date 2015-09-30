load 'heartScale.mat'

C = 5;
eta = 0.8;

dataSize = size(X,1);
dim = size(X,2);

trSize = 20;
agentNum = 20;
connectivity = 0.8;
ker = 'rbf';

zeroVec = zeros(1,dim);

%init concensus vector
L = 8;
concensusV = zeros(L, dim);
G = zeros(L, dim);
for j=1:1:L
    index = uint32(1+(size(X,1)-1)*rand());
    tempX = X(index,:);
    concensusV(j,:) = tempX;
    tempX = GFunc(tempX);
    G(j,:) = tempX;
end

%init agent position
agentSV = cell(agentNum, 1);
agentSVY = cell(agentNum, 1);
agentSVIdx = zeros(agentNum, trSize);
agentV = zeros(dim+1,agentNum);

for i = 1:1:agentNum
    trX = zeros(trSize, dim);
    trY = zeros(trSize, 1);
    for j = 1:1:trSize
        index = uint32(1+(size(X,1)-1)*rand());
        agentSVIdx(i,j) = index;
        trX(j,:) = X(index,:);
        trY(j,1) = Y(index,1);
    end
    agentSV{i,1} = trX;
    agentSVY{i,1} = trY;
end

%init adjacency
adjacency = eye(agentNum);
adjacencyCnt = zeros(agentNum,1);
for i=1:1:agentNum
    for j=1:1:agentNum
        if (i~=j)
            if(rand() > connectivity)
                adjacency(i,j) = 1;
                adjacencyCnt(i,1) = adjacencyCnt(i,1) + 1;
            end
        end
    end
end

adjacentList = cell(agentNum,1);
for i=1:1:agentNum
    adjacentIdx = [];
    for j=1:1:agentNum
        if (i~=j)
            if(adjacency(i,j) == 1)
                adjacentIdx = [adjacentIdx ; j];
            end
        end
    end
    adjacentList{i,1} = adjacentIdx;
end

%generate test data
tsSize = dataSize;
tsX = zeros(tsSize,dim);
tsY = zeros(tsSize,1);
for j=1:1:tsSize
    index = uint32(1+(size(X,1)-1)*rand());
    tsX(j,:) = X(index,:);
    tsY(j,1) = Y(index,1);
end

lamda = rand(trSize,agentNum);
b = rand(1, agentNum);
w = rand(L, agentNum);
alpha = zeros(L,agentNum);
beta = zeros(1, agentNum);

paramA = zeros(trSize, agentNum);
paramC = zeros(L, agentNum);
paramB = zeros(1, agentNum);


predictedY = zeros(agentNum, tsSize);

iteration = 10;
rate = zeros(agentNum, iteration);
for l = 1:1:iteration

    l
    
    for i=1:1:agentNum
        neighborW = [];
        neighborB = [];
        neighborIdx = adjacentList{i,1};
        for j=1:1:adjacencyCnt(i,1)
            neighborW = [neighborW w(:,neighborIdx(j))];
            neighborB = [neighborB b(:,neighborIdx(j))];
        end
        trX = agentSV{i,1};
        trY = agentSVY{i,1};
        [lamda(:,i), w(:,i) b(1,i)] = dnsvc(trX, trY, concensusV, agentNum * C ,lamda(:,i),w(:,i),b(:,1),alpha(:,i), beta(:,i), neighborW, neighborB, eta, ker);

    end
    
    for i=1:1:agentNum
        neighborW = [];
        neighborB = [];
        neighborIdx = adjacentList{i,1};
        for j=1:1:adjacencyCnt(i,1)
            neighborW = [neighborW w(:,neighborIdx(j))];
            neighborB = [neighborB b(:,neighborIdx(j))];
        end
        [alpha(:,i) beta(:,i)] = updateAlphaBeta(alpha(:,i), beta(:,i), w(:,i), b(:,i), neighborW, neighborB, eta);
    end
    
    for i=1:1:agentNum
        neighborW = [];
        neighborB = [];
        neighborIdx = adjacentList{i,1};
        for j=1:1:adjacencyCnt(i,1)
            neighborW = [neighborW w(:,neighborIdx(j))];
            neighborB = [neighborB b(:,neighborIdx(j))];
        end
        [paramA(:,i) paramB(:,i) paramC(:,i)] = UpdateParam(trX, trY, concensusV, w(:,i), b(:,i), alpha(:,i), beta(:,i), lamda(:,i), neighborW, neighborB, eta, ker);
    end

    for i=1:1:agentNum
        trX = agentSV{i,1};
        predictedY(i,:) = dnsvcoutput(tsX, trX, concensusV,  paramA(:,i), paramB(:,i), paramC(:,i), ker);
    end

    cnt = zeros(agentNum, 1);
    
    for i=1:1:agentNum
        for j=1:1:tsSize
            if predictedY(i,j) == tsY(j,1)
                cnt(i,1) = cnt(i,1) + 1;
            end     
        end
        rate(i,l) = cnt(i,1)/ tsSize;
    end
end