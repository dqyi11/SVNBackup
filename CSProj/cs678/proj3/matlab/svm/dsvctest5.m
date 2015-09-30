load 'heartScale.mat'

C = 5;
eta = 0.8;

dataSize = size(X,1);
dim = size(X,2);

trSize = 60;
agentNum = 50;
connectivity = 0.8;

%init agent position
agentSV = cell(agentNum, 1);
agentSVY = cell(agentNum, 1)
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



alpha = rand(dim+1,agentNum);
lamda = rand(trSize,agentNum);

predictedY = zeros(agentNum, tsSize);

iteration = 500;
rate = zeros(agentNum, iteration);
for l = 1:1:iteration

    l
    
    for i=1:1:agentNum
        neighborV = [];
        neighorIdx = adjacentList{i,1};
        for j=1:1:adjacencyCnt(i,1)
            neighborV = [neighborV agentV(:,neighorIdx(j))];
        end
        trX = agentSV{i,1};
        trY = agentSVY{i,1};
        [lamda(:,i), agentV(:,i)] = dsvc(trX, trY, agentNum * C ,lamda(:,i),agentV(:,i),alpha(:,i), neighborV, eta);

    end
    
    for i=1:1:agentNum
        neighborV = [];
        neighorIdx = adjacentList{i,1};
        for j=1:1:adjacencyCnt(i,1)
            neighborV = [neighborV agentV(:,neighorIdx(j))];
        end
        alpha(:,i) = updateAlpha(alpha(:,i), agentV(:,i), neighborV, eta);
    end

    for i=1:1:agentNum
        predictedY(i,:) = dsvcoutput(tsX, agentV(:,i));
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

