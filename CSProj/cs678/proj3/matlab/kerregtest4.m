load 'house.mat'
X = House(:,1:13);
Y = House(:,14);
lamda = 0.1;
datasize = size(House,1);
ker = 'rbf';

connectivity = 0.1;
agentNum = 100;
dim = size(X,2);

agentIndex = zeros(agentNum,1);
agentPos = zeros(agentNum,dim);
adjacency = eye(agentNum);
agentObs = zeros(agentNum, 1);

testX = zeros(agentNum, dim);
testRefY = zeros(agentNum,1);

%generate adjacency
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

%generate agent position
for i=1:1:agentNum
    index = uint32(1+(size(X,1)-1)*rand());
    agentIndex(i,1) = index;
    
    testX(i,:) = X(index,:);
    testRefY(i,1) = Y(index,1);
end

%initial from observation
for i=1:1:agentNum
    agentObs(i,1) = Y(agentIndex(i),:);
    agentPos(i,:) = X(agentIndex(i),:);
end

C = cell(agentNum,1);
K = cell(agentNum,1);

for i=1:1:agentNum
    C{i,1} = zeros(adjacencyCnt(i,1)+1,1);
    K{i,1} = zeros(adjacencyCnt(i,1)+1,dim);    
end

posIdx = zeros(agentNum,1);
adjacentList = cell(agentNum, 1);
RY = cell(agentNum,1); % Y for reference

for i = 1:1:agentNum
    % add slef position
    sensablePos = agentPos(i,:);
    
    adjacentIdx = [];
    RealObservation = Y(agentIndex(i,1),:);
    for j = 1:1:agentNum
        if(adjacency(i,j) > 0 && i~=j)
            sensablePos = [sensablePos; agentPos(j,:)];
            adjacentIdx = [adjacentIdx ; j];
            RealObservation = [RealObservation ; Y(agentIndex(j,1),:)];
            
        end            
    end
    K{i,1} = sensablePos;
    adjacentList{i,1} = adjacentIdx;
    RY{i,1} = RealObservation;
end

epoch = 20;
error = zeros(2, epoch);

Predict = zeros(agentNum,epoch);
Observation = zeros(agentNum,epoch);

for t = 1:1:epoch
    
    t
    
    for i=1:1:agentNum
        %update observation
        Observation(i,t) = Y(agentIndex(i,1));
        %update predict
        tempC = C{i,1};
        tempObsX = K{i,1};
        xPos = agentPos(i,:);
        estY = KernelEst(tempC, ker, tempObsX, xPos);
        Predict(i,t) = estY;
    end
    
    for i = 1:1:agentNum
        
        % update observation   
        obsY = Observation(i,t);

        obsX = K{i,1};
        % update predict from neighbors
        adjIdx = adjacentList{i,1};
        for j=1:1:adjacencyCnt(i)
            %tempC = C{j,1};
            %tempObsX = K{j,1};
            %obsAdjX = obsX(j+1,:);
            %estY = KernelEst(tempC, ker, tempObsX, obsAdjX);
            
            estY = Predict(adjIdx(j,1),t);
            obsY = [obsY; estY];
        end

        % calculate C
        tempC = C{i,1};
        tempC = KernelRegIt(obsX, obsY, lamda, ker, tempC);
        C{i,1} = tempC;
        
    end
    
    % test over all errors
    k = 4;
    for i=1:1:agentNum
       testY(i,1)  =  NetPredict(agentPos, C, K, testX(i,:), k, ker);
       testY(i,2) = NetPredict2(agentNum, C ,K ,testX(i,:), ker);
    end
    
    errAbs1 = abs(testY(:,1) - testRefY);
    errAbs2 = abs(testY(:,2) - testRefY);
    errAbsRt1 = errAbs1./testRefY;
    errAbsRt2 = errAbs2./testRefY;
    error(1,t) = sum(errAbsRt1) / size(testRefY,1);
    error(2,t) = sum(errAbsRt2) / size(testRefY,1);
    
end


