function y = NetPredict(agentPos, C, K, x, k, ker)

   agentNum = size(agentPos,1);
   dim = size(agentPos, 2);
   decideAgents = zeros(k, 1);
   
   dist = zeros(agentNum, 1);
   
   for i = 1:1:agentNum
       dist(i,1) = Kernel(ker, agentPos(i,:), x);
   end
   
   [sortVal sortIdx] = sort(dist);
   
   y = 0;
   for j=1:1:k
       tempC = C{sortIdx(agentNum+1-j),1};
       tempK = K{sortIdx(agentNum+1-j),1};
       y = y + KernelEst(tempC, ker, tempK, x);
   end
   
   y = y / k;

end