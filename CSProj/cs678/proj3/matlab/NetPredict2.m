function y = NetPredict2(agentNum, C, K, x, ker)

   y = 0;
   connTotal = 0;
   for i=1:1:agentNum
       tempC = C{i,1};
       tempK = K{i,1};
       conn = size(C, 1);
       connTotal = connTotal + conn;
       y = y + conn * KernelEst(tempC, ker, tempK, x);       
   end;
   y = y / connTotal;

end