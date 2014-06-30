function curveEnergy = GetCurveEnergy(nodes, chainIndex, chainNodes)
   
   [nodeNum, tmp] = size(nodes);
   curveEnergy = zeros(nodeNum, 1);
   [chainLength, tmp] = size(chainNodes);
   
   for i = 1:nodeNum
      
      if chainIndex == 1
         
         % do not care curve
         curveEnergy(i) = 0;

      elseif chainIndex == chainLength

         % do not care curve
         curveEnergy(i) =  0;
         
      else

         curveEnergy(i) = abs(atan((nodes(i,2)-chainNodes(chainIndex+1,2))/(nodes(i,1)-chainNodes(chainIndex+1,1)))...
                          - atan((nodes(i,2)-chainNodes(chainIndex-1,2))/(nodes(i,1)-chainNodes(chainIndex-1,1))));
         
         %(GetDistance(nodes(i,:), chainNodes(chainIndex+1,:)))^2 + (GetDistance(nodes(i,:), chainNodes(chainIndex-1,:)))^2;

      end
       
   end

end