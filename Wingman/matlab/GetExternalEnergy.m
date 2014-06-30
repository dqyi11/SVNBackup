function externalEnergy = GetExternalEnergy(nodes)

   global P;
   
   [nodeNum, tmp] = size(nodes);
   externalEnergy = zeros(nodeNum, 1);

   K = 2;

   for i = 1:nodeNum

      externalEnergy(i,1) = P(nodes(i,1), nodes(i,2));
      
      neighborNodes = GetNeighbors(nodes(i,:),K);
      acceptableRegion = TrimObstacleRegion(neighborNodes);
 
      [acceptNum, tmp] = size(acceptableRegion);

      for j = 1:acceptNum

         if -inf ~= P(acceptableRegion(j,1), acceptableRegion(j,2))
            externalEnergy(i,1) = externalEnergy(i,1) + P(acceptableRegion(j,1), acceptableRegion(j,2));
         else
            % we will give the obstacle with P = 1, like we are not
            % interested
            externalEnergy(i,1) = externalEnergy(i,1) + 1;
         end 
      end

   end

end