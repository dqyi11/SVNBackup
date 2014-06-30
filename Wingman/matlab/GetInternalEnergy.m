function internalEnergy = GetInternalEnergy(nodes, chainIndex, chainNodes)

   [nodeNum, tmp] = size(nodes);
   internalEnergy = zeros(nodeNum, 1);
   [chainLength, tmp] = size(chainNodes);
   
   global ExpectedStep;

   for i = 1:nodeNum

      if chainIndex == 1

         internalEnergy(i) =   (GetDistance(nodes(i,:), chainNodes(chainIndex+1,:)))^2;
         if internalEnergy(1) > ExpectedStep^2
             
            internalEnergy(i) = 10 * internalEnergy(i) ;
             
         end    

      elseif chainIndex == chainLength

         internalEnergy(i) =   (GetDistance(nodes(i,:), chainNodes(chainIndex-1,:)))^2;

         if internalEnergy(1) > ExpectedStep^2
             
            internalEnergy(i) = 10 * internalEnergy(i) ;
             
         end 
         
      else

         internalEnergy(i) =   (GetDistance(nodes(i,:), chainNodes(chainIndex+1,:)))^2 + (GetDistance(nodes(i,:), chainNodes(chainIndex-1,:)))^2;

         if internalEnergy(1) > (ExpectedStep * 2)^2
             
            internalEnergy(i) = 10 * internalEnergy(i) ;
             
         end 
      end
      
      % it will get penalty if the distance is out of the expected steps
      % num  
  
      
   end

end