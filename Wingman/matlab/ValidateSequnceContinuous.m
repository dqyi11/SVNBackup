function continuous = ValidateSequnceContinuous(nodeSequence)

   [length, tmp] = size(nodeSequence);
   
   global P;
   
   %assume it is continuous
   continuous = 1;
   
   for i = 1:length-1
      
      currentNode = nodeSequence(i,:);
      nextNode = nodeSequence(i+1,:);
      neighbors = GetNeighbors(currentNode,1,P);
      neighbors = [neighbors; currentNode];
      
      [neighborLength, tmp] = size(neighbors);
      continuous = -1;
      
      for j = 1:neighborLength
         if neighbors(j,:) == nextNode
            continuous = 1; 
         end    
      end
      
      if continuous == -1
         return;
      end    
   end    
end