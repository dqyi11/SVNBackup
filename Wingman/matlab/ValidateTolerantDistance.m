function inTolerantDistance = ValidateTolerantDistance(nodeSequence, humanStart, humanEnd)

   global tolerantDistance;

   [length, tmp] = size(nodeSequence);
   
   %assume they are all in tolerant distance
   inTolerantDistance = 1;
   
   for i = 1:length-1
      
      distance = GetDistance(nodeSequence(i,:), humanStart);
      
      if distance > tolerantDistance
         
         inTolerantDistance = -1;
         
         return;
      end  
      
      distance = GetDistance(nodeSequence(i,:), humanEnd);
      
      if distance > tolerantDistance
         
         inTolerantDistance = -1;
         
         return;
      end  
      
      
       
   end  

end