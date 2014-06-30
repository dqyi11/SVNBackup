function leftWingNodes = SelectOnlyLeftWingSide(selectableNodes, humanStepIndex, humanPath)
   
   global HexagonSize;
   
   [humanPathNum, tmp] = size(humanPath);
   [selectableNodeNum, tmp] = size(selectableNodes);
   
   leftWingNodes = selectableNodes;
   
   %disable this function
   %return;
   
   [currentStepX, currentStepY] = Hexagonindex2Center(humanPath(humanStepIndex,1),humanPath(humanStepIndex,2),HexagonSize);
   humanForeAngel = 0;
   humanBackAngel = 0;
   %calc human orientation
   if humanStepIndex == 1 

      [nextStepX, nextStepY] = Hexagonindex2Center(humanPath(humanStepIndex+1,1),humanPath(humanStepIndex+1,2),HexagonSize);
      humanForeAngel = atan2(nextStepY-currentStepY, nextStepX-currentStepX);
     
      if humanForeAngel <= 0
         humanBackAngel = humanForeAngel + pi;
      else
         humanBackAngel = humanForeAngel - pi;
      end    

   elseif humanStepIndex == humanPathNum

      [prevStepX, prevStepY] = Hexagonindex2Center(humanPath(humanStepIndex-1,1),humanPath(humanStepIndex-1,2),HexagonSize);
      humanBackAngel = atan2(prevStepY-currentStepY, prevStepX-currentStepX);
      if humanBackAngel <= 0
         humanForeAngel = humanBackAngel + pi;
      else
         humanForeAngel = humanBackAngel - pi;
      end 


   else

      [nextStepX, nextStepY] = Hexagonindex2Center(humanPath(humanStepIndex+1,1),humanPath(humanStepIndex+1,2),HexagonSize);
      [prevStepX, prevStepY] = Hexagonindex2Center(humanPath(humanStepIndex-1,1),humanPath(humanStepIndex-1,2),HexagonSize);
      humanForeAngel = atan2(nextStepY-currentStepY, nextStepX-currentStepX);
      humanBackAngel = atan2(prevStepY-currentStepY, prevStepX-currentStepX);
   end 

   if humanForeAngel < 0
      humanForeAngel = humanForeAngel + 2*pi;
   end

   if humanBackAngel < 0 
      humanBackAngel = humanBackAngel + 2*pi;
   end
   
   for i = 1:selectableNodeNum
       
      [NodeX, NodeY] = Hexagonindex2Center(selectableNodes(i,1),selectableNodes(i,2),HexagonSize);     
      turnAngel = atan2(NodeY-currentStepY, NodeX-currentStepX);
      
      %check if it is on left wing

      
      if turnAngel < 0 
         turnAngel = turnAngel + 2*pi
      end   
      
      acceptable = -1;
      if humanForeAngel > humanBackAngel
         if humanForeAngel < pi
            if turnAngel < humanForeAngel && turnAngel > humanBackAngel
               acceptable = 1; 
            end
         else
             if turnAngel > humanForeAngel || turnAngel < humanBackAngel
                acceptable = 1; 
             end
         end    
      elseif humanForeAngel < humanBackAngel  
         if humanForeAngel < pi
            if turnAngel > humanForeAngel && turnAngel < humanBackAngel
               acceptable = 1;
            end
         else                
            if turnAngel < humanForeAngel || turnAngel > humanBackAngel
                acceptable = 1;  
            end
         end    
      end   
      
      if acceptable == -1
        
         leftWingNodes = setdiff(leftWingNodes, selectableNodes(i,:), 'rows');
            
      end
          
   end

end