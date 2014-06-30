function keyNodes = FindRobotPathKeyNode(startNode, humanPath, tolerantDistance)

   keyNodes = humanPath;

   [chainNodeNum, tmp] = size(keyNodes);

   tmpChain = zeros(chainNodeNum, tmp);
   
   % startNode will not be moved
   tmpChain(1,:) = startNode;

   searchStopped = -1;

   stepCnt = 0;
   while searchStopped < 0 && stepCnt < 10
      for i = 2:chainNodeNum
   
         % the key nodes are moving step by step
         neighors = GetNeighbors(keyNodes(i,:), 1);
         %neighors = [neighors; keyNodes(i,:)];
         aroundHuman = GetNeighbors(humanPath(i,:), tolerantDistance);
         choice = intersect(neighors, aroundHuman, 'rows'); 
         choice = SelectOnlyLeftWingSide(choice, i, humanPath);
         choice = [choice; keyNodes(i,:)];

         totalEnergy = GetTotalEnergy(choice, i, keyNodes, humanPath(i,:));

         [choiceNum, tmp] = size(totalEnergy);
         bestEnergy = - inf;
         bestIndex = 0;
         for j = 1:choiceNum
            if totalEnergy(j) > bestEnergy
               bestEnergy = totalEnergy(j);
               bestIndex = j; 
            end    
         end
         
         whichToChoose = bestIndex;

         tmpChain(i,:) = choice(whichToChoose, :);
      end

      if tmpChain == keyNodes
         searchStopped = 1;
      else

         searchStopped = -1;
         keyNodes = tmpChain;
      end
      
      stepCnt = stepCnt + 1;
      
   end

end