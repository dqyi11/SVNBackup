function LimitedDepthFirstSearch(currentSequence, endNode, searchRegion, depth, maxDepth)
 
   global BestScore  BestPossiblePath;
   global P;
   
   [length, tmp] = size(currentSequence);
      
   if depth > maxDepth
      return;
   end

   if (depth > maxDepth) && (BestScore > 0)
      return;
   end
   
   if currentSequence(length,:)==endNode 
      if depth>1
         score = Score(currentSequence, depth);
         if score > BestScore
            BestPossiblePath = currentSequence;
            BestScore = score;
         
            return;
          
         end
      end
   end
   
   RobotConnectedCells = GetNeighbors(currentSequence(length,:),1);  % Find all cells connected to the robot's immediate location.
   RobotConnectedCells = [RobotConnectedCells;currentSequence(length,:)];  % Allow the robot to stay in one place.

   Children = intersect(RobotConnectedCells,searchRegion,'rows');
   depth = depth + 1;
   [NumChildren,tmp] = size(Children);
   
   %{
   % Order the children in a greedy manner, so that child with highest
   % payoff is visited first.  This should help the pruning be more
   % effective.
   ScoreVector = zeros(NumChildren,1);
   for i=1:NumChildren
      ScoreVector(i) = P(Children(i,1),Children(i,2));
   end
   [tmp,order] = sort(-ScoreVector);
   tmp = Children;
   for i=1:NumChildren
      tmp(i,1) = Children(order(i),1);
      tmp(i,2) = Children(order(i),2);
   end
   
   Children = tmp;
   % End of sorting children.   
   %}
   
   %NumberPruned = 0;
   for i=1:NumChildren 
      NewCurrentSequence = [currentSequence; Children(i,:)];
      % Add a heuristic that ends this link if the best path found so far
      % is better than the maximum that can be found.
      
      %score = Score(CurrentSequence,depth, P);
      %if (PossiblePathScore < score + (LimitedStep-depth)*(0.25*6+.75))
         LimitedDepthFirstSearch(NewCurrentSequence, endNode, searchRegion, depth, maxDepth);
         % MAGIC number: .25 is maximum cell value for each cell
         % assuming a .75 detection rate.  The 1 is the value of the
         % cell occupied by the robot and the 6 is the number of
         % neighbors.
         %
         % GOTCHA.  The 6 is for a robot sensor range of 1 cell.
         %else
      %end 
   end
   %fprintf(1,'In Traverse at level %d, Number pruned = %d\n',depth,NumberPruned);
   
   
end
