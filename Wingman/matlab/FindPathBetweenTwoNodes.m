function segmentPath = FindPathBetweenTwoNodes(startNode, endNode, humanStart, humanEnd, tolerantDistance)

   global BestPossiblePath BestScore;
   global P N LimitedStep ExpectedStep;  
   
   BestScore = 0;
   
   % if the two nodes are neighbor,
   % we plan not to generate another connection but
   startNeighbors = GetNeighbors(startNode, 1);
   %isEndIn = intersect(startNeighbors, endNode, 'rows');
   if 1 ~= isempty(intersect(startNeighbors, endNode, 'rows'))
      segmentPath = endNode;
      return;
   end

   % 1. get allowable walking region
   humanStartToleranceRegion = GetNeighbors(humanStart, tolerantDistance);
   searchStartRegion = TrimObstacleRegion(humanStartToleranceRegion);
   humanEndToleranceRegion = GetNeighbors(humanEnd, tolerantDistance);
   searchEndRegion = TrimObstacleRegion(humanEndToleranceRegion);
   searchRegion = union(searchStartRegion, searchEndRegion, 'rows');
   
   % 2. find the path from the start to end in limited step   
   BestScore = 0;
   tryStepNum = 2;
   while (BestScore ==0 && tryStepNum<=LimitedStep)
   
       LimitedDepthFirstSearch(startNode, endNode, searchRegion, 1, tryStepNum);
       tryStepNum = tryStepNum + 1;
   end
   
   %{
   if BestScore == 0
      
      LimitedDepthFirstSearch(startNode, endNode, searchRegion, 1, LimitedStep);
   end
   %}
 
   segmentPath = BestPossiblePath;
   
   %remove duplicate node since it will not lead to a move
   segmentPath = unique(segmentPath, 'rows');
   
end