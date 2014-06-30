function [robotPath] = PlanRobotPath(startNode, humanPath, tolerantDistance)
   
   keyNodes = FindRobotPathKeyNode(startNode, humanPath, tolerantDistance);
   
   %DrawNodeSequenceOnMap(keyNodes, 'yellow');
   
   %localP = P;
   %localP = UpdateMap(localP, keyNodes);
   
   [keyNodesNum, tmp] = size(keyNodes);
   
   robotPath = keyNodes(1,:);
   
   for i = 2:keyNodesNum
       
      segmentPath = FindPathBetweenTwoNodes(keyNodes(i-1,:), keyNodes(i,:), humanPath(i-1,:), humanPath(i,:), tolerantDistance);
      %{
      inTolerance = ValidateTolerantDistance(segmentPath, keyNodes(i-1,:), keyNodes(i,:));
      if inTolerance < 0
         
         fprintf(1,'The segement between node [%d %d] and node [%d %d] go out of the tolerance', ...
                keyNodes(i-1,1), keyNodes(i-1,2), keyNodes(i,1), keyNodes(i,2));
          
      end 
      %}
      robotPath = [robotPath ; segmentPath]; 
      
   end
   
   robotPath = [robotPath; keyNodes(keyNodesNum,:)];
   
   %{
   continuous = ValidateSequnceContinuous(robotPath, P);
   if continuous < 0 
      fprintf(1,'This sequence is not continuous');
   end
   %}
   
   DrawNodeSequenceOnMap(robotPath, 'green');
   %DrawNodeSequenceOnMap(keyNodes, 'yellow');
end