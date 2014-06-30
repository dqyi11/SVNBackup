function [robotPath] = PlanRobotPathWithDFS(startNode, humanPath, tolerantDistance)
    
    global ExpectedStep;
    global PossiblePath BestPossiblePath PossiblePathScore MaxDepth;

    %MaxDepth = length(agentpath) + 1; % If you want to search to the end of the path
    MaxDepth = length(humanPath) * ExpectedStep;
    PossiblePath = zeros(MaxDepth,2);
    PossiblePath(1,:) = startNode;
    BestPossiblePath = PossiblePath;
    PossiblePathScore = -inf;

    TraverseWithDFS(startNode, tolerantDistance, 1, humanPath);
    
    robotPath = BestPossiblePath;    

    DrawNodeSequenceOnMap(robotPath);
end