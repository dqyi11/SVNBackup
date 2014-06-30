function [ cost ] = TraverseWithDFS(Robot, tolerantDistance, depth, humanPath)

% Traverse: Recursively traverse the tree of possible robot locations using
% a depth first traversal.  Robot is the robot's cell in the current
% path, R is the radius of communication defined as the maximum number of
% steps that a robot can wander from the agent, and depth is book keeping
% that allows us to detect when we have hit the end of the tree.
global MaxDepth PossiblePath PossiblePathScore BestPossiblePath
global P;

cost = -inf;
%PossiblePath

if depth == MaxDepth % If at bottom of the tree, score the tree.
    score = Score(PossiblePath,depth);
    if (score > PossiblePathScore)
        BestPossiblePath = PossiblePath;
        PossiblePathScore = score;
        score
        BestPossiblePath
    end
    %pause
else
    humanStep = fix(depth/4) + 1;
    Human = humanPath(humanStep,:);
    HumanNeighborCells = GetNeighbors(Human,tolerantDistance); % Find all cells within R steps of human
    HumanNeighborCells = [HumanNeighborCells;Human]; % The human is always a neighbor of itself.
    RobotConnectedCells = GetNeighbors(Robot,1);  % Find all cells connected to the robot's immediate location.
    RobotConnectedCells = [RobotConnectedCells;Robot];  % Allow the robot to stay in one place.
    Children = intersect(RobotConnectedCells,HumanNeighborCells,'rows');
    depth = depth + 1;
    [NumChildren,tmp] = size(Children);
    
    % Order the children in a greedy manner, so that child with highest
    % payoff is visited first.  This should help the pruning be more
    % effective.
    ScoreVector = zeros(NumChildren,1);
    for (i=1:NumChildren)
        ScoreVector(i) = P(Children(i,1),Children(i,2));
    end;
    [tmp,order] = sort(-ScoreVector);
    tmp = Children;
    for (i=1:NumChildren)
        tmp(i,1) = Children(order(i),1);
        tmp(i,2) = Children(order(i),2);
    end;
    Children = tmp;
    % End of sorting children.
    
    %NumberPruned = 0;
    for i=1:NumChildren
        PossiblePath(depth,:) = Children(i,:);                
        % Add a heuristic that ends this link if the best path found so far
        % is better than the maximum that can be found.
        score = Score(PossiblePath, depth);
        %fprintf(1,'In Traverse at level %d searching node [%d,%d] Utility=%.2f\n',...
        %    depth, Children(i,1),Children(i,2),score);
        if (PossiblePathScore < score + (MaxDepth-depth)*(0.25*6+.75))
            cost = TraverseWithDFS(Children(i,:), tolerantDistance, depth, humanPath);
            % MAGIC number: .25 is maximum cell value for each cell
            % assuming a .75 detection rate.  The 1 is the value of the
            % cell occupied by the robot and the 6 is the number of
            % neighbors.
            %
            % GOTCHA.  The 6 is for a robot sensor range of 1 cell.
        %else
        end;  % 
    end
    %fprintf(1,'In Traverse at level %d, Number pruned = %d\n',depth,NumberPruned);
end

end