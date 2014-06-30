function score = Score(PossiblePath,depth)
% score = Score(PossiblePath)
% Translate a robot's path into a score for that path
% Use the implicit robot observation model (GOTCHA)
% Use the global variable P to compute the score

global P;

% If some of the cells are zero in possible path, then score just that
% portion of the path found to date.

%PartialPath = setdiff(PossiblePath,[0,0],'rows');
PartialPath = PossiblePath;
%[pathlength,tmp] = size(PartialPath);
score = 0;
localP = P;
for t=1:depth
    Robot = PartialPath(t,:);
    score = score + localP(Robot(1),Robot(2));
    localP(PartialPath(t,1),PartialPath(t,2)) = 0* ...
        localP(PartialPath(t,1),PartialPath(t,2));  
    % GOTCHA: MAGIC number for observation.  Assumes robot know for
    % certain if something of interest is in its same cell
    neighbors = GetNeighbors(Robot,1); %  GOTCHA: Assumes robot can see 2 cells. MAGIC Number
    
    [NumNeighbors,tmp] = size(neighbors);
    for i=1:NumNeighbors
        score = score + (1-0.75)*localP(neighbors(i,1),neighbors(i,2));
        localP(neighbors(i,1),neighbors(i,2)) = 0.75 * localP(neighbors(i,1),neighbors(i,2));
        % GOTCHA: MAGIC number for observation.  Assumes robot only knows
        % 25% of the time if something of interest is in a neighboring cell
    end;
end;
%PartialPath
%score
end