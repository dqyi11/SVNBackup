function NeighborCells = GetNeighbors(Agent,K);
% Find all neighbors with in K steps of the agent located at Agent
% NeighborCells = GetNeighbors(Agent,K)

    % global STAY EAST SE SW WEST NW NE
    global N;
    
    if K==0 NeighborCells = [];
    else
        NeighborCells = [Agent + [2,0];...
            Agent + [1,-1];...
            Agent + [-1,-1];...
            Agent + [-2,0];...
            Agent + [-1,1];...
            Agent + [1,1]];  % All neighbors one step away
        for j = 2:K
            [I,tmp] = size(NeighborCells);
            for i = 1:I
                Agent = NeighborCells(i,:);
                NeighborCells = [NeighborCells;[Agent + [2,0];...
                    Agent + [1,-1];...
                    Agent + [-1,-1];...
                    Agent + [-2,0];...
                    Agent + [-1,1];...
                    Agent + [1,1]]];  % All neighbors j steps away            
            end;
            % Remove redundant cells.
            NeighborCells = unique(NeighborCells,'rows');
        end;
        % Remove out of bound cells.
        tmp = find(NeighborCells(:,2)<=N & NeighborCells(:,2)>0); 
        NeighborCells = NeighborCells(tmp,:);
        tmp = find(NeighborCells(:,1) > 0 & NeighborCells(:,1)<=2*N);
        NeighborCells = NeighborCells(tmp,:);
        
%        remove = [];
%        [I,tmp] = size(NeighborCells);
%        for i=1:I
%            for j=i+1:I
%                %if (i==j) continue;
%                %else 
%                if (sum(NeighborCells(i,:) == NeighborCells(j,:))==2)
%                    remove = [remove,j];
%                end;
%            end;
%        end;
%        keep = setdiff([1:I],unique(remove));
%        NeighborCells = NeighborCells(keep,:);
    end;
end

