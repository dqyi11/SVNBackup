function UpdateMap(agentPath)

   global h P;

   for t=1:length(agentPath)
      Agent = agentPath(t,:);
      set(h(Agent(1),Agent(2)),'FaceColor',[1,1,1]);
     
      if (-inf ~= P(Agent(1), Agent(2)))
         
         % Update probability map
         P(Agent(1),Agent(2)) = 0;
         NeighborCells = GetNeighbors(Agent,1); % MAGIC NUMBER: 1 nearest neigbors
         [I,tmp] = size(NeighborCells);
         for i=1:I
            if (-inf ~= P(NeighborCells(i,1),NeighborCells(i,2))) 
               P(NeighborCells(i,1),NeighborCells(i,2))=0.75*...
               P(NeighborCells(i,1),NeighborCells(i,2));
               % MAGIC NUMBER: 0.75 is detection rate.
               set(h(NeighborCells(i,1),NeighborCells(i,2)),'FaceColor',...
                  [1,1,1]-P(NeighborCells(i,1),NeighborCells(i,2)));
            end
         end
         %drawnow
         %pause
      end
   end
   drawnow
end