function [agent, agentPath] = GeneratePathInMap(agent, actionSequence)
   
   % global STAY EAST SE SW WEST NW NE; %represent seven actions for agent to move in a hexagon world
   
   agentPath = zeros(length(actionSequence)+1,2);
   agentPath(1,:) = agent;
   
   for t=1:length(actionSequence)
      %dir = floor(7*rand(1,1));
      dir = actionSequence(t);
      agent = NextPosition(agent,dir);
      agentPath(t+1,:) = agent;
      
   end

end