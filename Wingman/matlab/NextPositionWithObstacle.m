function NewAgent = NextPositionWithObstacle(Agent,dir, nearHuamnRegion)

   global obstacle;
   NewAgent = NextPosition(Agent, dir);
   
   if isempty(setdiff(NewAgent, obstacle, 'rows'))
      
      NewAgent = Agent;
   end
 
  if size(nearHuamnRegion) == size(setdiff(nearHuamnRegion, NewAgent, 'rows'))
      
      NewAgent = Agent;
   end

end