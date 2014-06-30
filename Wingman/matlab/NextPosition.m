function NewAgent = NextPosition(Agent,dir)
   % given a direction, dir, move the agent from its current position
   % (Agent(1),Agent(2)) to a new position (NewAgent(1),NewAgent(2)) subject
   % to the bounds of the world.
   global STAY EAST SE SW WEST NW NE 
   global N

   switch dir
      case STAY
         NewAgent = Agent;
      case EAST
         NewAgent = Agent + [2,0];
      case SE
         NewAgent = Agent + [1,-1];
      case SW
         NewAgent = Agent + [-1,-1];
      case WEST
         NewAgent = Agent + [-2,0];
      case NW
         NewAgent = Agent + [-1,1];
      case NE
         NewAgent = Agent + [1,1];
   end
   
   if NewAgent(1) == 2*N+2 NewAgent(1) = 2*N;end;
   if NewAgent(1) == 2*N+1 NewAgent(1) = 2*N-1;end;
   if NewAgent(2) == N+2 NewAgent(2) = N;end;
   if NewAgent(2) == N+1 NewAgent(2) = N-1;end;
   if NewAgent(1) == 0 NewAgent(1) = 2;end;
   if NewAgent(1) == -1 NewAgent(1) = 1;end;
   if NewAgent(2) == 0 NewAgent(2) = 2;end;
   if NewAgent(2) == -1 NewAgent(2) = 1;end;

end