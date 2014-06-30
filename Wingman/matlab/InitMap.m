function InitMap()

   global P h;
   global N HexagonSize;
   
   d = HexagonSize; % Size of hexagons
   P = zeros(N, N);
   h = zeros(N, N); % Array of handles to hexagons
   
   % Draw tessalation and agent moving on it.  While doing this, compute the
   % probability distribution that remains after the human has traversed. Use
   % this to construct the performance criterion that the robot wants to
   % maximize as it plans its path.
   for i=2:2:2*N
      for j=2:2:N
         try
            [x,y] = Hexagonindex2Center(i,j,d);
         catch ME
            if strcmp(ME.identifier,'VerifyInput:OutOfBounds')
                disp('Input is out of bounds in Hexagonindex2Center');
            end
         end
         h(i,j) = Hexagon(x,y,d);
         P(i,j) = 1;
         P(i-1,j-1)=1;
         try
            [x,y] = Hexagonindex2Center(i-1,j-1,d);
         catch ME
            if strcmp(ME.identifier,'VerifyInput:OutOfBounds')
                disp('Input is out of bounds in Hexagonindex2Center');
            end
         end
         h(i-1,j-1) = Hexagon(x,y,d);
         P(i-1,j-1) = 1;
      end
   end;
   set(gca,'DataAspectRatio',[1,1,1]);
   

end