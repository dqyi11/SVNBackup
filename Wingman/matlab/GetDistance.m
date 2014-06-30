function distance = GetDistance(Node1, Node2)

   global HexagonSize;
   
   Node1Pos = zeros(1,2);
   Node2Pos = zeros(1,2);

   [Node1Pos(1,1) Node1Pos(1,2)] = Hexagonindex2Center(Node1(1),Node1(2),HexagonSize);
   [Node2Pos(1,1) Node2Pos(1,2)] = Hexagonindex2Center(Node2(1),Node2(2),HexagonSize);

   distance = sqrt((Node1Pos(1,1)-Node2Pos(1,1))^2 + (Node2Pos(1,2)-Node2Pos(1,2))^2);
  
end