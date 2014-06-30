function AddObstacle(x, y)

   global obstacle P;

   P(x,y) = -inf;
   obstacle =[obstacle;[x,y]];

end