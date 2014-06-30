function exclusiveEnergy = GetExclusiveEnergy(nodes, humanPos)

   [nodeNum, tmp] = size(nodes);
   exclusiveEnergy = zeros(nodeNum, 1);

   for i = 1:nodeNum

      exclusiveEnergy(i) = 1/ exp((GetDistance(nodes(i,:), humanPos)));

   end

end