function totalEnergy = GetTotalEnergy(nodes, chainIndex, chainNodes, humanPos)

   [nodeNum, tmp] = size(nodes);
   totalEnergy = zeros(nodeNum, 1);

   a = 0.4;
   b = 1;
   c = 0;
   d = 0.2;
   e = 0;

   internalEnergy = a * GetInternalEnergy(nodes, chainIndex, chainNodes);
   externalEnergy = b * GetExternalEnergy(nodes);
   exclusiveEnergy = c* GetExclusiveEnergy(nodes, humanPos);
   obstacleEnergy = d * GetObstacleEnergy(nodes);
   curveEnergy = e * GetCurveEnergy(nodes, chainIndex, chainNodes);

   totalEnergy =  - internalEnergy + externalEnergy - exclusiveEnergy - obstacleEnergy - curveEnergy;

end