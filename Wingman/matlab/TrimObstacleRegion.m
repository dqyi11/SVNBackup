function nonObstacleRegion = TrimObstacleRegion(region)

   global obstacle;

   % trim those nodes are obstacles
   nonObstacleRegion = setdiff(region, obstacle, 'rows');
   %nonObstacleRegion = region;
   
   
end