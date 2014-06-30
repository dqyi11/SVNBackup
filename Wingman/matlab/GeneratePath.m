function path = GeneratePath(startNode, gene)

   [tmp, geneLength] = size(gene);
   path = zeros(geneLength+1, 2);
   path(1,:) = startNode;
   
   for i = 1:geneLength
      
      path(i+1,:) = NextPosition(path(i,:), gene(i));
       
   end

end