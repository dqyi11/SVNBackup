function geneScore = ScoreGene(startNode, genes, humanPath, tolerantDistance)

    [geneNum, geneLength] = size(genes);   

    for gene_id = 1:geneNum

      robotPath(1,:) = startNode;

      for stepIndex = 1:geneLength
          
         nearHumanCells = GetNeighbors(humanPath(stepIndex+1,:), tolerantDistance);

         robotPath(stepIndex+1,:) = NextPositionWithObstacle(robotPath(stepIndex,:), genes(gene_id, stepIndex), nearHumanCells);

      end

      geneScore(gene_id) = Score(robotPath, geneLength+1);

   end

end