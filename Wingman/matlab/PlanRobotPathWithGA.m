function [robotPath] = PlanRobotPathWithGA(startNode, humanPath, tolerantDistance)

   [pathLength, tmp] = size(humanPath);
   
   global P StoringScore StoringBestScore;
   MAX_ITERATION_TIME = 1000;
   GENE_NUM = 200;
   NUM_TOP_GENES = min(floor(GENE_NUM/3), 10);
   MUTATION_THRESHOLD = 0.5;
   
   geneLength = pathLength - 1;
   
   robotPath = zeros(pathLength, 2);
   geneScore = zeros(GENE_NUM);
   
   % seven possible value for six directions
   genes = floor(7*rand(GENE_NUM,geneLength));
   % new generation of gene
   tmpgenes = floor(7*rand(GENE_NUM,geneLength));
   
   hbestgene=zeros(1,geneLength);  % Handle for path segments
   
   for it = 1:MAX_ITERATION_TIME
      
      geneScore = zeros(GENE_NUM, 1);
      
      %{
      for gene_id = 1:GENE_NUM
         
         robotPath(1,:) = startNode;
         
         for stepIndex = 1:geneLength
            
            robotPath(stepIndex+1,:) = NextPosition(stepIndex, genes(gene_id, stepIndex));
             
         end
         
         geneScore(gene_id) = Score(robotPath, pathLength);
      
      end
      %}
      
      geneScore = ScoreGene(startNode, genes, humanPath, tolerantDistance);
      
      StoringScore = [StoringScore; geneScore];
      
      [ranked_score, arg_rank_score] = sort(geneScore, 'descend');
     
         
      % KEEP THE TOP GENES
      for (i=1:NUM_TOP_GENES)  % number of top genes kept each generation.
         tmpgenes(i,:) = genes(arg_rank_score(i),:);
      end
      
      StoringBestScore = [StoringBestScore; ranked_score]
    
 
      % MUTATE THE TOP GENES
      for (i=NUM_TOP_GENES+1:2*NUM_TOP_GENES)  % number of top genes that have mutations
         tmpgenes(i,:) = genes(arg_rank_score(i-NUM_TOP_GENES),:);
         mutate = rand(1,geneLength)>MUTATION_THRESHOLD;  % genes above threshold change
         tmpi = find(mutate==1);  % The index of the genes that will mutate
         if (sum(mutate)>0)  % Only mutate if there is at least one gene that needs to change
            tmpgenes(i,tmpi) = floor(7*rand(1,sum(mutate)));  % sum gives number of genes needing to change
         end;
      end
      
      % CROSSOVER THE TOP GENES
      % Choose the cross over point -- not first step or last
      cross_index = 1+floor((geneLength-1) * rand(1,1));  % the first or last step will not be used for crossover
      % New gene has top ranked gene's head and second ranked gene's tail
      tmpgenes(2*NUM_TOP_GENES+1,:) = tmpgenes(1,:); 
      tmpgenes(2*NUM_TOP_GENES+1,[cross_index:geneLength]) = tmpgenes(2*NUM_TOP_GENES+2,[cross_index:geneLength]);
      % New gene has second ranked gene's head and top ranked gene's tail
      tmpgenes(2*NUM_TOP_GENES+2,:) = tmpgenes(2,:); 
      tmpgenes(2*NUM_TOP_GENES+2,[cross_index:geneLength]) = tmpgenes(2*NUM_TOP_GENES+1,[cross_index:geneLength]);
      % New gene has top ranked gene's head and third ranked gene's tail
      tmpgenes(2*NUM_TOP_GENES+1,:) = tmpgenes(1,:); 
      tmpgenes(2*NUM_TOP_GENES+1,[cross_index:geneLength]) = tmpgenes(2*NUM_TOP_GENES+3,[cross_index:geneLength]);
    
      genes = tmpgenes;
      
   end
   
   %recalculate the final score and sort
   geneScore =  ScoreGene(startNode, genes, humanPath, tolerantDistance);
   [ranked_score, arg_rank_score] = sort(geneScore, 'descend');
   
   hbestgene = genes(arg_rank_score(1),:);
   
   robotPath = GeneratePath(startNode, hbestgene);
   
   DrawNodeSequenceOnMap(robotPath, 'blue');
   
   figure(3);
   
   plot(1:MAX_ITERATION_TIME, StoringBestScore);
   
   save('BestScoreFromGA.mat', 'StoringBestScore');

end