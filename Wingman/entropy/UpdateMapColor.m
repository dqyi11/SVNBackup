function UpdateMapColor()

   global h P N;
   
   [pw,pl] = size(P);
   
   E = zeros(pw, pl);
   for (i=1:1:pw)
      for (j=1:1:pl)        
         E(i,j) = GetEntropy(P(i,j));        
      end
   end
   
   
   % Shade the cells according to the uncertainty in them. 
   for (i=2:2:2*N)
      for (j=2:2:N)
         set(h(i,j),'FaceColor',1-[E(i,j),E(i,j),E(i,j)])
      end
   end

end