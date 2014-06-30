function UpdateMapColor()

   global h P N;
   
   % Shade the cells according to the uncertainty in them. 
   for (i=2:2:2*N)
      for (j=2:2:N)
          if (-inf ~= P(i,j))
             set(h(i,j),'FaceColor',1-[P(i,j),P(i,j),P(i,j)])
          else   
             set(h(i,j), 'FaceColor', 'Red');
          end
          
          if (-inf ~= P(i-1,j-1))
             set(h(i-1,j-1),'FaceColor',1-[P(i-1,j-1),P(i-1,j-1),P(i-1,j-1)])
          else       
             set(h(i-1,j-1), 'FaceColor', 'Red');
      
          end
      end
   end

end