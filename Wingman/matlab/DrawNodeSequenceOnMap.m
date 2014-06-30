function DrawNodeSequenceOnMap(nodeSequence, color)

   global h;
   
   for t=1:length(nodeSequence)
     Agent = nodeSequence(t,:);
     set(h(Agent(1),Agent(2)),'FaceColor', color);
     
     drawnow
      %pause
   end

end