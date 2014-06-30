function e = GetEntropy(prob)

   if(prob==0)
       e = 0;
   end
   if(prob==1)
       e = 0;
   else
       e = -(prob*log2(prob)+(1-prob)*log2(1-prob));
   end
   

end