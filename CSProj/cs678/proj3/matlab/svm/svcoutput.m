function predictedY = svcoutput(trnX,trnY,tstX,ker,alpha,bias)

n = size(trnX,1);
m = size(tstX,1);
H = zeros(m,n);  
for i=1:m
  for j=1:n
    H(i,j) = trnY(j)*svkernel(ker,tstX(i,:),trnX(j,:));
  end
end

predictedY = sign(H*alpha + bias);

end
    