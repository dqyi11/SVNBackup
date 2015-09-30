function predictedY = dsvcoutput(tstX,v)

n = size(v,1);

W = v(1:n-1,1);

bias = v(n,1);

predictedY = sign(W' * tstX' + bias);

end