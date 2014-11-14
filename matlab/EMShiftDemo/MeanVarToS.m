function s=MeanVarToS(x,V)
L=chol(V);
s=[x(1) x(2) L(1,1) L(1,2) L(2,2)]';
