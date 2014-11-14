function [x,V]=SToMeanVar(s)
x=s(1:2);
L=[s(3:4)';0 s(5)];
V=L'*L;
