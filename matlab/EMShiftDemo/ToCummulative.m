function [rCummulative]=ToCummulative(r)
rCummulative=sum(tril(repmat(r,length(r))),2)';