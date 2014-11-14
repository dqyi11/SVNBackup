function rho=Similarity(im0,xt,V,histO,K)
%Bhattacharayya coefficient

%get target region
[data,rX,rY]=GetAffineRegion(im0,xt,V,K.rX,K.rY,K.sigmas);
%get histogram
[histR]=ColorHist(data,histO.nBinsD,K.rK);
rho=sum(sqrt(histO.data.*histR.data));