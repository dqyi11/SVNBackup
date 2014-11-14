function [s,val,iMax]=MaxGMM(rK,rS,rV)
%s=MaxGMM(rK,rS,rV)
%find the maximum of the GMM
%simple solution when the modes are very separate
%
%University of Amsterdam, The Netherlands
%Author:Zoran Zivkovic, www.zoranz.net
%Date: 17-5-2005

nModes=length(rK);
rP=zeros(1,nModes);
for iMode=1:nModes
    %assume diag matrix and components separate
    rP(iMode)=rK(iMode)/sqrt(det(rV(:,:,iMode)));
end
[val,iMax]=max(rP);
s=rS(:,iMax);