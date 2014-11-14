function [histogram]=colorhistogram(imIn,nBinsD,varargin)
%make color histogram
% hist=colorhistogram(imIn,nBinsD,imMask,sigma)
%
% Author: Z.Zivkovic
% Date: 21-6-2003

nBins=nBinsD^3;
hist=zeros(nBins,1);%RGB
nPoints=size(imIn,1)*size(imIn,2);
H=zeros(nPoints,nBins);
imIn=double(reshape(imIn,nPoints,3));
if (nargin==3)
    %use kernel mask
    imMask=varargin{1};
    imMask=reshape(imMask,1,nPoints);
else
    imMask=ones(1,nPoints)./nPoints;
end

nDiv=256/nBinsD;
imIn=floor(imIn/nDiv);
%H
for i=1:nPoints
    iBin=nBinsD^2*imIn(i,1)+nBinsD*imIn(i,2)+imIn(i,3);
    if (~isnan(iBin))
        H(i,iBin+1)=1;
    end
end
hist=imMask*H;
histogram.data=hist;
histogram.H=H;
histogram.nBinsD=nBinsD;