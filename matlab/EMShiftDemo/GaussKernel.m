function [Kernel]=GaussKernel(C,varargin)
%K=GaussKernel(C,nSigma)
% Make a gauss kernel of appropriate size from a covariance matrix C
% Input: C- covariance matrix
%        nSigma - cut at nSigma (default 2.5)
% Output: K - kernel structure
%
% Author: Zoran Zivkovic
% Date:11-4-2005

if (nargin==2)
    fNSigma=varargin{1};
else
    fNSigma=2.5;
end
nPoints=100;
w=0:2*pi/nPoints:2*pi;
rCircle=fNSigma*[cos(w);sin(w)];%circle
nPoints=length(w);

K=chol(C);
rPlot1=K'*rCircle;

min1=min(rPlot1,[],2);
max1=max(rPlot1,[],2);

sigmax=K(1,1);
sigmay=K(2,2);

sizX=round((max1(1)-min1(1))/2);
sizY=round((max1(2)-min1(2))/2);

[rY,rX]=meshgrid(-sizY:sizY,-sizX:sizX);
dist=(rX.^2/sigmax^2+rY.^2/sigmay^2)/2;
K=exp(-dist);
K=K/sum(sum(K));%normalize
%figure,imshow(K,[])

%make the structure and precalcualte moments
Kernel.rK=reshape(K,1,[]);
Kernel.rX=reshape(rX,1,[]);
Kernel.rY=reshape(rY,1,[]);
Kernel.rXn=reshape(rX,1,[])./sigmax;%normalized
Kernel.rYn=reshape(rY,1,[])./sigmay;
%Kernel.rXX=Kernel.rX.*Kernel.rX;
%Kernel.rXY=Kernel.rX.*Kernel.rY;
%Kernel.rYY=Kernel.rY.*Kernel.rY;
Kernel.sigmas=[sigmax, sigmay];
Kernel.size=size(K);



