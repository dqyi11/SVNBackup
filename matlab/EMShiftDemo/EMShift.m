function [xt,Vt]=EMShift(im0,xt,V,histO,K,beta,varargin)
%Calculate gradient step to maximize similarity between the target
%and the object histogram - Bhattacharayya coefficient
%The step is in both position and scale as described in 
%(original mean-shift does only position 
%- so if you do not use the resulting V you will get the standard meax-shift):
%Z.Zivkovic, B.Krose 
%An EM-like algorithm for color-histogram-based object tracking"
%IEEE Conference CVPR, June, 2004

%get target region
[data,rX,rY]=GetAffineRegion(im0,xt,V,K.rX,K.rY,K.sigmas);

%get histogram + mapping H pixels to bins
[histR]=ColorHist(data,histO.nBinsD,K.rK);
[riNonZero]=find(histR.data>0);%prevent divide by zero
rWeigths=(histR.H(:,riNonZero)*sqrt(histO.data(riNonZero)./histR.data(riNonZero))')'.*K.rK;
sumW=sum(rWeigths);
if (sumW>0) rQs=rWeigths/sumW;else rQs=rWeigths; end;
        
dx=[sum(rQs.*rX);sum(rQs.*rY)];
xt=xt+dx;
Vxy=sum(rQs.*rX.*rY);
Vt=beta*[sum(rQs.*rX.*rX),Vxy;Vxy,sum(rQs.*rY.*rY)];
%no convergence detection implemented here - uses fixed number of iterations

%clip
Vt(1,1)=max(Vt(1,1),1);Vt(2,2)=max(Vt(2,2),1);%clip to min 1
