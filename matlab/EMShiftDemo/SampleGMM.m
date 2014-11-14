function [rSamples]=SampleGMM(rPi,rMu,rV,nSamples,varargin)
%SampleGMM(rPi,rMu,rV,nSamples,rLimits,alpha)
%sample from a GMM
%rLimits [smin smax]- constrain to a rectangular region
%Author:Z.Zivkovic
nD=size(rMu,1);
rSamples=zeros(nD,nSamples);
alpha=1;
rLimits=[];
if (nargin>=5)
    rLimits=varargin{1};
end
if (nargin==6)
    alpha=varargin{2};
end

rT=ToCummulative(rPi);


if (size(rV,3)==1)%single cov matrix
    sqrt_V=chol(rV);
    for i=1:nSamples
        %sample from the multinomial
        ri=find(rT>=rand(1));iMode=ri(1);
        st=rMu(:,iMode);%sample from the corresponding mode
        %allow some wider sampling with probability alpha
        if (rand(1)>=(1-alpha))
            %add noise            
            st=st+sqrt_V'*randn(nD,1);
        else
            %sample wide noise
            st=st+2*sqrt_V'*randn(nD,1);
        end
        if(~isempty(rLimits))
            st=min(max(st,rLimits(:,1)),rLimits(:,2));%inside region
        end
        rSamples(:,i)=st;
    end  
else
    for i=1:nSamples
        %sample from the multinomial
        ri=find(rT>=rand(1));iMode=ri(1);
        st=rMu(:,iMode);%sample from the corresponding mode
        %allow some wider sampling with probability alpha
        if (rand(1)>=(1-alpha))
            %add noise
            %st=st+sqrt(W)*randn(nStates,1);
            sqrt_V=chol(rV(:,:,iMode));
            st=st+sqrt_V'*randn(nD,1);
        else
            %sample wide noise
            sqrt_V=chol(rV(:,:,iMode));
            st=st+2*sqrt_V'*randn(nD,1);
        end
        if(~isempty(rLimits))
            %st(1:2)=min(max(st(1:2),rLimits(1:2)'),rLimits(3:4)');%inside image-nBord
            st=min(max(st,rLimits(:,1)),rLimits(:,2));%inside region
        end
        rSamples(:,i)=st;
    end
end


