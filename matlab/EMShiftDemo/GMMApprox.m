function [rKz,rz,rR]=GMMApprox(rSamples,im0,histO,K,beta,lambda)
%use nModes samples
[nStates,nModes]=size(rSamples);

if (0)
count=0;
rKz=zeros(1,nModes);rz=zeros(nStates,nModes);rR=zeros(nStates,nStates,nModes);
for iModes=1:nModes
    %local search
    %find the mode
    nIterationMax=6;
    [xt,V]=SToMeanVar(rSamples(:,iModes));%take a sample as the starting point
    for iIteration=1:nIterationMax
        [xt,V]=EMShift(im0,xt,V,histO,K,beta);
    end
    z=MeanVarToS(xt,V);
    
    %Prune
    rCurrOverlap=zeros(1,count);%works also for count=0
    for iLocal=1:count
        rCurrOverlap(iLocal)=Overlap(rz(:,iLocal),z);
    end
    if (isempty(find(rCurrOverlap>0.7)))
        %add the mode
        count=count+1;
        R=FitGauss(im0,z,histO,K)*lambda^2;
        rho=Similarity(im0,xt,V,histO,K);
        rKz(count)=sqrt(det(R))*exp(-0.5*(1-rho)/lambda^2);
        rz(:,count)=z;
        rR(:,:,count)=R;
    end
end
rKz=rKz(1:count);rz=rz(:,1:count);rR=rR(:,:,1:count);
rKz=rKz/sum(rKz);
%Prune or renormalize can be done also here
%[rKz,rz,rR]=PruneGMM(rKz,rz,rR,0.5);
else
    %method 2
    %remove similar samples immediately
    count=0;
    rSt=zeros(nStates,nModes);
    for iModes=1:nModes
        %Prune
        rCurrOverlap=zeros(1,count);%works also for count=0
        for iLocal=1:count
            rCurrOverlap(iLocal)=Overlap(rSt(:,iLocal),rSamples(:,iModes));
        end
        if (isempty(find(rCurrOverlap>0.6)))
            %add the mode
            count=count+1;
            rSt(:,count)=rSamples(:,iModes);
        end
    end
    nModes=count;
    rSamples=rSt(:,1:count);
    count=0;
rKz=zeros(1,nModes);rz=zeros(nStates,nModes);rR=zeros(nStates,nStates,nModes);
for iModes=1:nModes
    %local search
    %find the mode
    nIterationMax=6;
    [xt,V]=SToMeanVar(rSamples(:,iModes));%take a sample as the starting point
    for iIteration=1:nIterationMax
        [xt,V]=EMShift(im0,xt,V,histO,K,beta);
    end
    z=MeanVarToS(xt,V);
    
    %Prune
    rCurrOverlap=zeros(1,count);%works also for count=0
    for iLocal=1:count
        rCurrOverlap(iLocal)=Overlap(rz(:,iLocal),z);
    end
    if (isempty(find(rCurrOverlap>0.7)))
        %add the mode
        count=count+1;
        R=FitGauss(im0,z,histO,K)*lambda^2;
        rho=Similarity(im0,xt,V,histO,K);
        rKz(count)=sqrt(det(R))*exp(-0.5*(1-rho)/lambda^2);
        rz(:,count)=z;
        rR(:,:,count)=R;
    end
end
rKz=rKz(1:count);rz=rz(:,1:count);rR=rR(:,:,1:count);
rKz=rKz/sum(rKz);
%Prune or renormalize can be done also here
%[rKz,rz,rR]=PruneGMM(rKz,rz,rR,0.5);
end