%EM-shift demo
%a simple color-histogram tracking demo script including various trackers
%from:
%
%Z. Zivkovic , A. Cemgil, B. Kröse
%"Approximate Bayesian methods for kernel-based object tracking"
%Computer Vision Image Understanding, vol. 113, pages 743-749, 2009
%
%University of Amsterdam, The Netherlands
%Author:Zoran Zivkovic, www.zoranz.net
%Date: 17-5-2005 - updated 20-10-2010
clear
sAvi='J:\tracking_video.avi';%set your own avi file name
nNImageStart=40;nNImageEnd=120;%select frames

%% choose the tracker
%D. Comaniciu, V. Ramesh, P. Meer
%Kernel-Based Object Tracking,
%IEEE Trans. Pattern Analysis Machine Intell., Vol. 25, No. 5, 564-575, 2003
rsTracker{1}='MS';
dScale=0.1;%used to adapt scale - +-dScale is tried as in the paper
%Z.Zivkovic, B.Krose
%An EM-like algorithm for color-histogram-based object tracking"
%IEEE Conference CVPR, June, 2004
rsTracker{2}='EMS';
beta=1.1;%EM shift parameters
nIterationMax=5;

%Mixture Kalman Filter from
%Z. Zivkovic , A. Cemgil, B. Kröse
%"Approximate Bayesian methods for kernel-based object tracking"
%Computer Vision Image Understanding, vol. 113, pages 743-749, 2009
rsTracker{3}='MKF';
nModes=5;%maximum number of modes in the MKF
alpha=0.9;%adding random noise to increase chance of recovering from occlusion alpha*GMM+(1-alpha)*uniformn
%Similar to (only scaling and no skew of the ellipse was used in the original paper):
%K. Nummiaro, E. Koller-Meier, L.J. Van Gool
%An adaptive color-based particle filter. Image Vision Comput. 21(1): 99-110 (2003)
rsTracker{4}='PF';
nParticles=100;

%select tracker
sTracker=rsTracker{1};
disp(['tracker: ' sTracker])


%%%%%%%%%%%%%%%%%%%
%Initialization
%%%%%%%%%%%%%%%%%%%
%manually select object
rObj=mmreader(sAvi);%new matlab api
%data=aviread(sAvi,nNImageStart);im0=data.cdata; %old malab interface
im0=read(rObj,nNImageStart);
figure(1),imMask=roipoly(im0);%roipoly is from the Image processing toolbox - use create mask option in the new matlab versions
nImSize=[size(im0,2);size(im0,1)];%x,y!!!
%shape=>ellipse
[ry,rx]=find(imMask);m0=mean([rx,ry])';V0=cov([rx,ry]);
[K]=GaussKernel(V0);%make kernel of appropriate size
%make object histogram
data=GetAffineRegion(im0,m0,V0,K.rX,K.rY,K.sigmas);
nBinsD=4;histO=ColorHist(data,nBinsD,K.rK);
%state vector
s0=MeanVarToS(m0,V0);nStates=length(s0);s=s0;
%dynamic model s=A*s+W
A=eye(nStates);
W=[3^2  0   0   0   0;
    0   3^2 0   0   0;
    0   0   1^2 0   0;
    0   0   0   1^2 0;
    0   0   0   0   1^2];
lambda=0.2;%observation model p~exp(-0.5*(1-rho)/lambda^2)
Pk=10^2*eye(nStates);%some high initial value for the variance

%define some bodrer limits for clipping - used to limit the random sampling
%steps in SampleGMM
nBorder=5;
rLimits=[nBorder nImSize(1)-nBorder;... 
    nBorder nImSize(2)-nBorder;...
    3 mean(nImSize)/3;...
    -10, 10;...
    3 mean(nImSize)/3];


%MKF init
rK=1;
rs=s0;
rPk(:,:,1)=Pk;%initial uncertainty

%Particle filter init
rPi=ones(1,nParticles)/nParticles;%uniformn
rPiCummulative=1:nParticles;%uniformn
rParticles=repmat(s0,1,nParticles);%+0.5*sqrt(W)*randn(nStates,nParticles);%(assume W diagonal)

%%%%%%%%%%%%%%%%%%%
%Tracking
%%%%%%%%%%%%%%%%%%%
for iFrame=nNImageStart:nNImageEnd
    im0=read(rObj,iFrame);
   % data=aviread(sAvi,iFrame);im0=data.cdata;%read an image %old malab interface
    %different trackers
    switch sTracker
        case 'EMS'
            %Z.Zivkovic, B.Krose
            %An EM-like algorithm for color-histogram-based object tracking"
            %IEEE Conference CVPR, June, 2004
            %predict
            s=A*s;
            Pk=A*Pk*A'+W;
            %find max
            [xt,V]=SToMeanVar(s);%prediction as starting point
            for iIteration=1:nIterationMax
                [xt,V]=EMShift(im0,xt,V,histO,K,beta);
            end
            z=MeanVarToS(xt,V);
            R=FitGauss(im0,z,histO,K)*lambda^2;
            %Kalman filter
            KFgain=Pk*inv(Pk+R);
            s=s+KFgain*(z-s);
            Pk=(eye(nStates)-KFgain)*Pk;
        case 'MS'
            %D. Comaniciu, V. Ramesh, P. Meer
            %Kernel-Based Object Tracking,
            %IEEE Trans. Pattern Analysis Machine Intell., Vol. 25, No. 5, 564-575, 2003
            %predict
            s=A*s;
            Pk=A*Pk*A'+W;
            %Find the local max using the prediction as the start point
            [xt,V]=SToMeanVar(s);%prediction as starting point
            for iIteration=1:nIterationMax
                xt=EMShift(im0,xt,V,histO,K,beta);%mean shift does not use the second moment
            end
            rho0=Similarity(im0,xt,V,histO,K);
            z0=MeanVarToS(xt,V);
            %measure V by trying a number of different scales
            rD=dScale*[0 0 0 0 0;0 0 s(3) 0 s(5); 0 0 -s(3) 0 -s(5)]';
            %try bigger
            [xt,V]=SToMeanVar(s+rD(:,2));
            for iIteration=1:nIterationMax
                xt=EMShift(im0,xt,V,histO,K,beta);%mean shift does not use the second moment
            end
            rho1=Similarity(im0,xt,V,histO,K);
            z1=MeanVarToS(xt,V);
            %try smaller
            [xt,V]=SToMeanVar(s+rD(:,3));
            for iIteration=1:nIterationMax
                xt=EMShift(im0,xt,V,histO,K,beta);%mean shift does not use the second moment
            end
            rho2=Similarity(im0,xt,V,histO,K);
            z2=MeanVarToS(xt,V);
            %choose the best
            [dummy,ri]=max([rho0 rho1 rho2]);
            if (ri(1)==1) z=z0; end;
            if (ri(1)==2) z=z1; end;
            if (ri(1)==3) z=z2; end;
            
            R=FitGauss(im0,z,histO,K)*lambda^2;
            %since we can be far from a local minima for the V
            %the local curvature is not good estimate for the V uncertainty !!! 
            %-use some fixed value - mean value from the EMS for example
            R(3,3)=1;R(4,4)=1;R(5,5)=1;
            %Kalman filter equations
            KFgain=Pk*inv(Pk+R);
            s=s+KFgain*(z-s);
            Pk=(eye(nStates)-KFgain)*Pk;
		case 'MKF'
			%Mixture Kalman Filter from
			%Z. Zivkovic , A. Cemgil, B. Kröse
			%"Approximate Bayesian methods for kernel-based object tracking"
			%Computer Vision Image Understanding, vol. 113, pages 743-749, 2009
			 %%%%%%%%%%%%%%%%%%
            %STEP1: Prediction
            %%%%%%%%%%%%%%%%%%
            nModesCurrent=length(rK);
            rKPred=rK;
            rsPred=A*rs;
            rPkPred=rPk;
            for iS=1:nModesCurrent
                rPkPred(:,:,iS)=A*rPk(:,:,iS)*A'+W;
            end
            
            %%%%%%%%%%%%%%%%%%
            %STEP2: Approximate the observation model
            %%%%%%%%%%%%%%%%%%
            %sample from the prediction
            rSamples=SampleGMM(rKPred,rsPred,rPkPred,nModes-1,rLimits,alpha);%sample from the prediction
            rSamples=[s rSamples];%always propagate the previous best one
            %local search + local approximation
            [rKz,rz,rR]=GMMApprox(rSamples,im0,histO,K,beta,lambda);
            
            %%%%%%%%%%%%%%%%%%
            %STEP3: Combine the observation with the prediction
            %%%%%%%%%%%%%%%%%%
            %posterior - total number of filters nNew*nOld
            nModesNew=nModesCurrent*length(rKz);
            rK=zeros(1,nModesNew);rs=zeros(nStates,nModesNew);
            rPk=zeros(nStates,nStates,nModesNew);
            count=0;
            for iModes=1:nModesCurrent
                for iModesZ=1:length(rKz)
                    count=count+1;
                    rK(count)=rKPred(iModes)*rKz(iModesZ);
                    %Kalman filter
                    Pk=rPkPred(:,:,iModes);
                    st=rsPred(:,iModes);
                    KFgain=Pk*inv(Pk+rR(:,:,iModesZ));
                    rs(:,count)=st+KFgain*(rz(:,iModesZ)-st);
                    rPk(:,:,count)=(eye(nStates)-KFgain)*Pk;
                end
            end
            rK=rK/sum(rK);%normalize
            %choose the best one
            [s,val,iMax]=MaxGMM(rK,rs,rPk);
            
            %%%%%%%%%%%%%%%%%%
            %STEP4: Approximate the new estimate with a GMM with a fixed
            %number of components
            %%%%%%%%%%%%%%%%%%
            %collapse the mixture to a mixture with a fixed no. components
            [rK,rs,rPk]=GMMCollapse(rK,rs,rPk,nModes);
        case 'PF'
            %Particle filter similar to (only scaling and no skew of the ellipse was used in the original paper):
            %K. Nummiaro, E. Koller-Meier, L.J. Van Gool
            %An adaptive color-based particle filter. 
            %Image Vision Comput. 21(1): 99-110 (2003)
            %%%%%%%%%%%%%%%%%%
            %STEP1: Prediction
            %%%%%%%%%%%%%%%%%%
            %deterministic drift
            %rParticles=A*rParticles;
            %+ blurr =>prediction: rPi,rParticles,W 
            
            %%%%%%%%%%%%%%%%%%
            %STEP2: Approximate the posterior
            %posterior=prior*observation
            %sample from the prior and determine weights using the
            %observation model
            %%%%%%%%%%%%%%%%%%
            rParticles=SampleGMM(rPi,rParticles,W,nParticles,rLimits);
            for i=1:nParticles
                [xt,V]=SToMeanVar(rParticles(:,i));
                rRho=Similarity(im0,xt,V,histO,K);
                %rRho=SimilarityS(im0,rParticles(:,i),histO,K);
                rPi(i)=exp(-0.5*(1-rRho)/lambda^2);
            end
            rPi=rPi/sum(rPi);%normalize
            s=sum(repmat(rPi,[],nStates).*rParticles(:,:),2);%Mean of the posterior
    end

    %show
    figure(1),imshow(im0);hold on;%imshow is from the Image processing toolbox
	switch sTracker
        case 'MKF'
			%show modes if any 
			for iModes=1:length(rK)
				h=PlotEllipse(rs(:,iModes));
				set(h,'LineWidth',1,'Color',[0 0 0],'LineStyle','-');
			end
	end
		
    h=PlotEllipse(s);set(h,'LineWidth',4,'Color',[1 0.7 1],'LineStyle','--');
    drawnow;hold off
end