function [rK,rs,rPk]=GMMCollapse(rK,rs,rPk,nModes)
%collapse the mixture by sampling from the multinomial distribution
%->Rao Blakwellized particle filter type of collapsing
%maximum of nModes components will remain
nModesNew=length(rK);
if(nModesNew>nModes)
    if (0)
        rT=ToCummulative(rK);rbInclude=zeros(1,nModesNew);
        for iModes=1:nModes
            %sample
            ri=find(rT>=rand(1));
            %if (iModes==1) ri=iMax;end; %always propagate the best one
            rbInclude(ri(1))=rbInclude(ri(1))+1;
        end
        riInclude=find(rbInclude>0);
        rs=rs(:,riInclude);
        rPk=rPk(:,:,riInclude);
        rK=rK(riInclude);
        rK=rK/sum(rK);%normalize
        %end
    else
        %take the largest ones
        [dummy,riInclude]=sort(-rK);
        riInclude=riInclude(1:nModes);
        %if (sum(riInclude==iMax)==0) riInclude(nModes)=iMax;end; %always propagate the best one
        rs=rs(:,riInclude);
        rPk=rPk(:,:,riInclude);
        rK=rK(riInclude);
        rK=rK/sum(rK);%normalize
    end
end