function [o]=Overlap(s1,s2)
%numerically compute overlap for two ellipses s1 and s2
%s2 can be a bounding box [x_left_top yl_left_top w h]
%Author:Zoran Zivkovic

nPoints=20;
w=0:2*pi/nPoints:2*pi;
rCircle=2*[cos(w);sin(w)];%circle
nPoints=length(w);

K=[s1(3:4)'; 0 s1(5)]';
rPlot1=K*rCircle+s1(1:2)*ones(1,nPoints);
%plot(rPlot1(1,:),rPlot1(2,:),'k-');
min1=min(rPlot1,[],2);
max1=max(rPlot1,[],2);

if (length(s2)==4)
    %BBOX input
    min2=s2(1:2)';
    max2=min2+s2(3:4)';
else
K=[s2(3:4)'; 0 s2(5)]';
rPlot2=K*rCircle+s2(1:2)*ones(1,nPoints);
%hold on,plot(rPlot2(1,:),rPlot2(2,:),'k-');
min2=min(rPlot2,[],2);
max2=max(rPlot2,[],2);
end

%approximation using rectangles
aUnion=prod(max1-min1)+prod(max2-min2);
dOverlap=min([max1 max2],[],2)-max([min1 min2],[],2);
if ((dOverlap(1)<0)|(dOverlap(2)<0))
    o=0;
else
    aOverlap=prod(dOverlap);
    o=aOverlap/(aUnion-aOverlap);
end


%TODO: further refine


