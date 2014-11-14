function h=PlotEllipse(s1)
%function h=PlotEllipse(s)
%plot ellipse s=[xpos,ypos, xscale, shear, yscale]
%returns handle h to the object
%
%University of Amsterdam, The Netherlands
%Author:Zoran Zivkovic, www.zoranz.net
%Date: 17-5-2005

nPoints=20;
w=0:2*pi/nPoints:2*pi;
rCircle=2*[cos(w);sin(w)];%circle
nPoints=length(w);
K=[s1(3:4)'; 0 s1(5)]';
rPlot1=K*rCircle+s1(1:2)*ones(1,nPoints);
h=plot(rPlot1(1,:),rPlot1(2,:),'k-');