function h = Hexagon(x,y,d)
% Hexagon.m
% (x,y) is center of hexagon
% d is distance from center of hexagon to sides
% Michael A. Goodrich
% 17 July 2012
s = 2/sqrt(3)*d;
h=patch([x-d,x-d,x,x+d,x+d,x,x-d],[y-s/2,y+s/2,y+s,y+s/2,y-s/2,y-s,y-s/2],'w');
end