function [x,y] = Hexagonindex2Center(i,j,d)
% Hexagon.m
% i,j is the index of the hexagon.  I'm using the first row as i=0, second
% as i=2, third as i=3, etc.  Similarly for the first column.
% d is distance from center of hexagon to sides
% Michael A. Goodrich
% 17 July 2012
if (rem(i,2)==0) && (rem(j,2)~=0)
    ME = MException('VerifyInput:OutOfBounds',...
        'Input indices must both be even or both odd in Hexagonindex2Center');
    throw(ME);
    %fprintf(1,'In Hexagonindex2Center, indices must both be even or odd\nExiting');
    
end;
s = 2/sqrt(3)*d;
% All columns are aligned to indices
x = i*d;
if rem(j,2) == 0
    y = j/2*d+j*s;
else
    y = j*s + j*d/2;
end

