function y = func3(x)
w = size(x,1);
y = zeros(w,1);

bumpCenters = [4 -3; 8, 2; -1 -5; -7 6];
bumpVars = [2, 3, 1.5 4];
bumpHeight = [2 3 4 5 ];

tempY = zeros(w,1);
for i = 1:1:w
    for j = 1:1:4
        tempY(i,1) = ((x(i,1)-bumpCenters(j,1))^2 + (x(i,2)-bumpCenters(j,2))^2)/ bumpVars(j)^2;
        tempY(i,1) = bumpHeight(j) * exp(-tempY(i,1));
        y(i,1) = y(i,1) + tempY(i,1);
    end
end