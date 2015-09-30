function y = func2(x)
w = size(x,1);
y = zeros(w,1);

bumpCenters = [0.4 -0.3; 0.8, 0.2; -0.1 -0.5];
bumpVars = [2, 3, 1.5];

tempY = zeros(w,1);
for i = 1:1:w
    for j = 1:1:3
        tempY(i,1) = ((x(i,1)-bumpCenters(1,1))^2 + (x(i,2)-bumpCenters(1,2))^2)/ bumpVars(j)^2;
        tempY(i,1) = exp(-tempY(i,1));
        y(i,1) = y(i,1) + tempY(i,1);
    end
end

