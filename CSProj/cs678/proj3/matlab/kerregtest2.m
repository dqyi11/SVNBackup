testSize = 20;
lamda = 2;
ker = 'rbf';

noiseVar = 0.5;
testX = zeros(testSize,2);
testX(:,1) = 2 * rand(testSize,1) - ones(testSize,1);
testX(:,2) = 2 * rand(testSize,1) - ones(testSize,1);

testY = func2(testX);
testY = testY + noiseVar * randn(testSize,1);

plot3(testX(:,1),testX(:,2),testY, '.');

grid on;
%mesh(testX(:,1), testX(:,2), testY);
%view(-1,1);
%box on;

x1 = -1:0.1:1;
x2 = -1:0.1:1;
size1 = length(x1);
size2 = length(x2);
[x1,x2] = meshgrid(x1, x2);

vizY = zeros(size1,size2);

for i= 1:1:size1
    for j = 1:1:size2
        vizX = [x1(i,j) x2(i,j)];
        vizY(i,j) = func2(vizX);
    end
end

hold on;

mesh(x1, x2, vizY);

hold on;

mesh(x1, x2, vizY);


C = KernelReg(testX, testY, lamda, ker);

EstY = zeros(size1,size2);
for i=1:1:size1
    for j=1:1:size2
        EstX = [x1(i,j) x2(i,j)];
        EstY(i,j) = KernelEst(C, ker, testX, EstX);
    end
end


figure;
mesh(x1,x2,vizY);
hold on;
mesh(x1,x2,EstY);

err = vizY - EstY;

errM = mean(mean(err));
errV = var(var(err));

errM
errV

