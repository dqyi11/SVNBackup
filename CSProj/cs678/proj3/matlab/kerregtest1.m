testSize = 100;
lamda = 0.8;
ker = 'rbf';

noiseVar = 0.5;
testX = 2 * rand(testSize,1) - ones(testSize, 1);
testY = func1(testX);
testY = testY + noiseVar * randn(testSize,1);

C = KernelReg(testX, testY, lamda, ker);

X = -1:0.02:1;
X = X';
predY = KernelEst(C, ker, testX, X);
refY = func1(X);

plot(X, refY, '-b', X, predY, '-r');
xlabel('Test Data X');
ylabel('Test Data Y');


