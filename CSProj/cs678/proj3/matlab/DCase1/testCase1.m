agentNum = 10;
%Connection = int8(rand(agentNum, agentNum));
Connection = eye(agentNum);
Connection(1,2) = 1;
Connection(2,1) = 1;

testSize = 100;
lamda = 0.8;

noiseVar = 0.5;
testX = 2 * rand(testSize,1) - ones(testSize, 1);
testY = func1(testX);
testY = testY + noiseVar * randn(testSize,1);

C = KernelReg(testX, testY, lamda);

X = -1:0.02:1
Y = KernelEst(C, testX, X);
refY = func1(X);

plot(testX, testY, '.', X, Y, '-r');
xlabel('Test Data X');
ylabel('Test Data Y');

figure;
plot(X, Y, '-r', X, refY, '-b');
legend('Estimated', 'Origin');

err = Y - refY';

errM = mean(err);
errV = var(err);

errM
errV

