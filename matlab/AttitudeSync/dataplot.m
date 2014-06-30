load('data1.mat');
load('data2.mat');
load('data3.mat');

figure();
plot(T1, errorMean1, T2, errorMean2, T3, errorMean3);
legend('CAS', 'CAS no damping', 'CAS \sigma');
title('root mean square');
xlabel('time');

figure();
plot(T1, errorVar1, T2, errorVar2, T3, errorVar3);
legend('CAS', 'CAS no damping', 'CAS \sigma');
title('variance');
xlabel('time');