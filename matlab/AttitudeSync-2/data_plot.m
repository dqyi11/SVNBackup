load('high-damping.mat');

T = D1(:,1);
Ref = D1(:,8);
DataLen = size(T);

D1Error1 = D1(:,2) - D1(:,8);
D1Error2 = D1(:,3) - D1(:,8);
D1Error3 = D1(:,4) - D1(:,8);
D1Error4 = D1(:,5) - D1(:,8);
D1Error5 = D1(:,6) - D1(:,8);
D1Error6 = D1(:,7) - D1(:,8);

D2Error1 = D2(:,2) - D2(:,8);
D2Error2 = D2(:,3) - D2(:,8);
D2Error3 = D2(:,4) - D2(:,8);
D2Error4 = D2(:,5) - D2(:,8);
D2Error5 = D2(:,6) - D2(:,8);
D2Error6 = D2(:,7) - D2(:,8);

D3Error1 = D3(:,2) - D3(:,8);
D3Error2 = D3(:,3) - D3(:,8);
D3Error3 = D3(:,4) - D3(:,8);
D3Error4 = D3(:,5) - D3(:,8);
D3Error5 = D3(:,6) - D3(:,8);
D3Error6 = D3(:,7) - D3(:,8);

D1errorVar = [];
D1errorMean = [];
D2errorVar = [];
D2errorMean = [];
D3errorVar = [];
D3errorMean = [];

errorMean = [];
errorVar = []


for i=1:1:DataLen
    
    e_1 = [D1Error1(i) D2Error1(i) D3Error1(i)];
    e_2 = [D1Error2(i) D2Error2(i) D3Error2(i)];
    e_3 = [D1Error3(i) D2Error3(i) D3Error3(i)];
    e_4 = [D1Error4(i) D2Error4(i) D3Error4(i)];
    e_5 = [D1Error5(i) D2Error5(i) D3Error5(i)];
    e_6 = [D1Error6(i) D2Error6(i) D3Error6(i)];
    
    ee = [e_1;e_2;e_3;e_4;e_5;e_6];
    ee = abs(ee);
    
    errorMean = [errorMean; mean(mean(ee))];
    errorVar = [errorVar; var(var(ee))];
end

figure;
plot(T, errorMean);
title('root square mean error');
xlabel('Time');

figure;
plot(T, errorVar);
xlabel('Time');
title('error variance');