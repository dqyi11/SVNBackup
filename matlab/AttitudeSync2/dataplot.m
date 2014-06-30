load('data1.mat');
load('data2.mat');
load('data3.mat');

figure();
plot(T1, errorMean1, T2, errorMean2, T3, errorMean3);
ax1 = legend('RAT', 'RAT - big K', 'RAT - big \Gamma ');
ylabel('error mean');
xlabel('time');
xlhand = get(gca,'xlabel');
ylhand = get(gca,'ylabel');
set(xlhand,'fontsize',15);
set(ylhand,'fontsize',15);
set(ax1,'fontsize',15);


figure();
plot(T1, errorVar1, T2, errorVar2, T3, errorVar3);
ax2 = legend('RAT', 'RAT - big K', 'RAT - big \Gamma ');
ylabel('error variance');
xlabel('time');
xlhand = get(gca,'xlabel');
ylhand = get(gca,'ylabel');
set(xlhand,'fontsize',15);
set(ylhand,'fontsize',15);
set(ax2,'fontsize',15);