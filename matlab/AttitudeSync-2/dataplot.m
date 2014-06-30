load('data4.mat');
load('data5.mat');

figure();
plot(T4, errorMean4, T5, errorMean5);
ax1 = legend('CAS - high sync', 'CAS - high damping');
ylabel('error mean');
xlabel('time');
xlhand = get(gca,'xlabel');
ylhand = get(gca,'ylabel');
set(xlhand,'fontsize',15);
set(ylhand,'fontsize',15);
set(ax1,'fontsize',15);


figure();
plot(T4, errorVar4, T5, errorVar5);
ax2 = legend('CAS - high sync', 'CAS - high damping');
ylabel('error variance');
xlabel('time');
xlhand = get(gca,'xlabel');
ylhand = get(gca,'ylabel');
set(xlhand,'fontsize',15);
set(ylhand,'fontsize',15);
set(ax2,'fontsize',15);