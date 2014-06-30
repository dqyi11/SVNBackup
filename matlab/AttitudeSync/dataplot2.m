load('csa-normal.mat');

figure();
plot(D1(:,1),D1(:,2),D1(:,1),D1(:,4),D1(:,1),D1(:,6),D1(:,1),D1(:,8),'-.');

ax = legend('agent 1', 'agent 3', 'agent 5', 'reference');
xlabel('time','fontsize',12);
ylabel('\sigma(1)','fontsize',12);
ylim([0,2.2]);
set(ax, 'fontsize',12);