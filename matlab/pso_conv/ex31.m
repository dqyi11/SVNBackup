load('param2.mat');

figure(2)
h1 = scatter(stable(1,:), stable(2,:),  'rs');
hold on;
h2 = scatter(unstable(1,:), unstable(2,:), 'bs');
h3 = legend('stable')
set(h1, 'SizeData', 36, 'MarkerFaceColor','r');
set(h2, 'SizeData', 36,'MarkerFaceColor','b');
set(h3, 'FontSize',14,'FontWeight','bold');

y = []
x1 = 0.001:0.01:1.0
for x = 0.001:0.01:1.0
    
    y = [y 2*(1+x)/x]
end
plot(x1,y,'y-','linewidth',6)
ylim([0,15])
xlabel('CHI','FontSize',14,'FontWeight','bold');
ylabel('PHI','FontSize',14,'FontWeight','bold');
