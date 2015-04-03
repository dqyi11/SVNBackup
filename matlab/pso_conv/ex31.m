load('param2.mat');

figure(2)
h1 = scatter(stable(1,:), stable(2,:), 36, [0./255 51./255 102./255], 's'  );
hold on;
h2 = scatter(unstable(1,:), unstable(2,:), 36, [153./255 204./255 255./255], 's' );
h3 = legend('stable')
set(h1, 'SizeData', 36, 'MarkerFaceColor',[0./255 51./255 102./255]);
set(h2, 'SizeData', 36,'MarkerFaceColor',[153./255 204./255 255./255]);
set(h3, 'FontSize', 25,'FontWeight','bold');
set(gca,'FontSize', 16)

y = []
x1 = 0.001:0.01:1.0
for x = 0.001:0.01:1.0
    
    y = [y 2*(1+x)/x]
end
plot(x1,y,'r--','linewidth',4)
ylim([0,15])
xlabel('\chi','FontSize',30,'FontWeight','bold');
ylabel('\phi','FontSize',30,'FontWeight','bold');
