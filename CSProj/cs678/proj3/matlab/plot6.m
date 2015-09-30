X1 = [0.0500 0.075 0.075 0.2251 0.25 0.275 0.325 0.4 0.425];
X2 = [0.1175 0.1850 0.2250 0.3925 0.425 0.56 0.68 0.7 0.835];
L = [0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9];
plot(L,X1,'-r' , L, X2, '-b');
legend('K-nearest neighbor: k=2', 'connectivity-average');
xlabel('Connectivity Decrease');
ylabel('Mean error ratio');