X = [0.8 0.4 0.2 0.1 0.05 0.02]
Y = [0.4404 0.2825 0.1645 0.0897 0.0469 0.0193]

plot(X, Y, '.-');
xlabel('smooth factor');
%set(gca,'XTickLabel',{'0.8' '0.4' '0.2' '0.1' '0.05' '0.02'}) 
ylabel('Means square error ratio');