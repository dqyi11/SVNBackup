i = 15;
xdata = 1:1:20
plot(xdata, errorRef, 'b', xdata, error(i,:),'r');
ylabel('mean error ratio');
xlabel('iteration number');
legend('Non-consensus based', 'Consensus based');