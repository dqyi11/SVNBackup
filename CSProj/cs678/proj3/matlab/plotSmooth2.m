X = [0.8 0.4 0.2 0.1 0.05 0.02]
Y = [0.9642 0.6806 0.3396 0.2145 0.1778 0.1667]

plot(X, Y, '.-');
title('smooth factor = 0.2');
xlabel('Noise Variance');
ylabel('Means square error ratio');