varErr = zeros(20,1);
for i=1:1:20
    varErr(i,1) = var(error(i,:));
end

xIt = 1:1:20

plot(xIt,varErr);
xlabel('Epoch');
ylabel('Consensus(Variance)');