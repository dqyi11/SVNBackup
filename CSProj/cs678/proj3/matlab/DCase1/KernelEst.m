function y = KernelEst(C, kernelX, x)

n1 = length(kernelX);
n2 = length(x);

y = zeros(n2,1);

for i=1:1:n2
    for j=1:1:n1
        y(i,1) = y(i,1) + C(j,1) * exp(-(kernelX(j,1)-x(1,i))*(kernelX(j,1)-x(1,i)));
    end
end