function y = KernelEst(C, kernelX, x)

n1 = length(kernelX);
[n2 h] = size(x);

y = zeros(n2,1);

for i=1:1:n2
    for j=1:1:n1
        dist = (kernelX(j,1)-x(i,1))^2 + (kernelX(j,2)-x(i,2))^2;
        y(i,1) = y(i,1) + C(j,1) * exp(-dist);
    end
end