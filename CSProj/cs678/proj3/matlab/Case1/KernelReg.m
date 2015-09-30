function C = KernelReg(testX, testY, lamda)
%lamda - smooth factor

n = length(testY);

K = zeros(n,n);

for i = 1:1:n
    for j = 1:1:n
        K(i,j) = exp(-(testX(i,1)-testX(j,1))'*(testX(i,1)-testX(j,1)));
    end
end

C = inv(K+lamda*eye(n)) * testY;