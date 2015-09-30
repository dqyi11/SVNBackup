function C = KernelReg(testX, testY, lamda)
%lamda - smooth factor

n = length(testY);

K = zeros(n,n);

for i = 1:1:n
    for j = 1:1:n
        dist = (testX(i,1)-testX(j,1))^2 + (testX(i,2)-testX(j,2))^2;
        K(i,j) = exp(-dist);
    end
end

C = inv(K+lamda*eye(n)) * testY;