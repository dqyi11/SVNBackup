function C = KernelRegIt(testX, testY, lamda, ker, C)

n = size(testX,1);

K = zeros(n,n);

for i=1:1:n
    for j=1:1:n
        K(i,j) = Kernel(ker, testX(i,:), testX(j,:));
    end
end

C = inv(K+lamda*eye(n))*(testY+lamda * C);

end