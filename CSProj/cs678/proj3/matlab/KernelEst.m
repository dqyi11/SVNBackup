function y = KernelEst(C, ker, sv, x)

n = size(x,1);
svn = size(sv,1);
y = zeros(n,1);

for i=1:1:n   
    for j=1:1:svn
         y(i) =  y(i) + C(j) * Kernel(ker, sv(j,:), x(i,:));     
    end
end

end