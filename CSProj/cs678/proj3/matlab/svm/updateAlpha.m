function alpha = updateAlpha (alpha, v, nv, eta)
   
B = size(nv,2);

for i=1:1:B
    alpha = alpha + (eta/2) * (v - nv(:,i));
end

end