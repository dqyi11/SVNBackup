phi_p = 1
phi_g = 1
chi = 0.7

maxeig = []

for u = 0:0.01:1
%u = 1
%max(abs(eig(transA(u,u,phi_p,phi_g,chi))))
    maxeig = [ maxeig max(abs(eig(transA(u,u,phi_p,phi_g,chi))))]
end

max(maxeig)