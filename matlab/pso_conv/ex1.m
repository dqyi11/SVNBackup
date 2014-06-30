%phi_p = 2
%phi_g = 2
%chi = 0.8

%u_p = 0.5
%u_g = 0.5

stable = []
unstable = []
for chi = 0.0:0.02:1.0 
    for phi_p = 0.0:0.1:1.0
        for phi_g = 0.0:1:1.0
            
            maxEig = []
            for u = 0.0:0.005:1.0
                maxEig = [maxEig max(abs(eig(transA(u,u,phi_p,phi_g,chi))))]
            end
            val = max(maxEig)
            if val <= 1
                stable = [stable [chi; phi_p; phi_g; val]]
            else
                unstable = [unstable [chi; phi_p; phi_g; val]]
            end
        end
    end
end

scatter3(stable(1,:), stable(2,:), stable(3,:), 'r.')
hold on
scatter3(unstable(1,:), unstable(2,:), unstable(3,:), 'b.')
