
stable = []
unstable = []
for chi = 0.01:0.01:0.99 
    for phi = 0.1:0.1:15.0
        maxEig = []
        for u = 0.0:0.005:1.0
            maxEig = [maxEig max(abs(eig(transA2(u, chi, phi))))]
        end
        val = max(maxEig)
        if val <= 1
            stable = [stable [chi; phi; val]]
        else
            unstable = [unstable [chi; phi; val]]
        end
        
    end
end

scatter(stable(1,:), stable(2,:),  'rs')
hold on
scatter(unstable(1,:), unstable(2,:),  'bs')