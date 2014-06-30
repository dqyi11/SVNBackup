phi1 = 2
phi2 = 2
chi = 0.8

phi = phi1 + phi2

histP = []
histQ = []

for p1 = 0.05:0.02:5
    for p2 = 0.05:0.02:5
        P = [p1, 0; 0, p2]
        for u = 0:0.1:1
            A = transA2(u, chi, phi)
            Q = P - A'* P * A
            if all(eig(Q) > 0)
                histP = [histP P]
                histQ = [histQ Q]
                break
                break
                break
            end
        end
    end
end

