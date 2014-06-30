function boundA = calcBoundA(phi, chi)

temp = []
for u = 0:0.05:1.0
    A = [chi, - chi * phi * u; chi, 1 - chi * phi * u]
    temp = [temp max(abs(eig(A)))]
end

boundA = max(temp)