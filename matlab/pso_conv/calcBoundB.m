function boundB = calcBoundB(phi_p, phi_g , chi)

temp = []
for u = 0:0.05:1.0
    B = [chi * phi_g * u , chi * phi_p * u; chi * phi_g * u , chi * phi_p * u]
    temp = [temp max(abs(eig(B)))]
end

boundB = max(temp)