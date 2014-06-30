function A = transA2(u, chi, phi)
A = [chi, - chi * phi * u; chi, 1 - chi * phi * u]