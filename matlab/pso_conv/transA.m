function A = transA(u_p, u_g, phi_p, phi_g, chi)

A = [chi, - u_p * phi_p - u_g * phi_g; chi, 1 - u_p * phi_p - u_g * phi_g ]