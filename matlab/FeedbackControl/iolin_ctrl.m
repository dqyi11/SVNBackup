function tau=iolin_ctrl(in)
    x       = in(1:3);
    r       = in(4);
    
    % define and initialize persistent variables
    g = 9.81;
    k = 0.001;
    L0 = 0.01;
    L1 = 0.02;
    R = 1;
    m = 0.1;
    a = 0.05;
    
    z1 = x(1);
    z2 = x(2);
    z3 = g - k*x(2)/m - L0*a*x(3)^2/(2*m*(a+x(1))^2);
    
    K1 = 500;
    K2 = 200;
    K3 = 100;

    tau = R*x(3) - L0*a*x(2)*x(3)/((a+x(1))^2) + L1*x(1)*x(2)*x(3)/(a+x(1)) - (m*L1*x(1)*(a+x(1))^2 / (L0*a*x(3)))*(-K1*(z1-r)-K2*z2-K3*z3);
end
