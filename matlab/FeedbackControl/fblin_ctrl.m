function tau=iolin_ctrl(in)

    x      = in(1:3);
    e      = in(4:6);
    r      = in(7:10);
    
    % define and initialize persistent variables
    g = 9.81;
    k = 0.001;
    L0 = 0.01;
    L1 = 0.02;
    R = 1;
    m = 0.1; 
    a = 0.05;
    
    K1 = 10;
    K2 = 5;
    K3 = 5;
    
    %dotx2 = g-k*x(2)/m-L0*a*(x(3)^2)/(2*m*((a+x(1))^2));
    
    %c = L0*a*x(3)/(-L1*x(1)*m*((a+x(1))^2));
    %alpha = -(k/m)*dotx2 + L0*a*x(2)*(x(3)^2)/(m*(a+x(1))^3) - r(3);
    %delta = R*x(3) - L0*a*x(2)*x(3)/((a+x(1))^2);

    %tau = -delta + ( - alpha - K1*e(1) - K2*e(2) - K3*e(3) )/c;
    
    %z1 = x(1);
    %z2 = x(2);
    %z3 = g - k*x(2)/m - ;
    
    K1 = 20;
    K2 = 10;
    K3 = 10;
    
    sig = -K1*e(1) - K2 * e(2) - K3 * e(3);
    alpha1 = (k/m) * (g - (k/m)*x(2) - ((L0*a)/(2*m))*(x(3)^2/(a+x(1))^2));
    alpha2 = - ((L0*a)/m) * ((x(2)*x(3)^2)/(a+x(1))^3);
    alpha = alpha1 + alpha2 + r(4);
    if x(3) == 0.0
        c = 0;
    else
        c =  - ((m*L1)/(L0*a)) * ((x(1)*(a+x(1))^2)/(x(3)));
    end
    delta = R * x(3) - ( (L0*a*x(2)*x(3)) / (a+x(1))^2 );
    tau = c*(sig + alpha) + delta;
    
    %tau = 0;
    %tau = R*x(3)- L0*a*x(2)*x(3)/((a+x(1))^2) + L1*x(1)*x(2)*x(3)/(a+x(1)) - (m*L1*x(1)*(a+x(1))^2 / (L0*a*x(3)))*(-K1*(z1-r(1))-K2*z2-K3*z3);
    
end
