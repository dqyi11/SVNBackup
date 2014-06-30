function tau=arm_ctrl(in)
    xm      = in(1:2);
    e       = in(3:4);
    r       = in(5);
    t       = in(6);
    
    % define and initialize persistent variables

    tau = mrsd_ctrl(xm, e, r, t);
end

function tau = mrsd_ctrl(xm, e, r, t)
% Model reference sliding model control
    tau = 0;
    m_h = 1;
    m_l = 0.25;
    l_h = 0.5;
    l_l = 0.1;
    g = 9.8;
    b_h = 1;
    b_l = -1;
    
    epsilon = 0.5;
    beta0 = 30.0;
    K1 = 40.0;
    
    b1 = 7.2;
    b2 = 16;
    x = xm + e;
    
    a1_h = m_h * l_h^2 / 3;
    a2_h = b_h;
    a3_h = m_h * g * l_h / 2;
    beta =  - a1_h * abs(e(1)+K1*e(2)+b2*xm(1)+b1*xm(2)-b2*r) - a2_h * abs(x(2)) - a3_h * abs(cos(x(1))) - beta0;
    %beta = - a1_h * ( abs(e(1)) + K1 *abs(e(2)) + b2 * abs(xm(1)) + b1 * abs( xm(2)) + b2 * abs(r) ) - a2_h * abs(x(2)) - a3_h * abs(cos(x(1))) - beta0;
    z1 = e(2) + K1*e(1);
    %tau = sign(z1) * beta;
    tau = sat(z1, epsilon) * beta;
end

function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end