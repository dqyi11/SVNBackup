function tau=arm_ctrl(in)
    xm      = in(1:2);
    x       = in(3:4);
    r       = in(5)
    t       = in(6);
    a1      = in(7);
    a2      = in(8);
    a3      = in(9);
    
    % define and initialize persistent variables

    tau = mrac_ctrl(xm, x, r, t, a1, a2, a3);
end

function tau = mrac_ctrl(xm, x, r, t, a1, a2, a3)
% Model reference adaptive control
    tau = 0;
    K1 = 40.0;
    K2 = 40.0;
    %a1 = 0.5 * 0.3 * 0.3 / 3;
    a2 = 0.01;
    a3 = 0.5 * 9.8 * 0.3 / 2;
    b1 = 7.2;
    b2 = 16;
    e = x - xm;
    z1 = e(2) + K1 * e(1);
    bx1 = (e(1)+K1*e(2)+b2*xm(1)+b1*xm(2)-b2*r);
    bx2 = x(2);
    bx3 = cos(x(1));
    tau = a1 * bx1 + a2 * bx2 + a3 * bx3 - K2 * z1;
end

function out = sat(in,limit)
    if     in > limit,      out = limit;
    elseif in < -limit,     out = -limit;
    else                    out = in;
    end
end