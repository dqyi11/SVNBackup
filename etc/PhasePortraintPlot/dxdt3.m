function d=dxdt3(t,x)

tmp = - 1/tan(x(1)) - sin(x(2));
if tmp >= 1
    tmp = 1;
elseif tmp <= -1
    tmp = -1;
end

d = [sin(x(2)); tmp ];

