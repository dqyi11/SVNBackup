% Differential flatness example for ECE 774
%
% Modified: 4/15/2014 - R. Beard
%
%
function u = ctrl(uu,P)
    xr = uu(1:4);
    x  = uu(5:8);
    
    A = [...
        0, 0, -xr(4)*sin(xr(3)), cos(xr(3));...
        0, 0,  xr(4)*cos(xr(3)), sin(xr(3));...
        0, 0, 0, 0;...
        0, 0, 0, 0;...
        ];
    B = [0, 0; 0, 0; 1, 0; 0, 1];
    
    K = place(A,B,[-P.a+j*P.a,-P.a-j*P.a,-P.b+j*P.b,-P.b-j*P.b]);
    
    u = -K*(x-wrap(xr,x));
end

function out = wrap(in,x)
    out = in;
    while out(3)-x(3)>pi, out(3)=out(3)-2*pi; end
    while out(3)-x(3)<-pi, out(3) = out(3)+2*pi; end
end