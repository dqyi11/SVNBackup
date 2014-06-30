figure(2)
hold on 
[y, phi] = meshgrid([-2*pi:pi/8:2*pi],[-2*pi:pi/8:2*pi]);
[wy, hy] = size(y);
[wp, hp] = size(phi);
for i = 1:1:wy
    for j = 1:1:hy
        x0 = [y(i,j); phi(i,j)];
        [t,x]=ode45(@dxdt3,[0 2],x0);
        plot(x(:,1),x(:,2))
    end
end
