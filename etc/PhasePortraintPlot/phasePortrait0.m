figure(1)
hold on 
for theta=[0:10]*pi/5
    x0=1e-5*[cos(theta);sin(theta)];
    [t,x]=ode45(@dxdt1,[0 8],x0);
    plot(x(:,1),x(:,2))
end