figure(2)
hold on 
for theta=[-10:10]/5
    x0=[theta 2];
    [t,x]=ode45(@dxdt2,[0 2],x0);
    plot(x(:,1),x(:,2))
    x0=[theta -2];
    [t,x]=ode45(@dxdt2,[0 2],x0);
    plot(x(:,1),x(:,2))
end
axis([-10 10 -2 2])