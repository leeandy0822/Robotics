

yprime = -2*y + sin(2*t);
y0 = -5:5; 
tspan = [0 1];
[t,y] = ode45(@(t,y) yprime,tspan,y0);plot(t,y)
grid on
xlabel('t')
ylabel('y')
title('Solutions of y'' = -2y + 2 cos(t) sin(2t), y(0) = -5,-4,...,4,5','interpreter','latex')