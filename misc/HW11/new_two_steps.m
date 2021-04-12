%% 
clear;clc;

%% Define parameters
g = 9.8;
des = 200;
Zc = 0.8;
Tsup = 0.8;
Sx = 0.3;
Sy = 0.2;

Tc = sqrt(Zc/g);
Xbar = 0.5*Sx;
Ybar = 0.5*Sy;
Vx = (Xbar*(1+cosh(Tsup/Tc)))/(Tc*sinh(Tsup/Tc));
Vy = (Ybar*(cosh(Tsup/Tc)-1))/(Tc*sinh(Tsup/Tc));

t = linspace(0,Tsup,des);
x = [0];
y = [0];

%% first step
x_step = -Xbar*cosh(t/Tc)+Vx*Tc*sinh(t/Tc);
y_step = Ybar*cosh(t/Tc)-Vy*Tc*sinh(t/Tc);
x = [x,x_step];
y = [y,y_step];

%% second step
x_step = -Xbar*cosh(t/Tc)+Vx*Tc*sinh(t/Tc)+Sx;
y_step = -Ybar*cosh(t/Tc)+Vy*Tc*sinh(t/Tc)+Sy;
x = [x,x_step];
y = [y,y_step];

%% plot
x = x(2:length(x));
y = y(2:length(y));
plot(x,y);
axis equal;
