%% zou liang bu, mei * zou liang bu
clear;clc;

%% Define parameters
% constants
g = 9.8;
% pre-setted
des = 200;
STEP = 4;
Zc = 0.8;
Tsup = 0.8;
Sx = 0.3;
Sy = 0.2;

%% Calculate parameters
t = linspace(-Sx*0.5,Sx*(STEP-0.5),STEP*des);
Tc = sqrt(g/Zc);
X_bar = zeros(STEP,1);
Y_bar = Sy*0.5*ones(STEP,1);
Vx = zeros(STEP,1);
Vy = zeros(STEP,1);
for i=1:STEP
    X_bar(i) = Sx*(i-1.5);
    Vx(i) = (0.5*Sx*(1+cosh(Tsup/Tc)))/(Tc*sinh(Tsup/Tc));
    Vy(i) = (Y_bar(i)*(cosh(Tsup/Tc)-1))/(Tc*sinh(Tsup/Tc));
end

X_bar
Y_bar
Vx
Vy

%% 
x = [0];
y = [0];
for i=1:STEP
    % t_i = t(((i-1)*des+1):i*des);
    t_i = linspace(0,Tsup,des);
    x_i = Sx*0.5*cosh(t_i/Tc) + Vx(i)*Tc*sinh(t_i/Tc);
    y_i = Y_bar(i)*cosh(t_i/Tc) + Vy(i)*Tc*sinh(t_i/Tc);
    x = [x, x_i];
    y = [y, y_i];
end
x = x(2:length(x));
y = y(2:length(y));

plot(x,y);
% axis equal;