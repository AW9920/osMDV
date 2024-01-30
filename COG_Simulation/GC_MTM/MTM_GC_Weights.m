%% Clean up
clc
close all
clear

%% Define Parameters
len = 81;

q1 = linspace(-45,45,len);
q2 = linspace(0,0,len);
q3 = linspace(0,0,len);

%Starting vlaues
m = 0.1;
l = 40;
phi = 0.1;

mu = m; mo = 0.4;
lu = l; lo = 80;
phiu=phi; phio = 10;
 
x0=[m,l,phi]; 

%% Function to fit
T_func = @T_gravity_MTM_ShoulderPitch;
T = T_func(q1,q2,q3,len);

%% Coefficient approximation
x = lsqcurvefit(@MTM_Ms_Weights, x0, q1, T,[mu,lu,phiu],[mo,lo,phio])

%% Compute brass cylinder height
roh_brass= 8.73E-3;
r = 20;
mc= x(1);

h = abs(mc*1E3) / (r^2*pi*roh_brass) 
%% --------------Compare both functions-------------------
%Plot Gravity imposed torque
figure
plot(q1,T,'.','MarkerSize',8)       %Gravity influence
xlim([-45 45])
hold on
% Plot GC curve
f = @MTM_Ms_Weights;
Ms = f(x,q1);
plot(q1, Ms, "Color","r","LineStyle","-")       %Spring moment
xlim([-45 45])
grid on
%Change appearance
xlabel('$q_1$\,/\,$\circ$','Interpreter','latex')
ylabel('$Torque$\,/\,Nmm','Interpreter','latex')

legend(["Gravitational torque $T$", "Gravitational counter torque $M_{cr}$"],'Interpreter','latex',"Location","southeast")
hold off
