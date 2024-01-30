%% Clean up
clc
close all
clear

%% Define Parameters
q0 = linspace(-40,40,101);

%Starting vlaues
l_cr0 = 200;
l_cr1 = 150;
phi = 8;
m0 = 0.5;
m1 = 0.1;
 
x0=[l_cr0, l_cr1, phi, m0, m1]; 

%% Function to fit
fit_fun = @T_g_fun;
fun = fit_fun(q0);

%% Coefficient approximation
x = lsqcurvefit(@M_cr, x0, q0, fun)

%% --------------Compare both functions-------------------
%Plot Gravity imposed torque
figure
plot(q0,fun,'.','MarkerSize',8)       %Gravity influence
box off
hold on
% Plot GC curve
f = @M_cr;
f_l = f(x,q0);
plot(q0, f_l, "Color","r","LineStyle","-")       %Spring moment
grid on
%Change appearance
xlabel('$q_1$\,/\,$\circ$','Interpreter','latex')
ylabel('Torque\,/\,Nmm','Interpreter','latex')

legend(["Gravitational torque $T$", "Gravitational counter torque $M_{cr}$"],'Interpreter','latex',"Location","southeast")
hold off
