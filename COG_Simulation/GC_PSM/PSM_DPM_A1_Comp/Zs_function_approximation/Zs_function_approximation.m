%% Clean up
clc
close all
clear

%% Define Parameters
len = 101;
alpha = linspace(-40,40,len);
%Starting vlaues
l = 50;
a = 0.1;
 
x0=[l, a]; 

%% Function to fit
base_fun = @Z_s_Calc;
base = base_fun(alpha,round(len/2));

%% Coefficient approximation
x = lsqcurvefit(@zs_fitfun, x0, alpha, base)

%% --------------Compare both functions-------------------
%Plot Gravity imposed torque
figure
plot(alpha,base,'.','MarkerSize',8)       %Calculated zs movement of COG
hold on
box off
% Plot GC curve
f = @zs_fitfun;
f_l = f(x,alpha);
plot(alpha, f_l, "Color","r","LineStyle","-")       %Spring moment
grid on
%Change appearance
xlabel('$q_2$\,/\,$\circ$','Interpreter','latex')
ylabel('$z_s$\,/\,mm', 'Interpreter','latex')

legend(["Dataset of Z-components of DPM COG", "Approcimated function"],'Interpreter','latex',"Location","southeast")
hold off
