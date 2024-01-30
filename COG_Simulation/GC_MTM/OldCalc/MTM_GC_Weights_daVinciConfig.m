%% Clean up
clc
close all
clear

%% Define Parameters
alpha_d = linspace(-45,45,1000);
alpha_r = alpha_d .* (pi/180);

% Upper and lower boudaries
mu = -0.4; mo = 0.4;
lu = -6; lo = 100;
% Starting vlaues
m = 0.1;
l = -1;
% x0=[m]; 
x0=[m,l]; 

%% Function to fit
T_func = @T_gravity_daVinciConfig_MTM_ShoulderPitch;
T = T_func(alpha_r);

%% Coefficient approximation
x = lsqcurvefit(@MTM_Ms_Weights_daVinciConfig, x0, alpha_r, T, [mu, lu], [mo, lo])

%% Compute brass cylinder height
roh_brass= 8.73E-3;
r = 20;
mc= x(1);

h = abs(mc*1E3) / (r^2*pi*roh_brass) 
%% --------------Compare both functions-------------------
%Plot Gravity imposed torque
figure
plot(alpha_d,T)       %Gravity influence
hold on
% Plot GC curve
f = @MTM_Ms_Weights_daVinciConfig;
Ms = f(x,alpha_r);
plot(alpha_d, Ms, "Color","r","LineStyle","--")       %Spring moment
grid on
%Change appearance
xlabel("Angle alpha / degrees")
ylabel("Torque / Nmm")

legend("Gravity Torque", "Counter Tourque","Location","best")
hold off
