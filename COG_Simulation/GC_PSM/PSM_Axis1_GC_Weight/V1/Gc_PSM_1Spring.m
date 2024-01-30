%% Clean up
clc
close all
clear

%% Define Parameters
alpha_d = linspace(-35,35,1000);
alpha_r = alpha_d .* (pi/180);
% Global variables
L0 = 31;
d = 110;
Hu = 300; Hl=L0;
ku = 2; kl = 0;
%Starting vlaues
H = 150;
k0 = 0.8;

% x0=[d H k0];   %Units [mm mm N/mm]
x0=[H k0];       %Units [mm N/mm]
% x0=[k0];       %Units [N/mm]

%% Function to fit
T_func = @T_gravity;
T = T_func(alpha_r);

%% Coefficient approximation
x = lsqcurvefit(@Ms_1Spring, x0, alpha_r, T, [Hl,kl],[Hu,ku])

%% --------------Compare both functions-------------------
%Plot Gravity imposed torque
figure
plot(alpha_d,T)       %Gravity influence
hold on
% Plot GC curve
f = @Ms_1Spring;
Ms = f(x,alpha_r);
plot(alpha_d, Ms, "Color","r","LineStyle","--")       %Spring moment
grid on
%Change appearance
xlabel("Angle alpha / degrees")
ylabel("Torque / Nmm")

legend("Gravity Torque", "Counter Tourque","Location","best")
hold off
