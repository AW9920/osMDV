%% Clean up
clc
close all
clear

%% Global variable
alpha = -40 : 1 : 40;

% Upper and lower boudaries
mu = 0; mo = 5;
l_crxu = 0; l_crxo = 200;
l_crzu = 0; l_crzo = 200;

m = 0.1;
l_crx = 80;l_crz = 80;

x0=[m, l_crx, l_crz];

%% Compute gravitational torque
T_g = @PSM_DPM_Tg;
T = T_g(alpha);

%% Compute spring counter torque
M_spring = @PSM_DPM_Mspring;
M_s = M_spring(alpha);

T = T + M_s';

%% Coefficient approximation
x = lsqcurvefit(@PSM_DPM_Ms_Weights, x0, alpha', T, [mu, l_crxu, l_crzu], [mo, l_crxo,l_crzo])

%% Plot data
%Plot Gravity imposed torque
figure
plot(alpha',T)       %Gravity influence
hold on
% Plot GC curve
f = @PSM_DPM_Ms_Weights;
Ms = f(x,alpha);
plot(alpha, Ms, "Color","r","LineStyle","--")       %Spring moment
grid on
%Change appearance
xlabel("Angle alpha / degrees")
ylabel("Torque / Nmm")

legend("Gravity Torque", "Counter Tourque","Location","best")
hold off



