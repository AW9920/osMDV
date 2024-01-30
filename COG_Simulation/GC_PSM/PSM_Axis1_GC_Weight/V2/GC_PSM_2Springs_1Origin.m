%% Clean up
clc
close all
clear

%% Define Parameters
alpha_d = linspace(-45,45,1000);
alpha_r = alpha_d .* (pi/180);
% Global Variables
L0 = 46;    % Unloaded Spring length
l = 120;
Hu = 50; Ho = 400;
ku = 0; ko = 10;

%Starting vlaues
H = 250;
k0 = 0.75;

%Starting values Array
% x0=[l H k0];   %Units [mm mm N/mm]
x0=[H k0];   %Units [mm N/mm]
% x0=[k0];   %Units [N/mm]

%% Function to fit
T_func = @T_gravity;
T = T_func(alpha_r);

%% Coefficient approximation
x = lsqcurvefit(@Ms_2Springs_1Origin, x0, alpha_r, T,[Hu,ku],[Ho,ko])

%% --------------Compare both functions-------------------
%Plot Gravity imposed torque
figure
plot(alpha_d,T)       %Gravity influence
hold on
% Plot GC curve
f = @Ms_2Springs_1Origin;
Ms = f(x,alpha_r);
plot(alpha_d, Ms, "Color","r")       %Spring moment
grid on
%Change appearance
xlabel("Angle alpha / degrees")
ylabel("Torque / Nmm")

legend("Gravity Torque", "Counter Tourque","Location","best")
hold off
