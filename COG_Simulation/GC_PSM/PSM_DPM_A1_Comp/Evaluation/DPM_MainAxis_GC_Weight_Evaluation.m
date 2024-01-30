%% Clean up
clc
close all
clear

%% Define Parameters
fntleg = 12;
fntax = 12;
fntsz = 15;
len = 26;
q1 = linspace(-25,25,len);
q2 = 0;

%% Function to fit
T_fun = @T_g_fun;
T = T_fun(q1,q2);

M_fun = @M_cr_eval;
M = M_fun(q1,q2);

%% Determine mean square error
RMSE = sqrt(sum((M-T).^2)/len)
E_rel = M-T;

%% --------------Compare both functions-------------------
%Setup figure
f = figure;
f.Position = [100,100,700,600];
hold on
grid on
box off
yyaxis left
ax1 = gca;
ax1.YColor = 'k';
ax1.FontSize = fntax;

%Plot Gravity imposed torque
plot(q1,T,'-o','MarkerSize',5)       %Gravity influence
hold on
% Plot GC curve
plot(q1, M, ':x',"Color","r",'MarkerSize',8)       %Spring moment
grid on
xlim([-25 25])
%Change appearance
xlabel('Joint value $q_1$\,/\,$\circ$','Interpreter','latex','FontSize',fntsz)
ylabel('Torque\,/\,Nmm','Interpreter','latex','FontSize',fntsz)

%Setup second axis
yyaxis right
ax2 = gca;
ax2.YColor = 'k';
ax1.FontSize = fntax;
ylabel('Absolute error $f_{abs}$\,/\,Nmm','Interpreter','latex','FontSize',fntsz)
ylim([-45 45])
%ylim('auto')
% Plot relative error
plot(q1, E_rel,'--',"Color","c")
%Change appearance

legend(["Gravitational torque $T(q_1, q_2=30^\circ, q_3=0\,$mm", ...
    "Counter torque $M_{cr}(q_1,q_2)$", ...
    "Absolute error $f_{abs}$ between $M_{cr}$ and $T$"],'Interpreter','latex',"Location","southeast",'FontSize',fntleg)
hold off
