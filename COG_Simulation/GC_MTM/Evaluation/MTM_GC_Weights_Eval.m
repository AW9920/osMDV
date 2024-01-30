%% Clean up
clc
close all
clear

%% Define Parameters
len = 21;
q1 = linspace(-40,40,len);
%% Observations
% % 1
% q2 = linspace(0,0,len);
% q3 = linspace(0,0,len);
% 2
q2 = linspace(0,0,len);
q3 = linspace(-60,-60,len);
% % 3
% q2 = linspace(30,30,len);
% q3 = linspace(0,0,len);
% % 4
% q2 = linspace(-20,-20,len);
% q3 = linspace(60,60,len);

%% Functions
T_func = @T_gravity_MTM_ShoulderPitch;
T = T_func(q1,q2,q3,len);

M_func = @MTM_Ms_Weights_Eval;
M = M_func(q1);

%% Determine mean square error
RMSE = sqrt(sum((M-T).^2)/len)
E_rel = M-T;

%% --------------Compare both functions-------------------
%Setup figure
f = figure;
f.Position = [100,100,550,500];
hold on
grid on
box off
yyaxis left
ax1 = gca;
ax1.YColor = 'k';

% Plot Gravitational torque
plot(q1,T,'-x','MarkerSize',8)       %Gravity influence
% Plot counter torque
plot(q1, M,'-o', "Color","r")       %Spring moment
xlim([-40 40])
ylim('auto')
xlabel('$q_1$\,/\,$\circ$','Interpreter','latex')
ylabel('$Torque$\,/\,Nmm','Interpreter','latex')

%Setup second axis
yyaxis right
ax2 = gca;
ax2.YColor = 'k';
ylabel('Absolute error $f_{abs}$\,/\,Nmm','Interpreter','latex')
% ylim([-7.5 -4.5])
% ylim('auto')
ylim([floor(min(E_rel))-1 ceil(max(E_rel))])
% Plot relative error
plot(q1, E_rel,'--',"Color","c")
%Change appearance


legend(["Gravitational torque $T(q_1, q_2=30^\circ, q_3=0^\circ)$", ...
    "Counter torque $M_{cr}(q_1)$", ...
    "Absolute error $f_{abs}$ between $M_{cr}$ and $T$"],'Interpreter','latex',"Location","northoutside")
hold off
