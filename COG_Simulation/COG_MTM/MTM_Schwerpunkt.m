%% Clean up
clc
close all
clear

%% Global variable
g = 9.81;
m1 = 0.150;     %0.374;
m2 = 0.056;     %0.273;
m3 = 0.039;     %0.463;

%% Decleartion of COG vector
f1 = @S1_COG_MTM;
f2 = @S2_COG_MTM;
f3 = @S3_COG_MTM;


%% Compute position of COG of parallel mechanism at certain angle.
len = 41;

q1 = linspace(-45,45,len);
q2 = linspace(-40,40,len);
q3 = linspace(-60,60,len);

for k = 1:len
    for j = 1:len
        for i = 1:len
            %Index calc
            a = 1 + 3*(i-1);
            b = 3 * i;
            %Fill matrix
            S0_1(a:b,j,k) = f1(q1(:,i));
            S0_2(a:b,j,k) = f2(q1(:,i), q2(:,j));
            S0_3(a:b,j,k) = f3(q1(:,i), q2(:,j), q3(:,k));
        end
        %% Computation of total COG
        S = (m1.*S0_1 + m2.*S0_2 + m3.*S0_3)./(m1+m2+m3);
    end
end


%% Plot movement of COG
% Data extraction
x1 = S((1 + 3*([1:len]-1)),:,round(len/2));
% Data plot x as function of q1 and q2; q3=0
fig1 = figure;
fig1.Name = "Total MTM COG x-component for variable q1 and q2";
surf(q2,q1,x1,'EdgeColor','none')
colorbar('location','northoutside')
xlabel('Joint value $q_2$\,/\,$\circ$','interpreter','latex')
ylabel('Joint value $q_1$\,/\,$\circ$','interpreter','latex')
zlabel('$S_x(q_1,q_2)$\,/\,mm','interpreter','latex')
grid on


% Data extraction
x2 = S((1 + 3*([1:len]-1)),round(len/2),:);
x2 = reshape(x2, [len, len]);
% Data plot x as function of q1 and q3, q2=0
fig2 = figure;
fig2.Name = "Total MTM COG x-component for variable q1 and q3";
surf(q3,q1,x2,'EdgeColor','none')
colorbar('location','northoutside')
xlabel('Joint value $q_3$\,/\,$\circ$','interpreter','latex')
ylabel('Joint value $q_1$\,/\,$\circ$','interpreter','latex')
zlabel('$S_x(q_1,q_3)$\,/\,mm','interpreter','latex')
grid on
