%% Clean up
clc
close all
clear

%% Global variable
g = 9.81;
m0 = 1.015;     %0.374;
m1 = 0.071;     %0.273;
m2 = 0.305;     %0.463;
m3 = 0.377;     %0.463;
m4 = 1.238;     %0.816;
m5 = 0.6;
%% Decleartion of COG vector
f0 = @S0_COG;
f1 = @S1_COG;
f2 = @S2_COG;
f3 = @S3_COG;
f4 = @S4_COG;
f5 = @S5_COG;

%% Compute position of COG of parallel mechanism at certain angle.
len = 101;
alpha = linspace(-40,40,len);
d = linspace(0,160,len);

%alpha = -40 : 5 : 40;
%d = 0 : 10 : 160;

for j = 1:len
    for i = 1:len
        S0_0((2*j-1):(2*j),i) = f0(alpha(:,i));
        S1_0((2*j-1):(2*j),i) = f1(alpha(:,i));
        S2_0((2*j-1):(2*j),i) = f2(alpha(:,i));
        S3_0((2*j-1):(2*j),i) = f3(alpha(:,i));
        S4_0((2*j-1):(2*j),i) = f4(alpha(:,i));
        S5_0((2*j-1):(2*j),i) = f5(alpha(:,i),d(:,j));
    end
    %% Computation of total COG
    x_s(j,:) = (m0.*S0_0(2*j-1,:)+m1.*S1_0(2*j-1,:)+m2.*S2_0(2*j-1,:)+m3.*S3_0(2*j-1,:)+m4.*S4_0(2*j-1,:) + m5.*S5_0(2*j-1,:))./(m0+m1+m2+m3+m4+m5);
    z_s(j,:) = (m0.*S0_0(2*j,:)+m1.*S1_0(2*j,:)+m2.*S2_0(2*j,:)+m3.*S3_0(2*j,:)+m4.*S4_0(2*j,:) + m5.*S5_0(2*j,:))./(m0+m1+m2+m3+m4+m5);
end
%% Plot movement of COG
fntS_leg = 20;
fntS_ax = 20;
fntS_lab = 20;

fig1 = figure;
fig1.Name = "Total COG x_s";
surf(alpha,d,x_s,'EdgeColor','none')
colorbar('location','northoutside')
xlabel('Joint value $q_2$\,/\,$\circ$','interpreter','latex')
ylabel('Joint value $q_3$\,/\,mm','interpreter','latex')
zlabel('$x_s$\,/\,mm','interpreter','latex')
grid on

fig2 = figure;
axes('FontSize', fntS_ax, 'NextPlot', 'add');
f = figure;
f.Position = [100 100 860 740];
fig2.Name = "Total COG z_s";
surf(alpha,d,z_s,'EdgeColor','none')
colorbar('location','northoutside')
xlabel('Joint value $q_2$\,/\,$\circ$','interpreter','latex','FontSize',fntS_lab)
ylabel('Joint value $q_3$\,/\,mm','interpreter','latex','FontSize',fntS_lab)
zlabel('${}^{O}S_{z}$\,/\,mm','interpreter','latex','FontSize',fntS_lab)
grid on
