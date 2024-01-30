function [Y] = Z_s_Calc(alpha,d_select)

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
len = size(alpha,2);
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

Y = z_s(d_select,:);

% %% Plot movement of COG
% fig1 = figure;
% fig1.Name = "Total COG x_s";
% surf(alpha,d,x_s)
% grid on
% fig2 = figure;
% fig2.Name = "Total COG z_s";
% surf(alpha,d,z_s)
% grid on

end