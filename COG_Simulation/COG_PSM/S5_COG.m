function [ Y ] = S5_COG(alpha,d)
% Geometric constant parameters    
L1 =290.816;
% CS 2 relative to CS 0
x_02 = 50; z_02 = 152;
O_02 = [-x_02 z_02];

% CS 4 relative to CS 2
x_24 =L1; z_24 = 0;
O_24 = [x_24 z_24];

% CS 5 relative to CS 4
x_45 =6.15; z_45 = 406.9-d;
O_45 = [x_45 z_45];

% S4 relative to CS 4:
x_s5 = 35.908; z_s5 = 139.456;
S5_5=[x_s5 -z_s5];

% Rotation matrix
R = [cosd(alpha) sind(alpha); 
    -sind(alpha) cosd(alpha)];

Y = O_24' + R*(O_02' + O_45' + S5_5');
end