function [ Y ] = S4_COG(alpha)
% Geometric constant parameters     
L1 =290.816;
% CS 2 relative to CS 0
x_02 = 50; z_02 = 152;
O_02 = [-x_02 z_02];

% CS 4 relative to CS 2
x_24 =L1; z_24 = 0;
O_24 = [x_24 z_24];

% S4 relative to CS 4:
x_s4 = 7; z_s4 = 192.37;
S4_4=[-x_s4 z_s4];

% Rotation matrix
R = [cosd(alpha) sind(alpha); 
    -sind(alpha) cosd(alpha)];

Y = O_24' + R*(O_02' + S4_4');
end