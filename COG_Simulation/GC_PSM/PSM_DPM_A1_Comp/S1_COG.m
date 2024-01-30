function [ Y ] = S1_COG(alpha)
% Geometric constant parameters
L0 = 100;
%CS 1 relative to CS 0
x_01 = L0; z_01=0;
O_01 = [x_01 z_01];

% S1 relative to CS 1:
x_s1 = 25.62; z_s1 = 77.91;    %x_s1 = 25.69; z_s1 = 78.1;
S1_1=[-x_s1 z_s1];

% Rotation matrix
R = [cosd(alpha) sind(alpha); 
    -sind(alpha) cosd(alpha)];

Y = O_01' + R * S1_1';
end