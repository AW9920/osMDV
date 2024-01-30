function [ Y ] = S2_COG(alpha)
% CS 2 relative to CS 0
x_02 = 50; z_02 = 152;
O_02 = [-x_02 z_02];

% S2 relative to CS 2:
x_s2 = 130.356; z_s2 = 0; %(Vernachlaessigt)
S2_2=[x_s2 -z_s2];

% Rotation matrix
R = [cosd(alpha) sind(alpha); 
    -sind(alpha) cosd(alpha)];

Y = R * O_02' + S2_2';
end