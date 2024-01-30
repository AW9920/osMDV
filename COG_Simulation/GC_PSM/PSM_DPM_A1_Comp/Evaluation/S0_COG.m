function [ Y ] = S0_COG(alpha)

% S0 relative to CS 0:
x_s0 = 10.979; z_s0 = 3.734;
S0_0=[-x_s0 -z_s0];

% Rotation matrix
R = [cosd(alpha) sind(alpha); 
    -sind(alpha) cosd(alpha)];

Y = R * S0_0';

end