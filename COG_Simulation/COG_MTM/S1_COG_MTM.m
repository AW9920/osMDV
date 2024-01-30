function [ Y ] = S1_COG_MTM(q1)
% S1 relative to {1}:
x_s0 = 23.67; z_s0 = 2.40;
S1_1=[x_s0  0  z_s0];

% Rotation matrix y axis
Ry = [cosd(q1) 0 sind(q1); 
         0       1      0;
    -sind(q1) 0 cosd(q1)];


Y = Ry * S1_1';
end