function [ Y ] = S2_COG_MTM(q1, q2)
%t {2} w.r.t {1} 
x_12 = 45; y_12 = 0; z_12 = 0;
t1_2 = [x_12 y_12 z_12];

% S2 relative to {2}:
x_s1 = 126.57; z_s1 = 0.02; 
S2_2=[x_s1 0 z_s1];

% Rotation matrices
Ry = [cosd(q1) 0 sind(q1); 
         0       1      0;
    -sind(q1) 0 cosd(q1)];

Rz = [cosd(q2) -sind(q2) 0; 
    sind(q2) cosd(q2) 0;
          0          0       1];

% Equation of motion
Y = Ry * (t1_2' + Rz * S2_2');
end