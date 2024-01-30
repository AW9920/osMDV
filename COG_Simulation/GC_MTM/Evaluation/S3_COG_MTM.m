function [ Y ] = S3_COG_MTM(q1, q2, q3)
% t {2} w.r.t {1} 
x_12 = 45; y_12=0; z_12=0;
t1_2 = [x_12 y_12 z_12];

% t {3} w.r.t {2} 
x_23 = 217.5; y_23=20.5; z_23 = 0;
t2_3 = [x_23 y_23 z_23];

% S3 relative to {3}:
x_s3 = 99.47; z_s3 = 0; %(Vernachlaessigt)
S3_3=[x_s3 0 z_s3];

% Rotation matrix y axis
Ry1 = [cosd(q1) 0 sind(q1); 
         0       1      0;
    -sind(q1) 0 cosd(q1)];

Rz = [cosd(q2) -sind(q2) 0; 
    sind(q2) cosd(q2) 0;
          0          0       1];

Ry2 = [cosd(q3) 0 sind(q3); 
         0       1      0;
    -sind(q3) 0 cosd(q3)];

R_off = [cosd(90) 0 sind(90); 
         0       1      0;
        -sind(90) 0 cosd(90)];

% Equation of motion
Y = Ry1 * (t1_2' + Rz * (t2_3' + Ry2 * R_off * S3_3'));
end