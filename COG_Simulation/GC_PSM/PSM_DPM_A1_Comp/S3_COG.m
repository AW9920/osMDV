function [ Y ] = S3_COG(alpha)
% CS 3 relative to CS 0
x_03 = 50; z_03 = 222;
O_03 = [-x_03 z_03];

% S3 relative to CS 3:
x_s3 = 126.718; z_s3 = 0;   %(Vernachlaessigt)
S3_3=[x_s3 z_s3];

% Rotation matrix
R = [cosd(alpha) sind(alpha); 
    -sind(alpha) cosd(alpha)];

Y = R * O_03' + S3_3';
end