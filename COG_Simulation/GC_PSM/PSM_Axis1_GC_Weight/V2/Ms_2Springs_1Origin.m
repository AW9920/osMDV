function [ Y ] = Ms_2Springs_1Origin(x,alpha)
%Function describing the spring induced moment as a function of alpha
%   The function is used for approximation of a curve of the gravity
%   induced torque as a function of alpha. The maximum range for alpha is
%   45 degrees.
%% Variable definition
% Function variables
L0 = 46;    % Unloaded Spring length
l = 120;

% Coefficients
% l = x(1); H = x(2); k = x(3);
H = x(1); k = x(2);
% k = x(1);

%% Equation decleration
lk1 = sqrt(H^2 + (l/2)^2 - H*l*sin(alpha));

lk2 = sqrt(H^2 + (l/2)^2 + H*l*sin(alpha));

gamma1 = atan(((l/2) * cos(alpha)) ./ (H - (l/2)*sin(alpha)));

gamma2 = atan(((l/2) * cos(alpha)) ./ (H + (l/2)*sin(alpha)));

%% Function to fit
Y = -k*(l/2)*((lk1-L0).*cos(gamma1-alpha)-(lk2-L0).*cos(gamma2-alpha));

end