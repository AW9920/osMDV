function [Y] = T_g_fun(q1,q2)
% This function describes the gravitational moment acting upon axis 0
% The information is extracted from COG calculation algorithm and fitted
% for a function with the aid of the z_s approximation algorithm

% Variables
l = 213.97;
phi = 9.1275;

m_ges = 3.6060;
g = 9.81;

%Approximated function of z_s(q2)
z = l * cosd(q2 - phi);

%Gravitational torque
Y = m_ges * g * z .* sind(q1);
end