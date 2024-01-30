function [Y] = T_g_fun(q0)
% This function describes the gravitational moment acting upon axis 0
% The information is extracted from COG calculation algorithm and fitted
% for a function with the aid of the z_s approximation algorithm

% Variables
q1 = -0;

l = 213.97;
phi = 9.1275;

m_ges = 3.6060;
g = 9.81;

%Approximated function of z_s(q1)
z = l * cosd(q1 - phi);

%Gravitational torque
Y = m_ges * g * z .* sind(q0);
end