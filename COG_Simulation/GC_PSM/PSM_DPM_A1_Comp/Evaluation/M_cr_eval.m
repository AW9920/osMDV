function [Y] = M_cr_eval(q1,q2)

%Variables
g = 9.81;

% Coefficients
l0 = 200.28;
l1 = 150.10;
phi = 7.95;
m0 = 2.63;
m1 = 1.65;


%Counterbalance torque
Y = m0 * g * l0 * sind(q1) + m1 * g* l1 * cosd(q2 - phi) .* sind(q1);
end