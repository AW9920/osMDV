function [Y] = M_cr(x,q0)

%Variables
g = 9.81;
q1 = 0;


% Coefficients
l0 = x(1);
l1 = x(2);
phi = x(3);
m0 = x(4);
m1 = x(5);


%Counterbalance torque
Y = m0 * g * l0 * sind(q0) + m1 * g* l1 * cosd(q1 - phi) .* sind(q0);
end