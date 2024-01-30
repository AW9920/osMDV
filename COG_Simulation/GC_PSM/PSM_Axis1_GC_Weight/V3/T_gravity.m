function [ Y ] = T_gravity(alpha)
    % Declare Coefficients
    g = 9.81;
    ls = 227;   %Changes minorly (higest value)
    m = 3.06;     %Estimated value

    Y = m * g * ls * sin(alpha);  
end