function [ Y ] = MTM_Ms_Weights(x,q1)
    %Declare parameters
    g = 9.81;       %gravitational acceleration

    %Declare coefficients
    m = x(1);       %mass per weight
    l = x(2);
    phi = x(3);
    
    Y = m * g * l * cosd(q1' + phi);
end