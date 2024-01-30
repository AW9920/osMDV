function [ Y ] = MTM_Ms_Weights_Eval(q1)
    %Declare parameters
    g = 9.81;       %gravitational acceleration

    %Declare coefficients
    m = 0.317;
    l = 72.50;
    phi = 8.79;
    
    Y = m * g * l * cosd(q1' + phi);
end