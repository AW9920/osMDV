function [ Y ] = MTM_Ms_Weights_daVinciConfig(x,alpha)
    %% Declare parameters
    g = 9.81;       %gravitational acceleration
    Rx = 60;

    %% Declare coefficients
    m = x(1);       %mass per weight
    Ry = x(2);
    %% Function of counter weight
    Y = m * g * (Rx * sin(alpha) - Ry * cos(alpha));
end