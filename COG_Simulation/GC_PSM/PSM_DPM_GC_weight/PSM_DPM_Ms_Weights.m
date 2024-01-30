function [Y] = PSM_DPM_Ms_Weights(x,alpha)
    %Declare parameters
    g = 9.81;       %gravitational acceleration

    %Declare coefficients
    m = x(1);       %mass per weight
    x_cr = x(2); 
    z_cr = x(3);
    
    x_cr_a = cos((pi*alpha)/180)*x_cr - sin((pi*alpha)/180)*z_cr;

    Y = m * g * x_cr_a;
end