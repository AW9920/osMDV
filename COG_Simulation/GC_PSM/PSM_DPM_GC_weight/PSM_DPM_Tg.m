function [Y] = PSM_DPM_Tg(alpha)
    g = 9.81;
    m0 = 1.015;     %0.374;
    m1 = 0.071;     %0.273;
    m2 = 0.305;     %0.463;
    m3 = 0.377;     %0.463;
    m4 = 1.238;     %0.816;
    m5 = 0.6;
    
    F0 = m0*g; 
    F2 = m2*g; 
    F3 = m3*g; 
    F4 = m4*g; 
    F5 = m5*g;
    
    L0 = 100;
    L1 = 290.816;
    ls3 = 130.356;
    % CS 2 relative to CS 0
    x_02 = 50; z_02 = 152;
    % CS 3 relative to CS 0
    x_03 = 50; z_03 = 222;
    % S0 relative to CS 0:
    x_s0 = 10.979; z_s0 = 3.734;

    %% Compute forces on Link0
    F1R = (-F2*(ls3-L0)-(F3/2+F4+F5)*(L1-L0))/L0;
    %F2R = (F2*ls3+(F3/2+F4+F5)*L1)/L0;

    l_02 = z_02*sin((pi*alpha)/180) - x_02*cos((pi*alpha)/180);

    l_03 = z_03*sin((pi*alpha)/180) - x_03*cos((pi*alpha)/180);
    
    l_s0 = - (x_s0*cos((pi*alpha)/180)) - (z_s0*sin((pi*alpha)/180));

    Y = F1R * l_02' + F3/2 * l_03' + F0*l_s0';
end