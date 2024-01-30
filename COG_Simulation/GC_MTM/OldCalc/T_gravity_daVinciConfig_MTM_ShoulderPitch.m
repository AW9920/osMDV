function [ Y ] = T_gravity_daVinciConfig_MTM_ShoulderPitch(alpha)
    % Declare Coefficients
    g = 9.81;       %gravitational acceleration

    %% Values for updated design Concept1
    g = 9.81;
    m = 0.569;      %Entire weight of MTM arm from SP axis
    % Position of COG
    lsy = 8.958;      %Distance COG arm to SP axis in zero pos horicontal
    lsx = 53.962;     %Distance COG arm to SP axis in zero pos vertical
    
    %% Function of gravitational influence
    Y = m * g * (cos(alpha) * lsy - sin(alpha) * lsx);  
end