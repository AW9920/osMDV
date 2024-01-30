function [Y] = PSM_DPM_Mspring(alpha)
    k = 0.2515; %N/mm
    l0 = 36;

    H = 70;
    L0 = 100;

    % CS 2 relative to CS 0
    x_02 = 50; z_02 = 152;
    % CS 3 relative to CS 0
    x_03 = 50; z_03 = 222;

    l_02x = z_02*sind(alpha) - x_02*cosd(alpha);
    l_03x = z_03*sind(alpha) - x_03*cosd(alpha);

    l_02z = x_02*sind(alpha) + z_02*cosd(alpha);
    l_03z = x_03*sind(alpha) + z_03*cosd(alpha);

    gamma = asind(H*cosd(alpha)./(sqrt(H^2+L0^2 - 2*H*L0*sind(alpha))));
    eps = asind(H*cosd(alpha)./(sqrt(H^2+L0^2 + 2*H*L0*sind(alpha))));

    lk1 = sqrt(H^2 + L0^2 - 2*H*L0*sind(alpha));
    lk2 = sqrt(H^2 + L0^2 + 2*H*L0*sind(alpha));

    if(lk1 >= l0)
        dl1 = lk1 - l0;
    else 
        dl1 = 0;
    end
    
    if(lk2 >= l0)
        dl2 = lk2 - l0;
    else
        dl2 = 0;
    end

    Y = 2 * k * H * cosd(alpha) .* (dl1 - dl2);
    %Y = 2 * k .* (dl1 .* (cosd(gamma) .* l_03z + sind(gamma) .* l_03x) + dl2 .* (cosd(eps) .* l_02z + sind(eps) .* l_02x));
end