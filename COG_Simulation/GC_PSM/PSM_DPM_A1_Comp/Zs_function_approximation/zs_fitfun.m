function [Y] = zs_fitfun(x,alpha)
    % Coefficients
    l = x(1);
    a = x(2);
    
    %Approximated function of z_s(alpha)
    Y = l*cosd(alpha - a);
end