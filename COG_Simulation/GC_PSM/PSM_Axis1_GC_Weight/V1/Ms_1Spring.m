function [ Y ] = Ms_1Spring(x,alpha)
    %Fixed Parameters
    d = 110;
%     H = 250;
    L0 = 31;

    %Approximated Parameters
%     d = x(1); H = x(2); k = x(3);
    H = x(1); k = x(2);
%     k = x(1);
    
    %Approximation Algorithm
    lk = sqrt(H^2 + d^2 - 2*H*d*cos(alpha));
    
    gamma = asin((sin(alpha)*d) ./ sqrt(H^2 + d^2 - 2*H*d*cos(alpha)));
    
    Y = k * d * (lk-L0) .* sin(gamma + alpha);
end