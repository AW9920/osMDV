function [ Y ] = T_gravity_MTM_ShoulderPitch(q1, q2, q3, len)
%% Global variable
g = 9.81;
m1 = 0.150;     %0.374;
m2 = 0.056;     %0.273;
m3 = 0.039;     %0.463;

m = m1 + m2 + m3;
%% Decleartion of COG vector
f1 = @S1_COG_MTM;
f2 = @S2_COG_MTM;
f3 = @S3_COG_MTM;


%% Compute position of COG of parallel mechanism at certain angle.


for k = 1:len
    for j = 1:len
        for i = 1:len
            %Index calc
            a = 1 + 3*(i-1);
            b = 3 * i;
            %Fill matrix
            S0_1(a:b,j,k) = f1(q1(:,i));
            S0_2(a:b,j,k) = f2(q1(:,i), q2(:,j));
            S0_3(a:b,j,k) = f3(q1(:,i), q2(:,j), q3(:,k));
        end
        %% Computation of total COG
        S = (m1.*S0_1 + m2.*S0_2 + m3.*S0_3)./(m1+m2+m3);
    end
end
x = S((1 + 3*([1:len]-1)),round(len/2),round(len/2));

Y = m * g * x ;  
end