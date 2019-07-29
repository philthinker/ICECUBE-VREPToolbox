function [ t ] = ur5inv( g )
%UR5INV Summary of this function goes here
%   gd is the desired transformation of joint6
    t = zeros(6, 8);
    d1 = 0.089159;
    d2 = 0;
    d3 = 0;
    d4 = 0.10915;
    d5 = 0.09465;
    d6 = 0.0823;
    a1 = 0;
    a2 = -0.425;
    a3 = -0.39225;
    a4 = 0;
    a5 = 0;
    a6 = 0;
    alpha1 = pi/2;
    alpha2 = 0;
    alpha3 = 0;
    alpha4 = pi/2;
    alpha5 = -pi/2;
    alpha6 = 0;
    
    % Calculating t1
    p05 = g * [0, 0, -d6, 1]'  - [0, 0, 0, 1]';
    psi = atan2(p05(2), p05(1));
    phi = acos(d4 / sqrt(p05(2)*p05(2) + p05(1)*p05(1)));
    t(1, 1:4) = pi/2 + psi + phi;
    t(1, 5:8) = pi/2 + psi - phi;
    t = real(t);
    
    
    % Calculating t5
    cols = [1, 5];
    for i=1:length(cols)
        c = cols(i);
        T10 = inv(DH(a1, alpha1, d1, t(1,c)));
        T16 = T10 * g;
        p16z = T16(3,4);
        t5 = acos((p16z-d4)/d6);
        t(5, c:c+1) = t5;
        t(5, c+2:c+3) = -t5;
    end
    t = real(t);
    
    % Calculating t6
    cols = [1, 3, 5, 7];
    for i=1:length(cols)
        c = cols(i);
        T01 = DH(a1, alpha1, d1, t(1,c));
        T61 = inv(g) * T01;
        T61zy = T61(2, 3);
        T61zx = T61(1, 3);
        t5 = t(5, c);
        t(6, c:c+1) = atan2(-T61zy/sin(t5), T61zx/sin(t5));
    end
    t = real(t);
    
    % Calculating t3
    cols = [1, 3, 5, 7];
    for i=1:length(cols)
        c = cols(i);
        T10 = inv(DH(a1, alpha1, d1, t(1,c)));
        T65 = inv(DH(a6, alpha6, d6, t(6,c)));
        T54 = inv(DH(a5, alpha5, d5, t(5,c)));
        T14 = T10 * g * T65 * T54;
        p13 = T14 * [0, -d4, 0, 1]' - [0,0,0,1]';
        p13norm2 = norm(p13) ^ 2;
        t3p = acos((p13norm2-a2*a2-a3*a3)/(2*a2*a3));
        t(3, c) = t3p;
        t(3, c+1) = -t3p;
    end
    t = real(t);
    
    % Calculating t2 and t4
    cols = [1, 2, 3, 4, 5, 6, 7, 8];
    for i=1:length(cols)
        c = cols(i);
        T10 = inv(DH(a1, alpha1, d1, t(1,c)));
        T65 = inv(DH(a6, alpha6, d6, t(6,c)));
        T54 = inv(DH(a5, alpha5, d5, t(5,c)));
        T14 = T10 * g * T65 * T54;
        p13 = T14 * [0, -d4, 0, 1]' - [0,0,0,1]';
        p13norm = norm(p13);
        t(2, c) = -atan2(p13(2), -p13(1))+asin(a3*sin(t(3,c))/p13norm);
        T32 = inv(DH(a3, alpha3, d3, t(3,c)));
        T21 = inv(DH(a2, alpha2, d2, t(2,c)));
        T34 = T32 * T21 * T14;
        t(4, c) = atan2(T34(2,1), T34(1,1));
    end
    t = real(t);
    
    % Wrap to 2pi
    for i=1:6
        for j=1:8
            if t(i,j) < 0
                t(i,j) = t(i,j) + 2*pi;
            elseif t(i,j) > 2*pi
                t(i,j) = t(i,j) - 2*pi;
            end
        end
    end
                
                    
end

