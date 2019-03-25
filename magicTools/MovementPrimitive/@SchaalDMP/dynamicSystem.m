function [ Y,fx ] = dynamicSystem( obj,x )
%dynamicSystem The dynamic system of DMP
%   @SchaalDMP
%   Y: M x 3, positions,velocites and accelerations
%   fx: M x 1, forcing signals

% Dynamic or transformed system
% tau * y = z
% tau * z = K * (g-y) - D * z - K * (g-y0) * x + K * f(x)

tau = obj.tau;
K = obj.K;
D = obj.D;
dt = obj.dt;
g = obj.g;
y0 = obj.y0;

T = 1/tau;
M = size(x,1);
y = zeros(M,1);
dy = zeros(M,1);
ddy = zeros(M,1);
z = zeros(M,1);
dz = zeros(M,1);

fx = obj.forcingFunc(x);
y(1) = obj.y0;

for i=1:M-1
    dz(i) = T*(K*(g - y(i))-D*z(i)-K*(g-y0)*x(i)+K*fx(i));
    dy(i) = T*z(i);
    ddy(i) = T*dz(i);
    z(i+1) = z(i) + dt*dz(i);
    y(i+1) = y(i) + dt*dy(i);
end
dz(M,:) = T*(K*(g - y(M,:))-D*z(M,:)-K*(g-y0)*x(M,:)+K*fx(M,:));
dy(M,:) = T*z(M,:);
ddy(M,:) = T*dz(M,:);


Y = [y,dy,ddy];

end

