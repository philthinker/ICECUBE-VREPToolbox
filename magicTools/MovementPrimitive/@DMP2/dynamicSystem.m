function [ Y,fx ] = dynamicSystem( obj,x )
%dynamicSystem The dynamic system of DMP
%   @DMP2
%   Y: M x 3, positions,velocites and accelerations
%   fx: M x 1, forcing signals


tau = obj.tau;
alpha = obj.alpha;
beta = obj.beta;
dt = obj.dt;
g = obj.g;

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
    dz(i) = T*(alpha*(beta*(g - y(i))-z(i))+fx(i));
    dy(i) = T*z(i);
    ddy(i) = T*dz(i);
    z(i+1) = z(i) + dt*dz(i);
    y(i+1) = y(i) + dt*dy(i);
end
dz(M,:) = T*(alpha*(beta*(g' - y(M,:))-z(M,:))+fx(M,:));
dy(M,:) = T*z(M,:);
ddy(M,:) = T*dz(M,:);

Y = [y,dy,ddy];

end

