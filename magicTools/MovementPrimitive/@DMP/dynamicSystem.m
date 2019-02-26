function [ y,dy,ddy ] = dynamicSystem( obj, x )
%dynamicSystem The dynamic system of DMP
%   x: M x 1;
%   y: M x D;
%   dy: M x D;
%   ddy: M x D;
%   @DMP

tau = obj.tau;
alpha = obj.alpha;
beta = obj.beta;
dt = obj.dt;
D = obj.D;
g = obj.g;

T = 1/tau;
M = size(x,1);
y = zeros(M,D);
dy = zeros(M,D);
ddy = zeros(M,D);
z = zeros(M,D);
dz = zeros(M,D);

f = obj.nonlinearTerm(x);
y(:,1) = obj.y0;

% for j = 1:D
%     for i = 1:M-1
%         dz(i,j) = T*(alpha*(beta*(g(j)-y(i,j))-z(i,j))+f(i,j));
%         dy(i,j) = T*z(i,j);
%         ddy(i,j) = T*dz(i,j);
%         z(i+1,j) = z(i,j) + dt*dz(i,j);
%         y(i+1,j) = y(i,j) + dt*dy(i,j);
%     end
% end

for i=1:M-1
    dz(i,:) = T*(alpha*(beta*(g' - y(i,:))-z(i,:))+f(i,:));
    dy(i,:) = T*z(i,:);
    ddy(i,:) = T*dz(i,:);
    z(i+1,:) = z(i,:) + dt*dz(i,:);
    y(i+1,:) = y(i,:) + dt*dy(i,:);
end

end

