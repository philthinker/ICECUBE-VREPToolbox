function [ x ] = canonicalSystem( obj )
%canonicalSystem The canonical system of DMP
%   x: M x 1;
%   @DMP2

dt = obj.dt;
tau = obj.tau;
alpha = obj.alphax;

T = 1/tau;
M = floor(tau/dt);
x = zeros(M,1);
dx = zeros(M,1);
x(1) = 1;

for i = 1:M-1
    dx(i) = -T*alpha*x(i);
    x(i+1) = x(i) + dx(i)*dt;
end

end

