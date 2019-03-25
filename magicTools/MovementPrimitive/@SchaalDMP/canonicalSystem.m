function [ x ] = canonicalSystem( obj )
%canonicalSystem The canonical system of DMP
%   x: M x 1;
%   @SchaalDMP

% tau * dx = -alphax * x, x0 = 1

dt = obj.dt;
tau = obj.tau;
alpha = obj.alphax;

M = floor(tau/dt);
x = zeros(M,1);
dx = zeros(M,1);
x(1) = 1;

for i = 1:M-1
    dx(i) = -(1/tau)*alpha*x(i);
    x(i+1) = x(i) + dx(i)*dt;
end

end

