function [ trajectory, N ] = circle_traj( center, radius, theta, step, normal )
%CIRCLE_TRAJ initialize a sequence of xyz points along a circle
% center: [x,y], circle center
% radius: the radius
% theta: [theta_initial, theta_final],define the segment of the circle
% step: the resolution
% normal: 'x','y' or 'z', the direction of the normal 
% trajectory: N x 3, the trajectory including x, y, z. Right-handed.

N = ceil(abs(theta(end)-theta(1))*radius/step);
% trajectory = zeros(N,1);
thetas = linspace(theta(1),theta(end),N)';
if normal == 'x'
    trajectory = center + radius*[zeros(N,1), cos(thetas), sin(thetas)];
elseif normal == 'y'
    trajectory = center + radius*[cos(thetas), zeros(N,1), sin(thetas)];
elseif normal == 'z'
    trajectory = center + radius*[cos(thetas), sin(thetas), zeros(N,1)];
else
    trajectory = center + zeros(N,3);
end

end

