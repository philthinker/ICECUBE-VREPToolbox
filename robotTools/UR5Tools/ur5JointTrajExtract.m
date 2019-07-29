function [ trajOut ] = ur5JointTrajExtract( trajPosition,dt )
%ur5jointTrajExtract Extract the velocities and accelerations from joint
%positions of UR5. Assume the trajectory is zero to zero.
%   trajPosition: M x 6, the positions.
%   dt: the time step.
%   trajOut: 1 x 6 cell whose elements are M x 3 matrices.

trajOut = cell(1,6); MAX = size(trajPosition,1);
for i = 1:6
    trajOut{i} = zeros(MAX,3);
end
if MAX >= 3
    % At least 3 points are needed
    for i = 1:6
        % Deal with the joitns one by one
        trajOut{i}(:,1) = trajPosition(:,i);    %Positions
        trajOut{i}(:,2) = (circshift(trajPosition(:,1),-1) - trajPosition(:,1))/dt; % Velocities
        trajOut{i}(end,2) = 0;
        trajOut{i}(:,3) = (circshift(trajOut{i}(:,2),-1) - trajOut{i}(:,2))/dt; % Accelerations
        trajOut{i}(end,3) = 0;
    end
end

end

