function [ optimized_points ] = trajectory_update0121( points )
%TRAJECTORY_UPDATE0121 update the trajectory

% optimized_indices = ones(1,length(optimized_points));
optimized_indices = points(:,7)>0;
optimized_points = points(optimized_indices,:);

end

