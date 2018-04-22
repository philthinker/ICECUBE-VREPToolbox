function [ optimized_points,reduced_indices, selected_indices] = Softmax_points_sel( points )
%SOFTMAX_POINTS_SEL the points selection strategy
% points: N x 13, the trajectory to optimize
% optimized_points: N1 x 13, the trajectory to follow
% selected_indices: the indices of selected points
% reduced_indices: the indices of reduced points
% 2018.01.20
% Haopeng Hu

% points:[x,y,z,eul_z,eul_y,eul_x,value,joint1,...joint6]
% Note that the first and last point are precluded.
temp_points = points(2:end-1,:);
% v_mean = mean(temp_points(:,7));
v_mean = median(temp_points(:,7));
p_v = probability_assignment0121(temp_points(:,7)-v_mean,2.0);

optimized_indices = binornd(ones(size(p_v)),p_v);
optimized_points = temp_points(optimized_indices==1,:);
optimized_points = [points(1,:);optimized_points;points(end,:)];

temp_indices = (1:size(points,1))';
optimized_indices = [1;optimized_indices;1];
reduced_indices = temp_indices(optimized_indices == 0 );
selected_indices = temp_indices(optimized_indices == 1);

end

