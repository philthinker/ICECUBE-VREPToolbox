function [ optimized_points ] = value_update0121( user_cmd,R,points,reduced_indices, selected_indices,alpha )
%VALUE_UPDATE0121 value update strategy
% user_cmd: 0 or 1
% R: the reward

% alpha = [0.5,0.8,0.5,0.5];
optimized_points = points;
if user_cmd ==1
    optimized_points(selected_indices,7) = ...
        points(selected_indices,7)+alpha(1)*R;
    optimized_points(reduced_indices,7) = ...
        points(reduced_indices,7)-alpha(2)*R;
elseif user_cmd ==0
    optimized_points(selected_indices,7) = ...
        points(selected_indices,7) +alpha(3)*R;
    optimized_points(reduced_indices,7) = ...
        points(reduced_indices,7) +alpha(4)*phi0123(R,1.0);
end

end

