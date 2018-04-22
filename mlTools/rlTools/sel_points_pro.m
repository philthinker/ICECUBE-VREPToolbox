function [ test_indices, remained_indices ] = sel_points_pro( V_table )
% Select some points from a trajectory
% V_table: N x 1, the values of each point
% test_indices: The selected indices
% remained_indices: The remained indices
% 2017.12.26

N = length(V_table);
temp_indices = (2:N-1);
temp_indices = temp_indices((V_table(2:N-1)>0)&(V_table(2:N-1)<10));    % Give up the 0-value points and high value points
N_sel = length(temp_indices);
if N_sel >=4
    temp = randperm(N_sel,floor(N_sel/2));
    test_indices = temp_indices(sort(temp))';
    remained_indices =[1, temp_indices, N]';
    for i = 1:length(temp)
        remained_indices(remained_indices == test_indices(i))=[];
    end
else
    test_indices = [];
    remained_indices =[1, temp_indices, N]';
end



end

