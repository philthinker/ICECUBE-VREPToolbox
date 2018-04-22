function [ value_update ] = eval_comp( evals )
% Compute the value of a particular state
% evals: 1 x 2, all the evaluations
% value_update: The updated value of a particular state

usr_cmd = evals(end);   % By default, the end one is the user's evaluation
delta_cost = evals(1);
if usr_cmd <= 0 % Necessary
    if delta_cost <0
        % Necessary but Inefficient
        value_update = 1 - abs(delta_cost)*0.05;
    else
        % Necessary and efficient
        value_update = 1 + abs(delta_cost)*0.1;
    end
else    % Unnecessary
    if delta_cost <0
        % Unnecessary and inefficient
        value_update = -2;
    else
        % Unnecessary but efficient
        value_update = -abs(delta_cost)*0.05;
end

end

