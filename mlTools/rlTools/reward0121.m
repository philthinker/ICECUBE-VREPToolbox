function [ R ] = reward0121( cost0,cost1 )
%REWARD0121 compute the reward of each episode
% cost0: the cost of the last episode
% cost1: the cost of the current episode

% Weights of each joint (1 to 6)
% temp_weights = [5,5,3,1,1,1];
temp_weights = [1,1,1,1,1,1];
weights = temp_weights/(sum(temp_weights));
R = sum(weights.*(cost0-cost1));

% Cope with zero reward
if abs(R)<0.01 && abs(R)>0
    R = sign(R)*0.01;
elseif R == 0.0
    R = 0.01;
end 

end

