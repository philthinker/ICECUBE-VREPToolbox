function [ pointsUpdated ] = trajUpdate0502( points )
%trajUpdate0502 Update the trajectory based on values
% points: N x 1, values
% pointsUpdated: indices for remaining points

logicIndices = points>0;
pointsUpdated = (1:size(points,1));
pointsUpdated = pointsUpdated(logicIndices == 1);

end

