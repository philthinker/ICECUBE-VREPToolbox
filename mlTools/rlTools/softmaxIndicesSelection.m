function [ selectedIndices, reducedIndices, biIndices ] = softmaxIndicesSelection( points )
%softmaxIndicesSelection Select points based on their value
% points: N x 1, points of values
% selectedIndices: Indices selected
% reducedIndices: Indices not selected
% biIndices: The logic indices, 1 for selected indices, 0 for reduced ones

% Haopeng Hu
% 2018.05.02

pPoints = probability_assignment0121(points - median(points),2.0);
biIndices = binornd(ones(size(pPoints)),pPoints);
tempIndices = (1:size(points,1))';
selectedIndices = tempIndices(biIndices == 1);
reducedIndices = tempIndices(biIndices == 0);

end

