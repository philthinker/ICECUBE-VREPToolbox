function [ TrajOut ] = DouglasPeucker( TrajIn, threshold )
%DouglasPeucker The Douglas-Peucker algorithm to optimize a curve to lines.
% TrajIn: N x M, M >= 3, the trajectory to be optimized.
% threshold: float, the DPA threshold.
% TrajOut: N1 x 3, the optimal trajectory.

% Haopeng Hu
% 2018.05.10

N = size(TrajIn,1);
criticalPoints = (1:N)';
criticalPoints(2:N-1) = 0;

segSign = true;
while segSign
    % maxDis is the maximum distance
    % Segmentation
    sIndex = 1; % Starting point of the current segment
%     eIndex = 1; % Ending point of the current segment
    segSign = false;
    for i = 2:N
        % Sweep the criticl points
        if criticalPoints(i) > 0 
            % Find a critical point
            eIndex = i;
            [maxDis, k] = max(dPoint2Line(TrajIn(sIndex:eIndex,1:3),[TrajIn(sIndex,1:3);TrajIn(eIndex,1:3)]));
%             maxDis
            if maxDis > threshold
                % Segment! - Create a new critical point
                criticalPoints(sIndex+k-1) = sIndex+k-1;
                segSign = true;
            end
            sIndex = eIndex;
        end
    end
end
TrajOut = TrajIn(criticalPoints>0,:);

end

