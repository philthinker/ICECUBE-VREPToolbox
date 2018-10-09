function [ trajOut ] = slideWindowAverage( trajIn, width )
%slideWindowAverage Filt a trajectory with a slide window
% trajIn: N x M, the original trajecotry.
% width: Integer, the width of the window.
% trajOut: N1 x M, the filted trajectory.

width = floor(width);
N = size(trajIn,1);
if N <= width
    trajOut = mean(trajIn,1);
else
    trajOut = zeros(ceil(N/width),size(trajIn,2));
    i = 1;j = 1;
    while i+width <= N
        % Slide window
        trajOut(j,:) = mean(trajIn(i:i+width-1,:),1);
        i = i + width;
        j = j+ 1;
    end
    trajOut(j,:) = mean(trajIn(i:end,:),1);
end

end

