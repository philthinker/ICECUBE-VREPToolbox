function [ trajOut ] = ur5JointTrajMend( trajIn )
%ur5JointTrajMend Mend the obtained trajectory by interpolation
%   trajIn: M x 6, joint trajectory input
%   trajout: K x 6, joint trajectory output, K <= M
% Here we assume that the issue points in trajIn is the ones include at
% least one NaN

%% Get rid of the first NaNs brought by void buffer

i = 1; MAX = size(trajIn,1);
while all(isnan(trajIn(i,:)))
    i = i + 1;
    if i > MAX
        % The last row is still all NaNs
        break;
    end
end
if i <= MAX
    % There is at least one row useful
    trajOut = trajIn(i:end,:);
else
    % All the rows in trajIn are NaNs
    trajOut = zeros(1,6);
    return;
end
    
%% Mend the NaNs in trajIn

% Linear interpolation 
MAX = size(trajOut,1);
% Deal with the each joint one by one.
for i = 1:6
    
    % Deal with the first row
    if isnan(trajOut(1,i))
        if MAX < 3
            % There is not enough data available
            trajOut = zeros(1,6);
            return;
        elseif isnan(trajOut(2,i)) || isnan(trajOut(3,i))
            % The second or third point is NaN
            trajOut = zeros(1,6);
            return;
        else
            % There is at least 3 rows
            trajOut(1,i) = 2*trajOut(2,i) - trajOut(3,i);
        end
    end
    
    % Deal with the remaining rows
    for k = 2 : MAX-1
        if isnan(trajOut(k,i))
            % There is a NaN
            if isnan(trajOut(k+1,i))
                % Oh no, the next one is also NaN
                trajOut = zeros(1,6);
                return;
            else
                % Linear interpolation
                trajOut(k,i) = (trajOut(k-1,i) + trajOut(k+1,i))/2;
            end
        end
    end
    
    % Deal with the last row
    if isnan(trajOut(MAX,i))
        % There is a NaN in the last row
        trajOut(MAX,i) = 2*trajOut(MAX-1,i) - trajOut(MAX-2,i);
    end
end

end

